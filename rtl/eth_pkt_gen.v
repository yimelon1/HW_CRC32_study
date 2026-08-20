/*
 * eth_pkt_gen — 固定長度 Ethernet packet generator（25GbE, 64-bit MII）
 *
 * 行為對標 AMD 官方 xxv_ethernet_0_pkt_gen_mon.v 的 xxv_ethernet_0_mii_pkt_gen，
 * 但補上官方版沒有的 FCS（官方 insert_crc 寫死 1'b0），且 DA/SA/EtherType 由外部指定。
 *
 * 三種長度都選 L ≡ 4 (mod 8)，換到兩個簡化：
 *   1. (L-4) mod 8 == 0  → CRC 涵蓋範圍剛好整數個 64-bit beat
 *                        → 只需一份 DATA_WIDTH=64 的 lfsr（不是 8 份）
 *   2. START 恆落 lane 0 → 免 swap_lanes（4-byte barrel shift）
 *
 * byte order 規則（最高風險項，錯了 loopback 也不會抓到）：
 *   mii_txd[7:0] = lane 0 = 最先傳的 byte
 *   DA[47:40] 先傳（network byte order）
 *   FCS 上線是 little-endian：fcs[7:0] 先傳
 *
 * golden model：golden_fcs.py（已對上 CRC-32("123456789") = 0xCBF43926）
 */

`resetall
`timescale 1ns / 1ps
`default_nettype none

module eth_pkt_gen (
    input  wire        clk,          // tx_mii_clk, 390.625 MHz
    input  wire        rst,

    // 控制
    input  wire        trig_start,   // 上升緣觸發一輪發送
    input  wire [7:0]  trig_rounds,  // 送幾個封包（0 = 不送）
    input  wire [1:0]  pkt_n,        // 長度選擇：0=68B, 1=260B, 2=1028B, 3=保留(當0)

    // 封包欄位
    input  wire [47:0] DA,           // DA[47:40] 先傳
    input  wire [47:0] SA,           // SA[47:40] 先傳
    input  wire [15:0] eth_type,     // eth_type[15:8] 先傳

    // 64-bit MII 輸出
    output reg  [63:0] mii_txd,
    output reg  [7:0]  mii_txc,

    // 狀態
    output wire        busy,
    output reg         done
);

// ── XGMII control character（IEEE 802.3 Table 46-3）────────────
localparam [7:0]
    XGMII_IDLE  = 8'h07,
    XGMII_START = 8'hFB,
    XGMII_TERM  = 8'hFD,
    ETH_PRE     = 8'h55,
    ETH_SFD     = 8'hD5;

// preamble 一個 beat 剛好塞滿：START 取代第一個 preamble octet（§46.2.2）
localparam [63:0] PREAMBLE_BEAT =
    {ETH_SFD, {6{ETH_PRE}}, XGMII_START};   // byte0 = START = 先傳
localparam [63:0] IDLE_BEAT = {8{XGMII_IDLE}};

// ── 三種長度的 data beat 數 = (L-4)/8 ──────────────────────────
//   pkt_n=0: L=68   → (68-4)/8   = 8
//   pkt_n=1: L=260  → (260-4)/8  = 32
//   pkt_n=2: L=1028 → (1028-4)/8 = 128
localparam [7:0] BEATS_0 = 8'd8,
                 BEATS_1 = 8'd32,
                 BEATS_2 = 8'd128;

// IFG：TERM 落在 FCS beat 的 byte4，同 beat 尾端已有 3 byte IDLE。
// 下個 START 必須在 beat 邊界，故 IFG = 3 + 8*IDLE_BEATS。
//   IDLE_BEATS=1 → IFG=11 < 12 不合法
//   IDLE_BEATS=2 → IFG=19 合法
localparam [3:0] IDLE_BEATS = 4'd2;

// ── 狀態機 ────────────────────────────────────────────────────
localparam [2:0]
    ST_IDLE     = 3'd0,
    ST_LOAD     = 3'd1,
    ST_PREAMBLE = 3'd2,
    ST_DATA     = 3'd3,
    ST_FCS_TERM = 3'd4,
    ST_IFG      = 3'd5;

reg [2:0] state, state_next;

reg [47:0] da_r,  sa_r;
reg [15:0] type_r;
reg [7:0]  beat_cnt,  beat_cnt_next;    // 剩餘 data beat
reg [7:0]  beats_tot;                   // 本次長度的 data beat 總數
reg [7:0]  round_cnt, round_cnt_next;   // 剩餘封包數
reg [3:0]  idle_cnt,  idle_cnt_next;
reg [1:0]  d_idx,     d_idx_next;       // data beat 序號，飽和於 3（表示 >=3）
reg [15:0] pl_cnt,    pl_cnt_next;      // 本 beat 第一個 payload 的 index

reg [63:0] txd_next;
reg [7:0]  txc_next;
reg        done_next;
reg        crc_upd;                     // 本 cycle 是否累積 CRC
reg        crc_rst;                     // 本 cycle 是否把 CRC state 歸位

// trig_start 上升緣
reg  trig_d;
wire trig_rise = trig_start & ~trig_d;

// time-0 初始值。模擬起始時若輸出是 X，XgmiiSink 會因為無法把 LogicArray 轉 int 而炸。
// FPGA 綜合會把它變成 register 的初始狀態（Xilinx GSR），
// 與 mac_ref/rtl/axis_xgmii_tx_64.v:171 在宣告時給初始值等價。
initial begin
    mii_txd = IDLE_BEAT;
    mii_txc = {8{1'b1}};
    done    = 1'b0;
    state   = ST_IDLE;
end

assign busy = (state != ST_IDLE);

// ── CRC-32：只需一份 64-bit unrolled LFSR ─────────────────────
// 參數與 mac_ref/rtl/axis_xgmii_tx_64.v:192-201 相同，只改 DATA_WIDTH
reg  [31:0] crc_state;
wire [31:0] crc_state_next;
wire [31:0] fcs = ~crc_state;           // FCS 取反相（對照 tx_64.v:290）

lfsr #(
    .LFSR_WIDTH(32),
    .LFSR_POLY(32'h4c11db7),
    .LFSR_CONFIG("GALOIS"),
    .LFSR_FEED_FORWARD(0),
    .REVERSE(1),
    .DATA_WIDTH(64),
    .STYLE("AUTO")
)
eth_crc (
    .data_in(txd_next),                 // 與輸出同一份資料，同步進 CRC
    .state_in(crc_state),
    .data_out(),
    .state_out(crc_state_next)
);

// ── 三種 beat 的內容（純組合）──────────────────────────────────
// data beat 1：DA 全 6 byte + SA 前 2 byte
wire [63:0] beat_hdr1 = {
    sa_r[39:32], sa_r[47:40],                                             // byte 7,6
    da_r[ 7: 0], da_r[15: 8], da_r[23:16], da_r[31:24], da_r[39:32], da_r[47:40]
};                                                                        // byte 5..0，byte0 先傳

// data beat 2：SA 後 4 byte + EtherType + payload[0:1]
//   14-byte header 不是 8 的倍數，所以這是唯一的混合 beat
wire [63:0] beat_hdr2 = {
    8'd1, 8'd0,                                                           // byte 7,6 = payload[1],[0]
    type_r[7:0], type_r[15:8],                                            // byte 5,4
    sa_r[7:0], sa_r[15:8], sa_r[23:16], sa_r[31:24]                       // byte 3..0
};

// data beat >=3：純 payload，遞增 pattern（payload[j] = j[7:0]）
wire [7:0] pl = pl_cnt[7:0];
wire [63:0] beat_payload = {
    pl + 8'd7, pl + 8'd6, pl + 8'd5, pl + 8'd4,
    pl + 8'd3, pl + 8'd2, pl + 8'd1, pl + 8'd0                            // byte0 = pl，先傳
};

// FCS beat：FCS(4, little-endian) + TERM + IDLE x3
wire [63:0] beat_fcs = {XGMII_IDLE, XGMII_IDLE, XGMII_IDLE, XGMII_TERM, fcs};

// 依 pkt_n 選 data beat 總數
wire [7:0] beats_sel = (pkt_n == 2'd1) ? BEATS_1 :
                       (pkt_n == 2'd2) ? BEATS_2 : BEATS_0;

// ── 組合：次態與輸出 ──────────────────────────────────────────
always @* begin
    state_next     = state;
    beat_cnt_next  = beat_cnt;
    round_cnt_next = round_cnt;
    idle_cnt_next  = idle_cnt;
    d_idx_next     = d_idx;
    pl_cnt_next    = pl_cnt;
    done_next      = 1'b0;
    crc_upd        = 1'b0;
    crc_rst        = 1'b0;

    // 預設輸出 IDLE（沒被覆蓋的 cycle 自動是 IDLE）
    txd_next = IDLE_BEAT;
    txc_next = {8{1'b1}};

    case (state)
        ST_IDLE: begin
            if (trig_rise) state_next = ST_LOAD;
        end

        ST_LOAD: begin
            // trig_rounds = 0 → 不送，直接回 IDLE
            state_next = (trig_rounds == 8'd0) ? ST_IDLE : ST_PREAMBLE;
        end

        ST_PREAMBLE: begin
            txd_next      = PREAMBLE_BEAT;
            txc_next      = 8'b0000_0001;      // 只有 lane 0 是 control(START)
            crc_rst       = 1'b1;              // preamble 不進 CRC，state 歸位
            beat_cnt_next = beats_tot;
            d_idx_next    = 2'd1;
            pl_cnt_next   = 16'd0;
            state_next    = ST_DATA;
        end

        ST_DATA: begin
            case (d_idx)
                2'd1:    txd_next = beat_hdr1;
                2'd2:    txd_next = beat_hdr2;
                default: txd_next = beat_payload;
            endcase
            txc_next = 8'h00;                  // 全部是 data
            crc_upd  = 1'b1;

            if (d_idx < 2'd3) d_idx_next = d_idx + 2'd1;

            // beat2 用掉 payload[0:1]，所以 beat3 從 index 2 起；之後每 beat +8
            if (d_idx == 2'd2)      pl_cnt_next = 16'd2;
            else if (d_idx > 2'd2)  pl_cnt_next = pl_cnt + 16'd8;

            beat_cnt_next = beat_cnt - 8'd1;
            if (beat_cnt == 8'd1) state_next = ST_FCS_TERM;
        end

        ST_FCS_TERM: begin
            txd_next      = beat_fcs;
            txc_next      = 8'hF0;             // byte4=TERM, byte5..7=IDLE 都是 control
            idle_cnt_next = IDLE_BEATS;
            state_next    = ST_IFG;
        end

        ST_IFG: begin
            // 輸出維持預設的全 IDLE
            idle_cnt_next = idle_cnt - 4'd1;
            if (idle_cnt == 4'd1) begin
                if (round_cnt > 8'd1) begin
                    round_cnt_next = round_cnt - 8'd1;
                    state_next     = ST_PREAMBLE;
                end else begin
                    done_next  = 1'b1;
                    state_next = ST_IDLE;
                end
            end
        end

        default: state_next = ST_IDLE;
    endcase
end

// ── 時序 ──────────────────────────────────────────────────────
always @(posedge clk) begin
    if (rst) begin
        state     <= ST_IDLE;
        mii_txd   <= IDLE_BEAT;
        mii_txc   <= {8{1'b1}};
        crc_state <= 32'hFFFFFFFF;
        beat_cnt  <= 8'd0;
        beats_tot <= BEATS_0;
        round_cnt <= 8'd0;
        idle_cnt  <= 4'd0;
        d_idx     <= 2'd0;
        pl_cnt    <= 16'd0;
        da_r      <= 48'd0;
        sa_r      <= 48'd0;
        type_r    <= 16'd0;
        trig_d    <= 1'b0;
        done      <= 1'b0;
    end else begin
        state     <= state_next;
        mii_txd   <= txd_next;
        mii_txc   <= txc_next;
        beat_cnt  <= beat_cnt_next;
        round_cnt <= round_cnt_next;
        idle_cnt  <= idle_cnt_next;
        d_idx     <= d_idx_next;
        pl_cnt    <= pl_cnt_next;
        trig_d    <= trig_start;
        done      <= done_next;

        // 觸發當下鎖存所有參數，之後整輪都用鎖存值（外部改動不影響進行中的發送）
        if (state == ST_LOAD) begin
            da_r      <= DA;
            sa_r      <= SA;
            type_r    <= eth_type;
            beats_tot <= beats_sel;
            round_cnt <= trig_rounds;
        end

        // CRC：preamble 時歸位，data beat 時累積，其餘保持
        if (crc_rst)      crc_state <= 32'hFFFFFFFF;
        else if (crc_upd) crc_state <= crc_state_next;
    end
end

endmodule

`resetall
