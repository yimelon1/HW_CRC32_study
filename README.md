# HW_CRC32_study

Verilog RTL for CRC-32 / LFSR study, including a 25GbE Ethernet packet
generator that exercises the CRC-32 (FCS) computation.

## Modules (`rtl/`)

- **`lfsr.v`** — Parametrizable combinatorial parallel LFSR/CRC core
  (Fibonacci or Galois configuration, arbitrary width and data width).
  Based on Alex Forencich's `verilog-lfsr` design.
- **`eth_pkt_gen.v`** — Fixed-length Ethernet packet generator for a
  25GbE / 64-bit MII interface. Generates preamble, header (DA/SA/EtherType),
  payload, and a proper FCS (CRC-32) trailer using `lfsr.v`, modeled after
  AMD's `xxv_ethernet_0_pkt_gen_mon.v` reference generator with FCS insertion
  added.

## Golden reference

FCS computation is validated against `CRC-32("123456789") = 0xCBF43926`.
