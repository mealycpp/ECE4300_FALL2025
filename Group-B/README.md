Group B Final Project Submission
Group Members

Demajay Francis

Carson Quesada

Cole Pedersen

Derek Jacoby

Project Overview

This final project benchmarks four RISC-V cores on the Nexys A7 FPGA board.
The cores analyzed include:

PicoRV32 (YosysHQ)

Ibex (lowRISC)

VexRiscv (h2ozrw)

cv32e40p (OpenHW Group)

All cores are configured as RV32IMC implementations.

Repository Structure

This project consists of the following sections:

1. Firmware

Contains the files needed to build the benchmarking test into a HEX file.

Benchmark source code: main.c

Build script: build.bat

Requirements: you must have the riscv-none-elf-gcc toolchain intalled and added to your user path to build.

2. PicoRV32 Test

Vivado project for benchmarking the PicoRV32 core on the Nexys A7.

3. Ibex Test

Vivado project for benchmarking the Ibex core on the Nexys A7. (Coming soon)

4. VexRiscv Test

Vivado project for benchmarking the VexRiscv core on the Nexys A7. (Coming soon)

5. cv32e40p Test

Vivado project for benchmarking the cv32e40p core on the Nexys A7. (Coming soon)

6. Documentation

Contains the project report, demonstration materials, and supporting details. (Demo coming soon)