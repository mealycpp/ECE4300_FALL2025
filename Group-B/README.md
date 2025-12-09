# Group B Final Project Submission

## Group Members
- Demajay Francis
- Carson Quesada
- Cole Pedersen
- Derek Jacoby

## Project Overview
This project benchmarks multiple RISC-V cores on the Nexys A7 FPGA board. The cores evaluated in this submission are:

- PicoRV32 (YosysHQ)
- VexRiscv (SpinalHDL)

All cores are implemented as RV32IMC designs.

A single, common benchmarking program is used across both cores. The existence of separate firmware folders does not indicate different benchmark tests—the benchmark logic is identical. Differences arise only from each core’s integration requirements, such as SoC structure, I/O handling, memory mapping, and the expected HEX file format.

## Repository Structure

### 1. PicoRV32 Firmware
Contains the files required to build the benchmarking program for the PicoRV32-based system. Although this folder is separate, it produces the same benchmark as the VexRiscv firmware; the build process differs only because of PicoRV32’s architecture and required firmware format. The firmware can be built using the included `build.bat` script.

### 2. VexRiscv Firmware
Contains the files required to build the same benchmarking program for the VexRiscv-based system. This folder exists to support VexRiscv-specific integration details (such as the Murax SoC layout, memory map, and I/O). The firmware can be built using the included `build.bat` script.

### 3. PicoRV32 Test
Vivado project for synthesizing and benchmarking the PicoRV32 core on the Nexys A7.

### 4. VexRiscv Test
Vivado project for synthesizing and benchmarking the VexRiscv core on the Nexys A7.  
(Coming soon.)

### 5. Documentation
Contains the final report, supporting notes, diagrams, benchmark methodology, and demonstration materials.
