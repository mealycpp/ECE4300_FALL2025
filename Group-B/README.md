# Group B Final Project Submission

## Group Members
- Demajay Francis
- Carson Quesada
- Cole Pedersen
- Derek Jacoby

## Project Overview
This final project benchmarks four RISC-V cores on the Nexys A7 FPGA board. The cores being benchmarked are:

- PicoRV32 (YosysHQ)
- Ibex (lowRISC)
- VexRiscv (h2ozrw)
- cv32e40p (OpenHW Group)

All cores are built as RV32IMC implementations.

## Repository Structure

1. **Firmware**  
   Contains the files necessary to build the benchmarking test as a HEX file. Benchmark code is located in `main.c` and can be built using the `build.bat` script.

2. **PicoRV32 Test**  
   Vivado project for benchmarking the PicoRV32 core on the Nexys A7.

3. **Ibex Test**  
   Vivado project for benchmarking the Ibex core on the Nexys A7 (Coming Soon).

4. **VexRiscv Test**  
   Vivado project for benchmarking the VexRiscv core on the Nexys A7 (Coming Soon).

5. **cv32e40p Test**  
   Vivado project for benchmarking the CV32E40P core on the Nexys A7 (Coming Soon).

6. **Documentation**  
   Contains the report and demonstration materials. Refer to this folder for more information about the project.
