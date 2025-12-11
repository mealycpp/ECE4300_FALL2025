# Matrix Multiplication - FPGA vs CPU

This project investigates the performance and energy characteristics of divide-and-conquer matrix multiplication on two computing platforms: a general-purpose CPU and a hardware-implemented accelerator on a Xilinx Nexys A7-100T FPGA. 

The paper is under the file: 
ECE_4300_Matrix_Multiplication_Comparison.pdf

The presentation is under the file:
4300 Project.pdf

Research Questions:

1. How does the execution time and computational throughput of matrix multiplication differ between CPU-based recursive algorithms and FPGA-based custom hardware implementations?

2. To what extent does hardware specialization in FPGAs improve energy efficiency compared to general-purpose CPU execution for matrix multiplication tasks?

3. How do resource constraints (LUTs, FFs, DSPs) on the FPGA influence the achievable performance, and what performance gains could be realized by parallelizing the hardware multiplier units?

4. What trade-offs arise between design complexity, flexibility, and performance when choosing between CPU software implementations and FPGA hardware architectures for matrix-intensive applications?

## Authors
Evan Tram, Scott Pan, Anthony Parra, Dan Nguyen
