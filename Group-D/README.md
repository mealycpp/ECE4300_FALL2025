Presentation Link: https://youtu.be/I4oTRnUxE5M

# Hardware-Accelerated Montgomery Multiplication for RSA on PYNQ-Z2

This repository provides a complete hardware–software system for accelerating Montgomery modular multiplication on the Xilinx PYNQ-Z2 platform. The project implements a 2048-bit iterative Montgomery multiplier in Verilog, integrates it with the ARM Cortex-A9 processing system using AXI4-Lite, and benchmarks the speedup for RSA-1024 and RSA-2048 encryption and decryption.

The system demonstrates that accelerating only the Montgomery multiplication kernel yields significant end-to-end RSA performance improvements on resource-constrained SoC platforms.

## 1. Project Overview

RSA encryption and decryption rely heavily on large-integer modular multiplication. Montgomery multiplication avoids expensive division operations and is well suited for FPGA implementation.

This project includes:

- A 2048-bit word-serial Montgomery multiplier in Verilog  
- An AXI4-Lite interface connecting the multiplier to the ARM PS  
- A matching software Montgomery multiplication implementation in C  
- Benchmark and verification infrastructure  
- Performance analysis comparing software-only vs. hardware-accelerated RSA workloads

Measured speedups:

- **~7.5× for RSA-2048 encryption**  
- **~12.5× for RSA-2048 decryption**  
- Similar gains for RSA-1024  

## 2. Repository Structure

```
/hardware
    montgomery_mul.v       # Core 2048-bit Montgomery multiplier
    montgomery_axi.v       # AXI4-Lite wrapper for PS–PL communication

/software
    main_1.c               # C implementation, test vectors, benchmarks
    python/                # Python control + benchmarking (if applicable)

/docs
    FinalPresentation.pptx
    GroupD_Final_Report.docx
```

## 3. Hardware Architecture

### 3.1 Montgomery Multiplier (`montgomery_mul.v`)

- 2048-bit operands (64 × 32-bit words)  
- Iterative, word-serial architecture (≈2048 cycles per multiplication)  
- Internal FSM manages load, accumulation, and reduction  
- Optimized for area–performance efficiency  
- Produces bit-accurate results matching software and OpenSSL vectors  

### 3.2 AXI4-Lite Wrapper (`montgomery_axi.v`)

The AXI interface exposes:

- Registers for A, B, modulus N, and n’ (Montgomery constant)  
- Control register to start multiplication  
- Status register for completion  
- Memory-mapped register access from the ARM processor  

Operation flow:

1. PS writes operands into AXI registers  
2. PS asserts start bit  
3. Hardware performs Montgomery multiplication  
4. PS polls “done”  
5. PS retrieves the 2048-bit result  

## 4. Software Architecture

### 4.1 Software Baseline (C implementation)

The `main_1.c` program includes:

- Pure software Montgomery multiplication  
- OpenSSL test vector correctness verification  
- RSA-1024 and RSA-2048 benchmarking  
- Timing using `clock_gettime()`  
- Comparison against FPGA-accelerated results  

### 4.2 Python Benchmark Harness

A Python script (when included) running on PYNQ:

- Loads operands into AXI registers  
- Initiates hardware execution  
- Reads back results  
- Verifies correctness against software  
- Records timing data  

## 5. Benchmark Results

### RSA-2048 Performance

| Operation | Software Cycles | Hardware Cycles | Speedup |
|----------|-----------------|------------------|---------|
| Encrypt  | ~6.08×10⁸       | ~7.98×10⁶        | ~7.5×   |
| Decrypt  | ~1.49×10⁸       | ~1.20×10⁷        | ~12.5×  |

### RSA-1024 Performance

| Operation | Software Cycles | Hardware Cycles | Speedup |
|----------|-----------------|------------------|---------|
| Encrypt  | ~1.06×10⁸       | ~1.97×10⁶        | ~7.5×   |
| Decrypt  | ~4.06×10⁸       | ~3.61×10⁶        | ~11.4×  |

All hardware results match software outputs bit-accurately across all test cases.


## 7. Key Contributions

- Designed a full 2048-bit Montgomery multiplication accelerator  
- Integrated AXI4-Lite interface for PS–PL communication  
- Developed matching software implementation for benchmarking  
- Validated hardware via OpenSSL test vectors  
- Demonstrated consistent 7.5×–12.5× RSA acceleration  
- Created a modular architecture supporting future scalability  

## 8. Future Work

- Full modular exponentiation acceleration in hardware  
- Pipelined or partially parallel Montgomery datapaths  
- Support for RSA-3072 and larger keys  
- Performance comparison with softcore or RISC-V processors  
- Energy-efficiency analysis of hardware vs. software RSA  

## 9. Authors

Caleb Pai  
Winson Zhu  
Paul Kim  
Hyukjin Jeong  

California State Polytechnic University, Pomona  
Department of Electrical & Computer Engineering  
