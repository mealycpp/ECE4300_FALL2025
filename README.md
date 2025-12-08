# FPGA vs Python A* Pathfinding Accelerator — Group E Final Project

## Project Contributors
- Changwe Musonda
- Emily Morales
- Kevin Wang

## Project Overview
This project implements and benchmarks a hardware-accelerated A* pathfinding engine on a Xilinx Artix-7 FPGA.
The design supports 8-connected omnidirectional movement, uses the Octile Distance heuristic, and is validated against a Python software reference model.

The system includes:
- Verilog Hardware Accelerator (optimized for 16×16 grids)
- Python Reference Implementation for correctness verification
- Interactive Web-Based Visualizer for demonstrating algorithm behavior
- Seven Comprehensive Test Mazes evaluating correctness, efficiency, and performance
All hardware and software implementations match exactly in nodes expanded, path quality, and optimality.

## Key Features
- 8-connected motion (N, S, E, W, NE, NW, SE, SW)
- Optimized Octile heuristic (admissible & consistent)
- 10–100× speedup vs. Python implementation
- FPGA resource-efficient (4.5% LUT usage on Artix-7)
- Verified across 7 test scenarios
- Real-time visualization via Python interface

## Repository Structure
  1. Code
    1. Python
       - Contains the software reference implementation of A*.
         Used for verifying hardware correctness and generating expected outputs.
    2. Verilog
       - Hardware implementation of the accelerator.
         Includes:
         1. astar_top - top-level module and logic components
         2. astar_tb - testbench used for simulation and verification
