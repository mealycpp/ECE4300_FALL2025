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
- Python Visualizer for demonstrating algorithm behavior
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
  2. Documentation
     - Contains IEEE formatted paper and rough draft powerpoint presentation
  3. Vizualizer
     - Python visualizer for interacting with A* pathfinding.
     - Supports grid editing, animated search visualization, and comparison modes.

## Test Cases
The system includes seven curated test mazes designed to verify path correctness, obstacle handling, diagonal traversal, and corner cases:
1. Simple Diagonal — Empty 16×16 grid, direct diagonal path from (0,0) to (7,7).
2. Spiral Maze — Nested walls forcing deeper search expansion.
3. Random Obstacles — 30% density randomized layout.
4. Snake Pattern — Alternating barriers requiring precise navigation.
5. Rooms & Corridors — Structured maze with segmented rooms and openings.
6. Diagonal Corridor — Maze intentionally optimized for 8-directional movement.
7. No Path — Fully blocked scenario testing failure handling and termination.
Each test case is validated across Python and hardware, with identical node-expansion behavior.

## Hardware Specifications
- Target Device: Xilinx Artix-7
- Grid Size: 16×16 (configurable)
- Motion: 8-connected (N, S, E, W, NE, NW, SE, SW)
- Move Costs: 10 (straight), 14 (diagonal)
- Heuristic: Octile Distance (scaled to match cost model)
- Resource Utilization:
  - 2,847 LUTs (4.5%)  
  - 1,923 FFs (1.5%)
  - 3 BRAM blocks
- Latency: 3–85 microseconds depending on test case
- Cycles per node: approx. 3–9 cycles
     
