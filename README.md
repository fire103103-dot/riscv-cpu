# 3-Stage Pipelined RISC-V CPU

**Course:** UC Berkeley EECS 151/251A (Fall 2025)  
**Completed:** Checkpoint 2 (Pipelined CPU & FPGA Implementation)

## Overview
Implemented a 3-stage pipelined RISC-V CPU supporting RV32I instruction set on Xilinx PYNQ-Z1 FPGA during my exchange semester at UC Berkeley.

## My Contributions
**This repository contains only code I personally wrote:**
- `cpu.v` (812 lines): Complete CPU datapath, control logic, hazard detection, data forwarding, and MMIO interface
- `reg_file.v`: 32x32-bit register file with write-first bypass logic
- Design documents: Detailed pipeline diagram and architectural decision documentation

**Note:** University-provided infrastructure (memories, testbenches, I/O circuits) is NOT included to respect academic integrity. This repository is kept private in accordance with UC Berkeley EECS 151 course policies.

## Performance Metrics
- **CPI:** 1.188 @ 50 MHz
- **Resource Usage:** 1,894 LUTs, 1,590 registers, 36 BRAMs
- **Testing:** 80/80 cpu_tb tests, 36/38 RISC-V ISA tests passed
- **Benchmark:** Matrix multiplication (mmult) with verified checksum (0x0001f800)

## Architecture Highlights
1. **3-Stage Pipeline:** IF (Instruction Fetch), ID/EX (Decode & Execute), MEM/WB (Memory & Writeback)
   - Adapted from classic 5-stage pipeline due to FPGA Block RAM synchronous read requirements
2. **Full Data Forwarding:** From EX/MEM and MEM/WB stages to both ALU inputs and store data path
3. **Load-Use Hazard Handling:** 1-cycle stall with forwarding from MEM/WB stage
4. **Branch Prediction:** Predict-not-taken with 1-cycle flush penalty
5. **Memory Architecture:** Separate BIOS, instruction, and data memories with MMIO support

## Technical Implementation
- **Hazard Detection:** Automatic detection of load-use dependencies with pipeline stalling
- **Control Logic:** Combinational decoder with comprehensive opcode support
- **MMIO:** UART communication, cycle counter, instruction counter, branch prediction counters
- **Memory Access:** Byte-aligned load/store operations with sign/zero extension

## Files Included
- `cpu.v`: Main CPU implementation (812 lines)
- `reg_file.v`: Integer register file with bypass logic
- `Checkpoint1_Answers.pdf`: Design decisions, pipeline analysis, and architectural Q&A
- `cpu_diagram_main.pdf`: Detailed block diagram with pipeline stages and control paths

## Academic Integrity Notice
This code is maintained as a private repository in accordance with UC Berkeley EECS 151 academic integrity policies. Access is granted only to potential employers, graduate schools, or recommendation letter writers upon request.

**Contact:** fire103103@gmail.com