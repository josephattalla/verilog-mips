# verilog-mips
This project implements a **32-bit 5-stage pipelined MIPS processor** entirely in Verilog. It models a complete pipelined CPU with instruction fetch, decode, execute, memory access, and write-back stages, supporting basic MIPS instructions along with forwarding, stall control, and jump/branch handling.

## Supported Instructions

1. **ADD rd, rs, rt**: `Reg[rd] = Reg[rs] + Reg[rt]`
2. **SUB rd, rs, rt**: `Reg[rd] = Reg[rs] - Reg[rt]`
3. **SLT rd, rs, rt**: `Reg[rd] = 1 if Reg[rs] < Reg[rt], else 0`
4. **BNE rs, rt, imm16**: `if (Reg[rs] != Reg[rt]) PC = PC + 4 + SignExt(imm16)<<2 else PC = PC + 4`
5. **J target**: `PC = {PC[31:28], target, 00}`
6. **JR rs**: `PC = Reg[rs]`
7. **LW rt, imm16(rs)**: `Reg[rt] = Mem[Reg[rs] + SignExt(imm16)]`
8. **SW rt, imm16(rs)**: `Mem[Reg[rs] + SignExt(imm16)] = Reg[rt]`
9. **XORI rt, rs, imm16**: `Reg[rt] = Reg[rs] XOR ZeroExt(imm16)`

## Features

- **5-stage pipeline**: IF, ID, EX, MEM, WB.
- **Forwarding Unit**: Resolves data hazards to minimize stalls.
- **Stall Control**: Detects load-use hazards and stalls the pipeline as needed.
- **Control Unit**: Generates control signals for ALU, memory, and register file operations.
- **ALU and ALU Control**: Supports arithmetic, logical operations, and immediate values.
- **Memory Access**: Instruction and data memory modules.
- **Register File**: 32 general-purpose registers.
- **PC Control**: Handles jumps, branches, and `JR` instructions.
- **Testbench Included**: Clock generation, reset, and waveform output for simulation.

## File Structure

- `top.v`: Top-level module for the pipelined processor.
- `register.v`: Register file and parameterized register module for pipeline registers and general storage.
- `instrMem.v`: Instruction memory module.
- `dataMem.v`: Data memory module.
- `control.v`: Main control unit for instruction decoding.
- `ALUControl.v`: Generates ALU operation signals.
- `alu.v`: Arithmetic Logic Unit module.
- `forwarding.v`: Forwarding unit for resolving data hazards.
- `stallControl.v`: Detects hazards and generates pipeline stall signals.
- `tb/alu_tb.v`: ALU testbench.
- `tb/instrmem_tb.v`: Instruction memory testbench.
- `tb/register_tb.v`: Register file testbench.
- `test.asm`: MIPS test instructions.
- `instr.txt`: Program memory.

## Simulation

- Generates waveforms using `$dumpfile` and `$dumpvars`.
- Can be viewed with **GTKWave** or any Verilog simulator.
- Monitors key registers (`$zero`, `$s0–$s5`) and pipeline flush signals.

## How to Run

1. Put instruction machine code into instrmem.txt
2. Synthesize
```bash
iverilog top.v
```
3. Run the simulation and generate waveform files:
```bash
vvp ./a.out
```
4. View signals in GTKWave
```bash
gtkwave mips.vcd
```
