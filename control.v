`timescale 1 ps / 100 fs

// Control unit
// inputs:
//  - opcode: upper 6 bits of instruction
// outputs:
//  - RegDst: write register mux control --- 1: rd, 0: rt
//  - ALUSrc: ALU operand 2 mux control --- 1: immediate, 0: rt (rd2)
//  - MemtoReg: write read data from data mem to write register
//  - RegWrite: write to write register
//  - MemRead: read from Data Mem
//  - MemWrite: write to Data Mem
//  - Branch: take branch, set pc to branch address
//  - Jump: set pc to jump address
//  - SignZero: 0 extend instead of sign extend
module Control (
    input [5:0] Opcode,
    output reg RegDst,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg Branch,
    output reg Jump,
    output reg SignZero,
    output reg [1:0] ALUOp
);

  always @(*)
    case (Opcode)
      6'b000000: begin  // R - type
        RegDst = 1'b1;
        ALUSrc = 1'b0;
        MemtoReg = 1'b0;
        RegWrite = 1'b1;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        Branch = 1'b0;
        ALUOp = 2'b10;
        Jump = 1'b0;
        SignZero = 1'b0;
      end
      6'b100011: begin  // lw - load word
        RegDst = 1'b0;
        ALUSrc = 1'b1;
        MemtoReg = 1'b1;
        RegWrite = 1'b1;
        MemRead = 1'b1;
        MemWrite = 1'b0;
        Branch = 1'b0;
        ALUOp = 2'b00;
        Jump = 1'b0;
        SignZero = 1'b0;  // sign extend
      end
      6'b101011: begin  // sw - store word
        RegDst = 1'bx;
        ALUSrc = 1'b1;
        MemtoReg = 1'bx;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b1;
        Branch = 1'b0;
        ALUOp = 2'b00;
        Jump = 1'b0;
        SignZero = 1'b0;
      end
      6'b000101: begin  // bne - branch if not equal
        RegDst = 1'b0;
        ALUSrc = 1'b0;
        MemtoReg = 1'b0;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        Branch = 1'b1;
        ALUOp = 2'b01;
        Jump = 1'b0;
        SignZero = 1'b0;  // sign extend
      end
      6'b001110: begin  // XORI - XOR immidiate
        RegDst = 1'b0;
        ALUSrc = 1'b1;
        MemtoReg = 1'b0;
        RegWrite = 1'b1;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        Branch = 1'b0;
        ALUOp = 2'b11;
        Jump = 1'b0;
        SignZero = 1'b1;  // zero extend
      end
      6'b000010: begin  // j - Jump
        RegDst = 1'b0;
        ALUSrc = 1'b0;
        MemtoReg = 1'b0;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        Branch = 1'b0;
        ALUOp = 2'b00;
        Jump = 1'b1;
        SignZero = 1'b0;
      end
      default: begin
        RegDst = 1'b0;
        ALUSrc = 1'b0;
        MemtoReg = 1'b0;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        Branch = 1'b0;
        ALUOp = 2'b10;
        Jump = 1'b0;
        SignZero = 1'b0;
      end

    endcase

endmodule
