`timescale 1 ps / 100 fs

// ALU Control unit
// inputs:
//  - Function[5:0]: 6-bit function code of isntruction
//  - ALUOp[1:0]: ALU opcode
// outputs:
//  - ALUControl[1:0]: updated ALU opcode
module ALUControl_Block (
    input [5:0] Function,
    input [1:0] ALUOp,
    output reg [1:0] ALUControl
);
  wire [7:0] ALUControlIn;
  assign ALUControlIn = {ALUOp, Function};
  always @(ALUControlIn)
    casex (ALUControlIn)
      8'b11xxxxxx: ALUControl = 2'b01;  // xori, perform xor
      8'b00xxxxxx: ALUControl = 2'b00;  // lw and sw, performs add
      8'b01xxxxxx: ALUControl = 2'b10;  // bne, performs sub
      8'b10100000: ALUControl = 2'b00;  // add
      8'b10100010: ALUControl = 2'b10;  // sub
      8'b10101010: ALUControl = 2'b11;  // slt
      default: ALUControl = 2'b00;
    endcase
endmodule
