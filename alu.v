`timescale 1 ps / 100 fs

// ALU
// inputs:
//  - A[31:0]: operand 1
//  - B[31:0]: operand 2
//  - ALUControl[1:0]:
//    - 00: Add
//    - 01: XOR
//    - 10: Sub
//    - 11: SLT (set less than --- out = 1 if A - B < 0)
// outputs:
//  - result[31:0]: result of operation
//  - carry: carry out of add or sub
//  - overflow: operation resulted in overflow
//  - zero: result is 0
module alu (
    input [31:0] A,
    input [31:0] B,
    input [1:0] ALUControl,
    output [31:0] result,
    output carry,
    output overflow,
    output zero
);

  reg [32:0] tmp;

  always @(*) begin
    case (ALUControl)
      2'b00:   tmp = A + B;  // ADD
      2'b01:   tmp = {1'b0, A ^ B};  // XOR (no carry, force MSB=0)
      2'b10:   tmp = A - B;  // SUB
      2'b11:   tmp = (A < B) ? 33'd1 : 33'd0;  // SLT
      default: tmp = 33'd0;
    endcase
  end

  assign result = tmp[31:0];
  assign carry = tmp[32];
  assign overflow = (ALUControl == 2'b00) ?  // add overflow
      ((A[31] == B[31]) && (result[31] != A[31])) : (ALUControl == 2'b10) ?  // sub overflow
      ((A[31] != B[31]) && (result[31] != A[31])) : 1'b0;
  assign zero = (result == 32'd0);

endmodule
