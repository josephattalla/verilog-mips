`timescale 1 ps / 100 fs

// Registers: handle register data and writing
// inputs:
//  - ReadRegister1[31:0]: register to read
//  - ReadRegister2[31:0]: register to read
//  - WriteRegister [31:0]: register to write to
//  - WriteData[31:0]: data to write
//  - RegWrite: enable writing WriteData to WriteRegister
//  - reset: clear register data
//  - clk
// outputs:
//  - ReadData1[31:0]: register 1 data
//  - ReadData2[31:0]: register 2 data
module RegisterFile (
    input  [ 4:0] ReadRegister1,
    input  [ 4:0] ReadRegister2,
    input  [ 4:0] WriteRegister,
    input  [31:0] WriteData,
    input         RegWrite,
    input         reset,
    input         clk,
    output [31:0] ReadData1,
    output [31:0] ReadData2
);

  // array for all 32, 32 bit registers
  reg [31:0] RegArray[0:31];

  integer i = 0;
  always @(posedge clk or posedge reset) begin
      if (reset)
          for (i = 0; i < 32; i++) RegArray[i] <= 0;
      else if (RegWrite && WriteRegister != 0)
          RegArray[WriteRegister] = WriteData;
  end

  assign ReadData1 = RegArray[ReadRegister1];
  assign ReadData2 = RegArray[ReadRegister2];

endmodule


// register: N bit register
// acts as N d flip flops
module register #(
    parameter N = 32
) (
    input [N-1:0] d,
    input WriteEn,
    input clk,
    input reset,
    output reg [N-1:0] q
);

  always @(posedge clk) begin
    if (reset) q <= {N{1'b0}};  // all zeros
    else if (WriteEn) q <= d;
  end

endmodule
