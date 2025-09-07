`timescale 1 ps / 100 fs

// dataMem: data memory module, holds program (ram)
// inputs:
//  - address [31:0]: address to access, comes from ALU result
//  - writedata [31:0]: data to write to address, comes from rd2
//  - WriteEn: enable writing
//  - MemRead: output data from address
//  - clk
// outputs:
//  - data [31:0]: data at given address
module dataMem (
    input [31:0] address,
    input [31:0] writedata,
    input WriteEn,
    input MemRead,
    input clk,
    output [31:0] data
);

  reg [ 7:0] datamem[31:0];
  reg [31:0] tmp;

  // assign data byte wise
  always @(*)
    if (WriteEn) begin
      datamem[address]   = writedata[31:24];
      datamem[address+1] = writedata[23:16];
      datamem[address+2] = writedata[15:8];
      datamem[address+3] = writedata[7:0];
    end

  // get data byte wise
  always @(*) begin
    if (MemRead)
      tmp = {datamem[address], datamem[address+1], datamem[address+2], datamem[address+3]};
  end
  assign data = tmp;

endmodule
