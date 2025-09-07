`timescale 1 ps / 100 fs

// 1024kb memory for instructions
// inputs:
//  - address[31:0]: instruction address
// outputs:
//  - instruction[31:0]: instruction at requested address
module InstructionMem (
    input [31:0] address,
    output reg [31:0] instruction
);


  // 1024, 32 bit memory for instructions (4kb)
  reg [31:0] instrmem[0:1023];

  // get instruction from input address
  // convert from byte address to word address
  // ex:
  // address 0 → instruction 0
  // address 4 → instruction 1
  // address 8 → instruction 2
  // address 12 → instruction 3
  always @(address) begin
    instruction = instrmem[address>>2];
  end

  // initialize memory array 
  initial begin
    $readmemb("instr.txt", instrmem);
  end

endmodule
