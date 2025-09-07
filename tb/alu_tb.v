`include "../alu.v"

module alu_tb ();

  reg [31:0] A, B;
  reg  [ 1:0] ALUControl;
  wire [31:0] result;
  wire carry, overflow, zero;

  // instantiate ALU
  alu dut (
      .A(A),
      .B(B),
      .ALUControl(ALUControl),
      .result(result),
      .carry(carry),
      .overflow(overflow),
      .zero(zero)
  );

  initial begin
    $monitor("T=%0t | ALUControl=%b | A=%h | B=%h | result=%h | carry=%b | ovf=%b | zero=%b",
             $time, ALUControl, A, B, result, carry, overflow, zero);

    // Test 1: ADD
    A = 32'h00000005;
    B = 32'h00000003;
    ALUControl = 2'b00;
    #10;

    // Test 2: XOR
    A = 32'hAAAAAAAA;
    B = 32'h55555555;
    ALUControl = 2'b01;
    #10;

    // Test 3: SUB (positive result)
    A = 32'h0000000A;
    B = 32'h00000005;
    ALUControl = 2'b10;
    #10;

    // Test 4: SUB (negative result)
    A = 32'h00000005;
    B = 32'h0000000A;
    ALUControl = 2'b10;
    #10;

    // Test 5: SLT
    A = 32'h00000005;
    B = 32'h0000000A;
    ALUControl = 2'b11;
    #10;

    // Test 6: ADD overflow
    A = 32'h7FFFFFFF;
    B = 32'h00000001;
    ALUControl = 2'b00;
    #10;

    // Test 7: SUB overflow
    A = 32'h80000000;
    B = 32'h00000001;
    ALUControl = 2'b10;
    #10;

    $finish;
  end

endmodule
