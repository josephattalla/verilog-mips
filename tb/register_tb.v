`include "../register.v"

module registerfile_tb ();

  reg [4:0] rr1, rr2, wr;
  reg [31:0] writedata;
  reg regwrite, reset, clk;
  wire [31:0] rd1, rd2;

  // instantiate the register file
  RegisterFile rf (
      .ReadRegister1(rr1),
      .ReadRegister2(rr2),
      .WriteRegister(wr),
      .WriteData(writedata),
      .RegWrite(regwrite),
      .reset(reset),
      .clk(clk),
      .ReadData1(rd1),
      .ReadData2(rd2)
  );

  // clock generator
  initial clk = 0;
  always #5 clk = ~clk;

  // Test stimulus
  initial begin
    $monitor("Time=%0t, rr1=%d, rr2=%d, rd1=%h, rd2=%h", $time, rr1, rr2, rd1, rd2);

    // initialize
    reset = 1;
    regwrite = 0;
    wr = 0;
    writedata = 0;
    rr1 = 0;
    rr2 = 0;
    #10 reset = 0;

    //---------write and read tests

    // test 1: set register 5 to 0xFFFFFFFF
    // rr1 = 5, rr2 = 0, rd1 = 0xFFFFFFFF, rd2 = 0x00000000
    wr = 5;
    writedata = 32'hFFFFFFFF;
    regwrite = 1;
    #10 regwrite = 0;
    rr1 = 5;
    rr2 = 0;
    #10;

    // test 2: set register 2 to 0xDEADBEEF
    // rr1 = 2, rr2 = 5, rd1 = 0xDEADBEEF, rd2 = 0xFFFFFFFF
    wr = 2;
    writedata = 32'hDEADBEEF;
    regwrite = 1;
    #10 regwrite = 0;
    rr1 = 2;
    rr2 = 5;
    #10;

    // test 3: set register 2 to 0x00000000
    // rr1 = 2, rr2 = 5, rd1 = 0x00000000, rd2 = 0xFFFFFFFF
    wr = 2;
    writedata = 32'd0;
    regwrite = 1;
    #10 regwrite = 0;
    #10;

    $finish;
  end

endmodule
