`include "register.v"
`include "instrMem.v"
`include "control.v"
`include "dataMem.v"
`include "ALUControl.v"
`include "alu.v"
`include "forwarding.v"
`include "stallControl.v"

`timescale 1 ps / 100 fs

module MIPS (
    input clk,
    input reset
);

  // PC wires
  wire [31:0] PC, PCin;
  wire [31:0] PC4, ID_PC4, EX_PC4;
  wire [31:0] PCbne, PC4bne, PCj, PC4bnej, PCjr;

  // instruction wires
  wire [31:0] Instruction, ID_Instruction, EX_Instruction;
  wire [5:0] Opcode, Func;

  // extension wires
  wire [15:0] imm16;
  wire [31:0] Im16_Ext, EX_Im16_Ext;
  wire [31:0] sign_ext_out, zero_ext_out;

  // regfile wires
  wire [4:0] rs, rt, rd, EX_rs, EX_rt, EX_rd, EX_WriteRegister, MEM_WriteRegister, WB_WriteRegister;
  wire [31:0] WB_WriteData, ReadData1, ReadData2, EX_ReadData1, EX_ReadData2;

  // ALU wires
  reg [31:0] Bus_A_ALU, Bus_B_forwarded;
  wire [31:0] Bus_B_ALU;
  wire [31:0] EX_ALUResult, MEM_ALUResult, WB_ALUResult;
  wire ZeroFlag, OverflowFlag, CarryFlag;

  // memory write wires
  wire [31:0] WriteDataOfMem, MEM_ReadDataOfMem, WB_ReadDataOfMem;

  // control signals
  wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump, SignZero, JRControl;
  wire ID_RegDst,ID_ALUSrc,ID_MemtoReg,ID_RegWrite,ID_MemRead,ID_MemWrite,ID_Branch,ID_JRControl;
  wire EX_RegDst,EX_ALUSrc,EX_MemtoReg,EX_RegWrite,EX_MemRead,EX_MemWrite,EX_Branch,EX_JRControl;
  wire MEM_MemtoReg, MEM_RegWrite, MEM_MemRead, MEM_MemWrite;
  wire WB_MemtoReg, WB_RegWrite;
  wire [1:0] ALUOp, ID_ALUOp, EX_ALUOp;
  wire [1:0] ALUControl;
  wire bneControl;
  wire JumpControl, JumpFlush;
  wire [1:0] ForwardA, ForwardB;

  // flush
  wire IF_flush, IFID_flush, Stall_flush, flush;

  // shift left
  wire [31:0] shiftleft2_bne_out, shiftleft2_jump_out;  // shift left output

  // PC Write Enable, IF/ID Write Enable
  wire PC_WriteEn, IFID_WriteEn;

  // =================== IF ===================

  // pc register
  register PC_Reg (
      .q(PC),
      .d(PCin),
      .WriteEn(PC_WriteEn),
      .reset(reset),
      .clk(clk)
  );

  // pc + 4 wire
  assign PC4 = PC + 4;

  // get instruction
  InstructionMem InstructionMem1 (
      .address(PC),
      .instruction(Instruction)
  );

  // IF/ID registers
  register IFID_PC4 (
      .q(ID_PC4),
      .d(PC4),
      .WriteEn(IFID_WriteEn),
      .reset(reset),
      .clk(clk)
  );
  register IFID_Instruction (
      .q(ID_Instruction),
      .d(Instruction),
      .WriteEn(IFID_WriteEn),
      .reset(reset),
      .clk(clk)
  );

  // IF flush: see PC control section
  register #(
      .N(1)
  ) IF_flush_bit (
      .q(IFID_flush),
      .d(IF_flush),
      .WriteEn(IFID_WriteEn),
      .reset(reset),
      .clk(clk)
  );

  // =================== ID ===================

  // get instruction info
  assign Opcode = ID_Instruction[31:26];
  assign Func = ID_Instruction[5:0];
  assign rs = ID_Instruction[25:21];
  assign rt = ID_Instruction[20:16];
  assign rd = ID_Instruction[15:11];
  assign imm16 = ID_Instruction[15:0];

  // main control
  Control MainControl (
      .Opcode(Opcode),
      .RegDst(RegDst),
      .ALUSrc(ALUSrc),
      .MemtoReg(MemtoReg),
      .RegWrite(RegWrite),
      .MemRead(MemRead),
      .MemWrite(MemWrite),
      .Branch(Branch),
      .Jump(Jump),
      .SignZero(SignZero),
      .ALUOp(ALUOp)
  );

  // register file
  RegisterFile regfile (
      .ReadData1(ReadData1),
      .ReadData2(ReadData2),
      .WriteData(WB_WriteData),
      .ReadRegister1(rs),
      .ReadRegister2(rt),
      .WriteRegister(WB_WriteRegister),
      .RegWrite(WB_RegWrite),
      .reset(reset),
      .clk(clk)
  );

  // sign extend
  assign Im16_Ext = SignZero ? {16'd0, imm16} : {{16{imm16[15]}}, imm16};



  // ID/EX registers
  register IDEX_PC4 (
      .q(EX_PC4),
      .d(ID_PC4),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );

  register IDEX_ReadData1 (
      .q(EX_ReadData1),
      .d(ReadData1),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register IDEX_ReadData2 (
      .q(EX_ReadData2),
      .d(ReadData2),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register IDEX_Im16_Ext (
      .q(EX_Im16_Ext),
      .d(Im16_Ext),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register IDEX_rs_rt_rd (
      .q(EX_Instruction),
      .d(ID_Instruction),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_RegDst (
      .q(EX_RegDst),
      .d(ID_RegDst),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_ALUSrc (
      .q(EX_ALUSrc),
      .d(ID_ALUSrc),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_MemtoReg (
      .q(EX_MemtoReg),
      .d(ID_MemtoReg),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_RegWrite (
      .q(EX_RegWrite),
      .d(ID_RegWrite),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_MemRead (
      .q(EX_MemRead),
      .d(ID_MemRead),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_MemWrite (
      .q(EX_MemWrite),
      .d(ID_MemWrite),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_Branch (
      .q(EX_Branch),
      .d(ID_Branch),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) IDEX_JRControl (
      .q(EX_JRControl),
      .d(ID_JRControl),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(2)
  ) IDEX_ALUOp1 (
      .q(EX_ALUOp),
      .d(ID_ALUOp),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );

  // =================== EX ===================

  assign EX_rs = EX_Instruction[25:21];
  assign EX_rt = EX_Instruction[20:16];
  assign EX_rd = EX_Instruction[15:11];

  //  forwarding unit
  ForwardingUnit Forwarding_Block (
      MEM_RegWrite,
      WB_RegWrite,
      MEM_WriteRegister,
      WB_WriteRegister,
      EX_rs,
      EX_rt,
      ForwardA,
      ForwardB
  );

  // forward operand A
  always @(*) begin
    case (ForwardA)
      2'b00: Bus_A_ALU = EX_ReadData1;
      2'b10: Bus_A_ALU = MEM_ALUResult;
      2'b01: Bus_A_ALU = WB_WriteData;
    endcase
  end

  // forward operand B
  always @(*) begin
    case (ForwardB)
      2'b00: Bus_B_forwarded = EX_ReadData2;
      2'b10: Bus_B_forwarded = MEM_ALUResult;
      2'b01: Bus_B_forwarded = WB_WriteData;
    endcase
  end

  // choose operand B w/forwarded B and immediate
  assign Bus_B_ALU = EX_ALUSrc ? EX_Im16_Ext : Bus_B_forwarded;

  // ALU Control
  ALUControl_Block ALUControl_Block1 (
      EX_Im16_Ext[5:0],  // EX_Im16_Ext[5:0] is Func
      EX_ALUOp,
      ALUControl
  );

  // ALU
  alu alu_block (
      Bus_A_ALU,
      Bus_B_ALU,
      ALUControl,
      EX_ALUResult,
      CarryFlag,
      OverflowFlag,
      ZeroFlag
  );

  // choose write register
  assign EX_WriteRegister = EX_RegDst ? EX_rd : EX_rt;

  // EX/MEM register
  register EXMEM_ALUResult (
      .q(MEM_ALUResult),
      .d(EX_ALUResult),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register EXMEM_WriteDataOfMem (
      .q(WriteDataOfMem),
      .d(Bus_B_forwarded),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) EXMEM_MemtoReg (
      .q(MEM_MemtoReg),
      .d(EX_MemtoReg),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) EXMEM_RegWrite (
      .q(MEM_RegWrite),
      .d(EX_RegWrite),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) EXMEM_MemRead (
      .q(MEM_MemRead),
      .d(EX_MemRead),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) EXMEM_MemWrite (
      .q(MEM_MemWrite),
      .d(EX_MemWrite),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(5)
  ) EXMEM_WriteRegister (
      .q(MEM_WriteRegister),
      .d(EX_WriteRegister),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );

  // =================== MEM ===================

  // Data Memory
  dataMem dataMem1 (
      .address  (MEM_ALUResult),     // address
      .writedata(WriteDataOfMem),    // writedata
      .WriteEn  (MEM_MemWrite),      // writeenable
      .MemRead  (MEM_MemRead),
      .clk      (clk),
      .data     (MEM_ReadDataOfMem)  // data
  );

  // MEM/WB register
  register MEMWB_ReadDataOfMem (
      .q(WB_ReadDataOfMem),
      .d(MEM_ReadDataOfMem),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register MEMWB_ALUResult (
      .q(WB_ALUResult),
      .d(MEM_ALUResult),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(5)
  ) MEMWB_WriteRegister (
      .q(WB_WriteRegister),
      .d(MEM_WriteRegister),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) MEMWB_MemtoReg (
      .q(WB_MemtoReg),
      .d(MEM_MemtoReg),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );
  register #(
      .N(1)
  ) MEMWB_RegWrite (
      .q(WB_RegWrite),
      .d(MEM_RegWrite),
      .WriteEn(1'b1),
      .reset(reset),
      .clk(clk)
  );

  // =================== WB ===================

  // select data to WriteData for regfile
  assign WB_WriteData = WB_MemtoReg ? WB_ReadDataOfMem : WB_ALUResult;

  // =================== Stall control ===================

  StallControl StallControl_block (
      .ID_EX_MemRead(EX_MemRead),
      .IF_ID_rs(rs),
      .IF_ID_rt(rt),
      .ID_EX_rt(EX_rt),
      .PCWrite(PC_WriteEn),
      .IF_ID_Write(IFID_WriteEn),
      .ID_EX_controlZero(Stall_flush)
  );

  // =================== PC Control ===================

  // set JRControl to 1 if alu op and Func are 10, 001000
  assign JRControl = {ALUOp, Func} == 8'b10001000;

  assign IF_flush = JumpControl | bneControl | EX_JRControl;
  assign ID_flush = bneControl | EX_JRControl;

  // clear control lines if flush detected
  assign flush = ID_flush | IFID_flush | Stall_flush;
  wire notFlush = ~flush;
  assign ID_RegDst = RegDst & notFlush;
  assign ID_ALUSrc = ALUSrc & notFlush;
  assign ID_MemtoReg = MemtoReg & notFlush;
  assign ID_RegWrite = RegWrite & notFlush;
  assign ID_MemRead = MemRead & notFlush;
  assign ID_MemWrite = MemWrite & notFlush;
  assign ID_Branch = Branch & notFlush;
  assign ID_ALUOp = ALUOp & {2{notFlush}};
  assign ID_JRControl = JRControl & notFlush;

  // bne
  assign shiftleft2_bne_out = EX_Im16_Ext << 2;
  assign PCbne = EX_PC4 + shiftleft2_bne_out;
  assign bneControl = EX_Branch & ~ZeroFlag;
  assign PC4bne = bneControl ? PCbne : PC4;

  // jump
  assign shiftleft2_jump_out = {6'b0, ID_Instruction[25:0]} << 2;
  assign PCj = {ID_PC4[31:28], shiftleft2_jump_out[27:0]};

  assign JumpFlush = Jump & ~IFID_flush;
  assign JumpControl = JumpFlush & ~bneControl;
  assign PC4bnej = JumpControl ? PCj : PC4bne;

  // jr
  assign PCjr = Bus_A_ALU;
  assign PCin = EX_JRControl ? PCjr : PC4bnej;

endmodule


`timescale 1 ps / 100 fs
module mips_tb ();
  parameter ClockDelay = 5000;
  reg clk, reset;

  MIPS dut (
      .clk  (clk),
      .reset(reset)
  );

  initial begin
    clk = 0;
    forever #(ClockDelay / 2) clk = ~clk;  // clock generator
  end

  initial begin
    reset = 1;  // assert reset
    #10000 reset = 0;  // release after a short delay
  end

  initial begin
    $dumpfile("mips.vcd");
    $dumpvars(0, dut);

    // print key state
    $monitor("PC=%h  IF_instr=%b ", dut.PC, dut.Instruction, "id_instr=%b ifid_flush=%h ",
             dut.ID_Instruction, dut.IFID_flush,
             "$zero=%h $s0=%h $s1=%h $s2=%h $s3=%h $s4=%h $s5=%h %b",
             dut.regfile.RegArray[0],  // $zero
             dut.regfile.RegArray[16],  // $s0
             dut.regfile.RegArray[17],  // $s1
             dut.regfile.RegArray[18],  // $s2
             dut.regfile.RegArray[19],  // $s3
             dut.regfile.RegArray[20],  // $s4
             dut.regfile.RegArray[21],  // $s5
             dut.Stall_flush);

    #160000 $finish;

  end
endmodule
