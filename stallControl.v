`timescale 1 ps / 100 fs


// stall control unit:
// inputs:
//  - ID_EX_MemRead: set write data to the data read from Data Mem (lw instruction)
//  - IF_ID_rs[4:0]: if/id rs
//  - IF_ID_rt[4:0]: if/id rt
//  - ID_EX_rt[4:0]: ID_EX rt
// outputs:
//  - PCWrite: write to PC (if 0 keeps pc same)
//  - IF_ID_Write: freeze if/id register
//  - ID_EX_controlZero: zero out control signals (noop)
module StallControl (
    input ID_EX_MemRead,
    input [4:0] IF_ID_rs,
    input [4:0] IF_ID_rt,
    input [4:0] ID_EX_rt,
    output PCWrite,
    output IF_ID_Write,
    output ID_EX_controlZero
);
  // stall the pipeline when the instruction in ID depends on a value
  // that is being loaded by the instruction in EX (ID/EX)
  // this happens when:
  //   - ID/EX.MemRead == 1  (the instruction in EX is a load word)
  //   - AND the load’s destination register (ID/EX.RegisterRt)
  //     matches one of the source registers of the IF/ID instruction
  //     (IF/ID.RegisterRs or IF/ID.RegisterRt)
  wire flush = ID_EX_MemRead && (ID_EX_rt == IF_ID_rs || ID_EX_rt == IF_ID_rt);

  assign PCWrite = ~flush;
  assign IF_ID_Write = ~flush;
  assign ID_EX_controlZero = flush;

endmodule
