`timescale 1 ps / 100 fs

// Forwarding Unit
module ForwardingUnit (
    input EX_MEM_RegWrite,
    input MEM_WB_RegWrite,
    input [4:0] EX_MEM_WriteRegister,
    input [4:0] MEM_WB_WriteRegister,
    input [4:0] ID_EX_rs,
    input [4:0] ID_EX_rt,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
);

  // forward from EX/MEM has priority because it will hold the newest value
  // forward A from EX/MEM result if: (EX/MEM.RegWrite == 1)
  //                                and (EX/MEM.WriteRegister != 0)
  //                                and (EX/MEM.WriteRegister == ID/EX.rs)
  //                              then: ForwardA = 10
  // forward A from MEM/WB result if: (MEM/WB.RegWrite == 1)
  //                                and (MEM/WB.WriteRegister != 0)
  //                                and (MEM/WB.WriteRegister == ID/EX.rs)
  //                              then: ForwardA = 01
  // forward B from EX/MEM result if: (EX/MEM.RegWrite == 1)
  //                                and (EX/MEM.WriteRegister != 0)
  //                                and (EX/MEM.WriteRegister == ID/EX.rt)
  //                              then: ForwardB = 10
  // forward B from MEM/WB result if: (MEM/WB.RegWrite == 1)
  //                                and (MEM/WB.WriteRegister != 0)
  //                                and (MEM/WB.WriteRegister == ID/EX.rt)
  //                              then: ForwardB = 01
  always @(*) begin
    // default: no forward
    ForwardA = 2'b00;
    ForwardB = 2'b00;

    // Forward A
    if (EX_MEM_RegWrite && (EX_MEM_WriteRegister != 0) && (EX_MEM_WriteRegister == ID_EX_rs))
      ForwardA = 2'b10;
    else if (MEM_WB_RegWrite && (MEM_WB_WriteRegister != 0) && (MEM_WB_WriteRegister == ID_EX_rs))
      ForwardA = 2'b01;

    // Forward B
    if (EX_MEM_RegWrite && (EX_MEM_WriteRegister != 0) && (EX_MEM_WriteRegister == ID_EX_rt))
      ForwardB = 2'b10;
    else if (MEM_WB_RegWrite && (MEM_WB_WriteRegister != 0) && (MEM_WB_WriteRegister == ID_EX_rt))
      ForwardB = 2'b01;
  end

endmodule
