// tb_csr
`timescale 1ns / 1ps

import decoder_pkg::*;
module tb_csr;

  localparam integer unsigned CsrWidth = 32;  // default to word
  localparam type CsrDataT = logic [CsrWidth-1:0];  // derived

 (* DONT_TOUCH = "TRUE" *)
  logic clk;
   (* DONT_TOUCH = "TRUE" *)
  logic reset;
   (* DONT_TOUCH = "TRUE" *)
  logic csr_enable;
   (* DONT_TOUCH = "TRUE" *)
  csr_addr_t csr_addr;
   (* DONT_TOUCH = "TRUE" *)
  r rs1_zimm;
   (* DONT_TOUCH = "TRUE" *)
  word rs1_data;
   (* DONT_TOUCH = "TRUE" *)
  csr_op_t csr_op;
   (* DONT_TOUCH = "TRUE" *)
  CsrDataT ext_data;
   (* DONT_TOUCH = "TRUE" *)
  logic ext_write_enable;
  
 (* DONT_TOUCH = "TRUE" *)
  word out; 
  // logic match;
  
  (* DONT_TOUCH = "TRUE" *)
  csr #(.CsrWidth(CsrWidth)) dut (
      // in
      .clk,
      .reset,
      .csr_enable,
      .csr_addr,
      .rs1_data,
      .csr_op,
      .rs1_zimm,
      .ext_data,
      .ext_write_enable,
      // out
      //.match(match),
      .out(out)
  );

  initial begin
//    $dumpfile("csr.fst");
//    $dumpvars;

    csr_addr = 0;  // should match
    reset = 1;
    #15;
    reset = 0;
    #5;

    ext_data = 0;
    ext_write_enable = 0;

    clk = 0;
    csr_enable = 1;

    rs1_data = 'b1011;
    csr_op = CSRRW;
    // notice out will be delayed
    $display("CSRRW 'b1011 out %h", out);
    assert (out == 0);

    #20;
    $display("wait out %h", out);
    assert (out == 'b1011);
    // assert (match == 1);
    rs1_data = 'b1100;
    csr_op   = CSRRS;
    $display("CSRRS 'b1100 out %h", out);

    #20;
    $display("wait out %h", out);
    assert (out == 'b1111);
    rs1_data = 'b1100;
    csr_op   = CSRRC;
    $display("CSRRC 'b1100 %h", out);

    #20;
    $display("wait out %h", out);
    assert (out == 'b0011);

    rs1_zimm = 1;
    csr_op   = CSRRWI;
    $display("CSRRWI rs1 =1 out %h", out);

    #20;
    $display("wait out %h", out);
    assert (out == 'b0001);

    rs1_zimm = 2;
    csr_op   = CSRRSI;
    $display("CSRRSI rs1 =2 out %h", out);

    #20;
    $display("wait out %h", out);
    assert (out == 'b0011);

    rs1_zimm = 1;
    csr_op   = CSRRCI;
    $display("CSRRCI rs1 =1 out %h", out);

    #20;
    $display("wait out %h", out);
    assert (out == 'b0010);

    rs1_zimm = 1;
    csr_op   = CSRRCI;
    $display("CSRRCI rs1 =1 out %h --- dummy just to get out", out);

    csr_addr = 1;  // should cause address miss match
    #1;
    assert (out == 0);


    #20 $finish;
  end
  
  always #10 clk = ~clk;
endmodule
