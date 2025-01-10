// top_arty
`timescale 1ns / 1ps

import config_pkg::*;
import arty_pkg::*;
import decoder_pkg::*;

module top_arty (
    input  logic clk,
    input  logic reset,
    input  BtnT  btn,
    output LedT  led,
    output logic tx,
    // TODO: gpio

    // I2C
    inout logic scl,
    inout logic sda
);
  IMemAddrT pc_interrupt_mux_out;
  // registers
  IMemAddrT pc_reg_out;
  reg_n #(
      .DataWidth(IMemAddrWidth)
  ) pc_reg (
      .clk(clk),
      .reset(reset),
      .in(pc_interrupt_mux_out),
      .out(pc_reg_out)
  );

  // pc related
  word alu_res;
  pc_branch_mux_t branch_logic_out;
  IMemAddrT pc_adder_out;
  IMemAddrT pc_branch_mux_out;
  pc_branch_mux #(
      .AddrWidth(IMemAddrWidth)
  ) pc_branch_mux (
      .sel(branch_logic_out),
      .pc_next(pc_adder_out),
      .pc_branch(IMemAddrWidth'(alu_res)),
      .out(pc_branch_mux_out)
  );

  IMemAddrT n_clic_interrupt_addr;
  pc_interrupt_mux_t n_clic_pc_interrupt_sel;
  pc_interrupt_mux #(
      .AddrWidth(IMemAddrWidth)
  ) pc_interrupt_mux (
      .sel(n_clic_pc_interrupt_sel),
      .pc_normal(pc_branch_mux_out),
      .pc_interrupt(n_clic_interrupt_addr),
      .out(pc_interrupt_mux_out)
  );

  // adder
  pc_adder #(
      .AddrWidth(IMemAddrWidth)
  ) pc_adder (
      .in (pc_reg_out),
      .out(pc_adder_out)
  );

  // instruction memory
  word imem_data_out;

`ifdef VERILATOR
  rom imem (
      // in
      .clk(clk),
      .address(pc_reg_out[IMemAddrWidth-1:0]),
      // out
      .data_out(imem_data_out)
  );
`else
  spram imem (
      // in
      .clk(clk),
      // used with 1-cycle latency spram read
      .address(pc_interrupt_mux_out[IMemAddrWidth-1:0]),
      // used with combinational 0-cycle latency spram read
      // .address(pc_reg_out[IMemAddrWidth-1:0]),
      .reset,
      // out
      .data_out(imem_data_out)
  );
`endif

  // decoder
  wb_mux_t decoder_wb_mux_sel;
  alu_a_mux_t decoder_alu_a_mux_sel;
  alu_b_mux_t decoder_alu_b_mux_sel;
  alu_op_t decoder_alu_op;
  mul_op_t decoder_mul_op;
  logic decoder_sub_arith;
  word decoder_imm;
  r decoder_rs1;
  r decoder_rs2;

  // mem
  logic decoder_dmem_write_enable;
  logic decoder_dmem_sign_extend;
  mem_width_t decoder_mem_with;

  // branch
  logic decoder_branch_instr;
  branch_op_t decoder_branch_op;
  logic decoder_branch_always;

  // csr
  logic decoder_csr_enable;
  csr_op_t decoder_csr_op;
  CsrAddrT decoder_csr_addr;
  mem_width_t decoder_dmem_width;
  r decoder_rd;

  // write back
  logic decoder_wb_write_enable;

  decoder decoder (
      // in
      .instr(imem_data_out),
      // out
      .csr_addr(decoder_csr_addr),
      // register file
      .rs1(decoder_rs1),
      .rs2(decoder_rs2),
      .imm(decoder_imm),
      // branch logic
      .branch_always(decoder_branch_always),
      .branch_instr(decoder_branch_instr),
      .branch_op(decoder_branch_op),
      // alu
      .alu_a_mux_sel(decoder_alu_a_mux_sel),
      .alu_b_mux_sel(decoder_alu_b_mux_sel),
      .alu_op(decoder_alu_op),
      .sub_arith(decoder_sub_arith),
      .mul_op(decoder_mul_op),
      // data memory
      .dmem_write_enable(decoder_dmem_write_enable),
      .dmem_sign_extend(decoder_dmem_sign_extend),
      .dmem_width(decoder_dmem_width),
      // csr
      .csr_enable(decoder_csr_enable),
      .csr_op(decoder_csr_op),
      // write back
      .wb_mux_sel(decoder_wb_mux_sel),
      .rd(decoder_rd),
      .wb_write_enable(decoder_wb_write_enable)
  );

  // register file
  word  wb_mux_out;
  word  rf_rs1;
  word  rf_rs2;
  logic n_clic_interrupt_out;
  word  rf_stack_ra;

  PrioT n_clic_level_out;
  rf_stack rf (
      // in
      .clk,
      .reset,
      .writeEn(decoder_wb_write_enable),
      .writeRaEn(n_clic_interrupt_out),
      .level(n_clic_level_out),
      .writeAddr(decoder_rd),
      .writeData(wb_mux_out),
      .readAddr1(decoder_rs1),
      .readAddr2(decoder_rs2),
      // out
      .readData1(rf_rs1),
      .readData2(rf_rs2)
  );

  // branch logic
  branch_logic branch_logic (
      // in
      .a(rf_rs1),
      .b(rf_rs2),
      .branch_always(decoder_branch_always),
      .branch_instr(decoder_branch_instr),
      .op(decoder_branch_op),
      // out
      .out(branch_logic_out)
  );

  // Alu related
  word alu_a_mux_out;
  alu_a_mux alu_a_mux (
      // in
      .sel (decoder_alu_a_mux_sel),
      .imm (decoder_imm),
      .rs1 (rf_rs1),
      .zero(32'(0)),
      // out
      .out (alu_a_mux_out)
  );

  word alu_b_mux_out;
  alu_b_mux alu_b_mux (
      // in
      .sel      (decoder_alu_b_mux_sel),
      // out
      .rs2      (rf_rs2),
      .imm      (decoder_imm),
      .pc_plus_4(32'($signed(pc_adder_out))),  // Should we sign extend?
      .pc       (32'($signed(pc_reg_out))),    //
      .out      (alu_b_mux_out)
  );


  alu alu (
      .a(alu_a_mux_out),
      .b(alu_b_mux_out),
      .sub_arith(decoder_sub_arith),
      .op(decoder_alu_op),
      .res(alu_res)
  );
  word mul_res;
  mul mul (
      .a  (alu_a_mux_out),
      .b  (alu_b_mux_out),
      .op (decoder_mul_op),
      .res(mul_res)
  );

  word  dmem_data_out;
  logic dmem_alignment_error;
  mem dmem (
      // in
      .clk(clk),
      .write_enable(decoder_dmem_write_enable),
      .width(decoder_dmem_width),
      .sign_extend(decoder_dmem_sign_extend),
      .address(alu_res[DMemAddrWidth-1:0]),
      .data_in(rf_rs2),
      // out
      .data_out(dmem_data_out),
      .alignment_error(dmem_alignment_error)
  );

  // led out
  word csr_led_out;
  word csr_led_direct_out;  // currently not used
  csr #(
      .CsrWidth(LedWidth),
      .Addr(LedAddr)
  ) csr_led (
      // in
      .clk,
      .reset,
      .csr_enable(decoder_csr_enable),
      .csr_addr(decoder_csr_addr),
      .rs1_zimm(decoder_rs1),
      .rs1_data(rf_rs1),
      .csr_op(decoder_csr_op),
      .ext_data(0),
      .ext_write_enable(0),
      // out
      .direct_out(csr_led_direct_out),
      .out(csr_led_out)
  );
  assign led = LedT'(csr_led_out[LedWidth-1:0]);
  // assign rx = csr_led_out[LedWidth-1];  //last pin is RX

  // Button input
  word csr_btn_out;
  word csr_btn_direct_out;  // currently not used
  csr #(
      .CsrWidth(BtnWidth),
      .Addr(BtnAddr),
      .Write(0)  // only readable register
  ) csr_btn (
      // in
      .clk,
      .reset,
      .csr_enable(decoder_csr_enable),
      .csr_addr(decoder_csr_addr),
      .rs1_zimm(decoder_rs1),
      .rs1_data(rf_rs1),
      .csr_op(decoder_csr_op),
      .ext_data(0),
      .ext_write_enable(0),
      // out
      .direct_out(csr_btn_direct_out),
      .out(csr_btn_out)
  );
  assign csr_btn.data = btn;


  // TODO: GPIO
  //   word csr_gpio_dir_out;
  //   word csr_gpio_direct_out;  // not used
  //   assign gpio_dir = GpioT'(csr_gpio_dir_out);
  //   csr #(
  //       .CsrWidth(GpioNum),  // Number of GPIOs
  //       .Addr(GpioCsrDir)  // Direction register
  //   ) csr_gpio_dir (
  //       // in
  //       .clk,
  //       .reset,
  //       .csr_enable(decoder_csr_enable),
  //       .csr_addr(decoder_csr_addr),
  //       .rs1_zimm(decoder_rs1),
  //       .rs1_data(rf_rs1),
  //       .csr_op(decoder_csr_op),
  //       .ext_data(0),
  //       .ext_write_enable(0),
  //       // out
  //       .direct_out(csr_gpio_direct_out),  // not used
  //       .out(csr_gpio_dir_out)
  //   );

  //   word csr_gpio_data_out;
  //   csr_gpio csr_gpio_data (
  //       // in
  //       .clk,
  //       .reset,
  //       .csr_enable(decoder_csr_enable),
  //       .csr_addr(decoder_csr_addr),
  //       .rs1_zimm(decoder_rs1),
  //       .rs1_data(rf_rs1),
  //       .csr_op(decoder_csr_op),
  //       .ext_data(0),
  //       .ext_write_enable(0),
  //       .direction(GpioT'(csr_gpio_dir_out)),
  //       // out
  //       .out(csr_gpio_data_out),
  //       // gpi
  //       .gpio_in,
  //       .gpio_out
  //   );

  // TEMP
  // Send a single cycle signal to NCLIC if btn0 is pressed
  logic button_pressed;
  
  posedge_detector posedge_detector (
    .clk,
    .signal(btn[0]),
    .out(button_pressed)
  );

  word n_clic_csr_out;
  n_clic n_clic (
      // in
      .clk,
      .reset,
      .csr_enable(decoder_csr_enable),
      .csr_addr(decoder_csr_addr),
      .rs1_zimm(decoder_rs1),
      .rs1_data(rf_rs1),
      //.rd(decoder_rd),
      .csr_op(decoder_csr_op),
      .pc_in(pc_branch_mux_out),
      // External (button) interrupt
      .ext_interrupt(button_pressed),

      // out
      .csr_out(n_clic_csr_out),
      .int_addr(n_clic_interrupt_addr),
      .pc_interrupt_sel(n_clic_pc_interrupt_sel),
      .level_out(n_clic_level_out),
      .interrupt_out(n_clic_interrupt_out)
  );

  logic uart_next;
  word  uart_fifo_data;
  word  uart_fifo_csr_data_out;
  logic uart_fifo_have_next;
  fifo #(
    .Addr(UARTFifoCsrAddr),
    .PtrSize(UARTFifoPtrSize),
    .QueueSize(UARTFifoQueueSize)
  ) i_fifo (
      .clk_i(clk),
      .reset_i(reset),
      .next(uart_next),
      .csr_enable(decoder_csr_enable),
      .csr_addr(decoder_csr_addr),
      .rs1_zimm(decoder_rs1),
      .rs1_data(rf_rs1),
      .csr_op(decoder_csr_op),
      .data(uart_fifo_data),
      .csr_data_out(uart_fifo_csr_data_out),
      .have_next(uart_fifo_have_next)
  );
  uart i_uart (
      .clk_i(clk),
      .reset_i(reset),
      .prescaler(0),
      .d_in(uart_fifo_data),
      .rts(uart_fifo_have_next),
      .tx,
      .next(uart_next)
  );

// I2C master and fifo send/write buffer
  logic i2c_fifo_next;
  word  i2c_fifo_data;
  word  i2c_fifo_csr_data_out;
  logic i2c_fifo_have_next;

  // TEMP
  // Start transmission if button4 is pressed
  logic start_pressed;
  posedge_detector posedge_detector2 (
    .clk,
    .signal(btn[3]),
    .out(start_pressed)
  );

  logic scl_i;
  logic scl_o;
  logic scl_t;
  logic sda_i;
  logic sda_o;
  logic sda_t;
  fifo #(
    .Addr(I2CFifoCsrAddr),
    .PtrSize(I2CFifoPtrSize),
    .QueueSize(I2CFifoQueueSize)
  ) i2c_fifo (
      .clk_i(clk),
      .reset_i(reset),
      .next(i2c_fifo_next & i2c_fifo_have_next),
      .csr_enable(decoder_csr_enable),
      .csr_addr(decoder_csr_addr),
      .rs1_zimm(decoder_rs1),
      .rs1_data(rf_rs1),
      .csr_op(decoder_csr_op),
      .data(i2c_fifo_data),
      .csr_data_out(i2c_fifo_csr_data_out),
      .have_next(i2c_fifo_have_next)
  );
  i2c_master i2c (
    // Prescale by 12.5 (20000000/(4*400000) = 12.5). 13 is the closest
    .clk,
    .rst(reset),
    .prescale(13),

    // SCL/SDA pins
    .scl_i,
    .scl_o,
    .scl_t,
    .sda_i,
    .sda_o,
    .sda_t,

    // TEMP
    .s_axis_cmd_address(7'h27),
    .s_axis_cmd_valid('1),
    // .s_axis_data_tdata(8'h34),
    // .s_axis_data_tvalid('1),
    // .s_axis_data_tlast(btn[3]), // stop send
    .s_axis_cmd_write_multiple(start_pressed), // start send

    // Connect to send buffer FIFO
    .s_axis_data_tdata(i2c_fifo_data), // Data from FIFO
    .s_axis_data_tlast(!i2c_fifo_have_next), // FIFO has data, means not the last
    .s_axis_data_tready(i2c_fifo_next), // When I2C becomes ready (sends a new byte), load next word from fifo
    .s_axis_data_tvalid('1) // Should this always be true?
  );

  // Combine separate i/o/t into single inout channels (with Z for high to allow pull by slave)
  assign scl_i = scl;
  assign scl = scl_t ? 1'bz : scl_o;
  assign sda_i = sda;
  assign sda = sda_t ? 1'bz : sda_o;

  word csr_out;
  // match CSR addresses
  always_comb begin
    // TODO: for now only btn
    case (decoder_csr_addr)
      BtnAddr: csr_out = csr_btn_out;
      default: csr_out = 0;
    endcase
  end

  wb_mux wb_mux (
      .sel(decoder_wb_mux_sel),
      .dm(dmem_data_out),
      .alu(alu_res),
      .csr(csr_out),
      .pc_plus_4(32'($signed(pc_adder_out))),  // should we sign extend?
      .mul(mul_res),
      .out(wb_mux_out)
  );
endmodule
