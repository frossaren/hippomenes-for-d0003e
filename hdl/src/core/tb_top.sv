// top module

// import pc_adder::*;
// import register_file::*;
// import decoder::*;

module tb_top;
    logic pc_mux_select;
    logic [31:0] pc_mux_next;
    logic [31:0] pc_mux_branch;
    logic [31:0] pc_mux_out;

    PcMux pc_mux(
        .select(pc_mux_select),
        .next(pc_mux_next),
        .branch(pc_mux_branch),
        .out(pc_mux_out)
    );

    initial begin
    $dumpfile("top.vcd");
    $dumpvars;

    pc_mux_select = 0;
    pc_mux_next = 'h0000_0004;
    pc_mux_branch = 'h0000_0008;


    #10;

    pc_mux_select = 1;


    #10 $finish;
    end

endmodule