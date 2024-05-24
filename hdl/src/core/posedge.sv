`timescale 1ns / 1ps

module posedge_detector (
  input  logic clk,
  input  logic signal,
  output logic out
);

  logic signalPrev;

  always_ff @(posedge clk) begin
    signalPrev <= signal;
    out       <= (signal && !signalPrev);
  end

endmodule