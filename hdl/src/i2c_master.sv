// i2c_master
`timescale 1ns / 1ps

/* TODO:
 * Create a clock structure that generates the necessary 400KHz
 * From that structure, generate a "data clock" that signals when to update SDA line
 * Allow enabling/disabling of output of SCL
 * Finish FSM to allow writing and reading of data
 */


// I2C master controller, supports clock stretching
module i2c_master (
    input sysclk,
    input logic reset,
    input logic enable,
    input logic rw,
    output logic busy,

    input logic [6:0] address,
    inout logic [7:0] data,

    output logic scl,
    inout logic sda
);

  /* 
   * States:
   * Ready
   * Start transmission
   * Send command bit
   * Slave Ack 1
   * Send write bit
   * Slave Ack 2
   * Read bit
   * Master Ack
   * Stop transmission
   */
  typedef enum { 
    Ready,
    Start_Transmission,
    Start_Clock,
    Send_Command_Bit,
    Slave_Ack1,
    Send_Write_Bit,
    Slave_Ack2,
    Read_Bit,
    Master_Ack,
    Stop_Clock,
    Stop_Transmission
  } State;

  State state, next_state;
  reg [4:0] bits_left;
  reg [7:0] data_buff;
  reg [6:0] addr_buff;
  reg rw_buff;

  logic clk; // TODO

  always_ff @(posedge clk or negedge clk or posedge reset) begin
    if (reset) begin
      state <= Ready;
    end else begin
      state <= next_state;
    end
  end

  // State will update on every edge of clk
  always_comb begin
    case (state)
      Ready: begin
        if (enable) begin
          addr_buff <= address;
          data_buff <= data;
          rw_buff <= rw;
          busy <= 1'b1;
          next_state <= Start_Transmission;
        end else begin
          busy <= 1'b0;
          next_state <= Ready;
        end
      end

      Start_Transmission: begin
        sda <= 1'b0;
        next_state <= Start_Clock;
      end

      Start_Clock: begin
        if (!clk) begin
          scl <= 1'b0;
          next_state <= Send_Command_Bit;
        end
      end

      Send_Command_Bit: begin
        if ()
      end
      
      Slave_Ack1: begin

      end
      
      Send_Write_Bit: begin

      end
      
      Slave_Ack2: begin

      end
      
      Read_Bit: begin

      end
      
      Master_Ack: begin

      end
      
      Stop_Transmission: begin

      end
      
      default: begin

      end
    endcase
  end

endmodule
