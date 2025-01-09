// i2c_master
`timescale 1ns / 1ps

/* TODO:
 * Create a clock structure that generates the necessary 400KHz from main 100MHz
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

    // TEMP
    inout logic scl,
    inout logic sda
);

  reg [4:0] bits_left;
  reg [7:0] data_buff;
  reg [6:0] addr_buff;
  reg rw_buff;

  logic scl_stretch = 1'b0;
  logic state_tick = 1'b0; // Tick to update state
  reg [4:0] tick_counter = '0;

  // Internal values, write 1s here as Z on the real bus (to permit clock stretch)
  logic scl_internal = 1'b0;
  logic sda_internal = 1'b0;

  logic test = 1'b0;
  logic test2 = 1'b0;

  // System clock runs at 20 000 KHz
  // We need 4 ticks per bit @ 400KHz, so use 2 bit divider (divide by 4) for 1600KHz ticks
  always_ff @(posedge sysclk or posedge reset) begin
    if (reset) begin
      tick_counter <= '0;
      scl_stretch <= 1'b0;
    end else begin
      if (tick_counter == 25) {
        tick_counter <= '0;
      }

      // Increase counter if it's not being stretched, or reset at 249
      if (scl_stretch == 0) begin
        tick_counter <= tick_counter + 1'b1;
      end

      // Update tick
      if (tick_counter < 2) begin
        state_tick <= 1'b1;
      end else begin
        state_tick <= 1'b0;
      end
    end
  end


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
  reg [1:0] process_counter;

  always_ff @(posedge state_tick or posedge reset) begin
    if (reset) begin
      state <= Ready;
      process_counter <= '0;
    end else begin
      // Increment the process counter; on every 4th, change to next state
      process_counter <= process_counter + 1;
      if (process_counter == 0) begin
        state <= next_state;
      end
    end
  end

  // State will update on every edge of clk
  always_comb begin
    case (state)
      Ready: begin
        sda_internal <= 1;
        scl_internal <= 1;

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
        // sda <= 1'b0;
        // next_state <= Start_Clock;

        // Temp
        sda_internal <= 1;
        scl_internal <= 0;
        next_state <= Ready;
      end

      // Start_Clock: begin
      //   if (!clk) begin
      //     scl <= 1'b0;
      //     next_state <= Send_Command_Bit;
      //   end
      // end

      // Send_Command_Bit: begin
      //   if ()
      // end
      
      // Slave_Ack1: begin

      // end
      
      // Send_Write_Bit: begin

      // end
      
      // Slave_Ack2: begin

      // end
      
      // Read_Bit: begin

      // end
      
      // Master_Ack: begin

      // end
      
      // Stop_Transmission: begin

      // end
      
      default: begin

      end
    endcase
  end

  // TODO: TEMP
  assign scl = process_counter[0];
  assign sda = process_counter[1];

endmodule
