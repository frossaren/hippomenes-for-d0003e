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
    output logic scl,
    output logic sda
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

  // SCL register will be cycled once every 250 normal clock cycles
  // 100MHz / 250 = 400KHz
  // Quadrants of 250: 0-61, 62-124, 125-187, 188-249
  logic scl_internal = 1'b0;
  logic scl_stretch = 1'b0;
  logic sda_clk = 1'b0; // Clock signal that SDA line is clear to update (SCL is low)
  reg [7:0] scl_counter = '0;

  always_ff @(posedge sysclk or posedge reset) begin
    if (reset) begin
      scl_counter <= '0;
      scl_stretch <= 1'b0;
    end else begin
      // Increase counter if it's not being stretched, or reset at 249
      if (scl_counter == 249) begin
        scl_counter <= '0;
      end else if (scl_stretch == 0) begin
        scl_counter <= scl_counter + 1'b1;
      end

      // Send a proper data clock and SCL signal depending on where in the SCL cycle we are
      if (scl_counter < 62) begin
        scl_internal <= 1'b0;
        sda_clk <= 1'b0;
      end else if (scl_counter > 61 && scl_counter < 125) begin
        scl_internal <= 1'b0;
        sda_clk <= 1'b1;
      end else if (scl_counter > 124 && scl_counter < 188) begin
        scl_internal <= 1'b1;
        // Check for slave stretching clk
        if (scl == 1'b0) begin
          scl_stretch <= 1'b1;
        end
        sda_clk <= 1'b1;
      end else begin
        scl_internal <= 1'b1;
        sda_clk <= 1'b0;
      end
    end
  end

  always_ff @(posedge sda_clk or posedge reset) begin
    if (reset) begin
      state <= Ready;
    end else begin
      state <= next_state;
    end
  end

  // State will update on every edge of clk
  // always_comb begin
  //   case (state)
  //     Ready: begin
  //       if (enable) begin
  //         addr_buff <= address;
  //         data_buff <= data;
  //         rw_buff <= rw;
  //         busy <= 1'b1;
  //         next_state <= Start_Transmission;
  //       end else begin
  //         busy <= 1'b0;
  //         next_state <= Ready;
  //       end
  //     end

  //     Start_Transmission: begin
  //       sda <= 1'b0;
  //       next_state <= Start_Clock;
  //     end

  //     Start_Clock: begin
  //       if (!clk) begin
  //         scl <= 1'b0;
  //         next_state <= Send_Command_Bit;
  //       end
  //     end

  //     Send_Command_Bit: begin
  //       if ()
  //     end
      
  //     Slave_Ack1: begin

  //     end
      
  //     Send_Write_Bit: begin

  //     end
      
  //     Slave_Ack2: begin

  //     end
      
  //     Read_Bit: begin

  //     end
      
  //     Master_Ack: begin

  //     end
      
  //     Stop_Transmission: begin

  //     end
      
  //     default: begin

  //     end
  //   endcase
  // end

  // TODO: TEMP
  assign scl = scl_counter[0];
  assign sda = scl_counter[1];

endmodule
