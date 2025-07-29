/* ************************************************************************* *
 * @file      peripheral_library.v
 * @author    nitelich
 * @since     2024-06-10
 * @copyright Copyright &copy; 2025 nitelich, all rights reserved
 * @brief
 *
 * A collection of MCU-adjacent peripherals using only "low level" primitive
 * functions such as bitwise logic, multiplexers (as inferred from ternary
 * expressions), and other simple combinatorial and register logic.
 *
 * @details
 *
 * Contains implementations of the following high-level modules:
 *   - Register File (2 read, 2 write)
 *   - Asynchronous Counter
 *      - Pulse Width Modulation Peripheral
 *   - Shift Register
 *      - Linear Feedback Shift Register
 *      - Alternating Step Generator
 *      - Serial Peripheral Interface Peripheral (Controller version)
 *      - Universal Asynchronous Receiver Transmitter Peripheral
 *
 * No potential future extensions identified at this time.
 *
 * ************************************************************************* */

/**
 * Set a simulation timescale of 1ns per "tick" and a display resolution
 * of 100ps.
 */
`timescale 1ns/1ns

/**
 * Library version history:
 *   v0.1.2 - Added Tiny Tapeout Conway's Game of Life.
 *            Started 2025-06-24, Completed 2025-07-02
 *   v0.1.1 - Adding peripheral SPI and learning too much about metastability.
 *            Started 2025-02-10, Completed 2025-03-19
 *   v0.1.0 - Initial feature complete version.
 *            Started 2024-06-10, Completed 2025-01-30
 */
`define VERSION_PERIPH "0.1.1"

// Include the shared definitions file.
`include "shared_definitions.v"

/* Helper Macros *********************************************************** */

/* Primary Modules ********************************************************* */

/**
 * Two-read, two-write register file. Simultaneous read from and write to
 * the same address result in a read-through of the written value.
 * @tparam[in] WIDTH The width of data stored in the register file.
 * @tparam[in] DEPTH Log2 of the number of registers in the file. That is,
 * a depth of 5 would give (2 ** 5) = 32 registers. Represented as a log
 * value to significantly simplify error logic.
 * @tparam[in] READ_THRU If non-zero, allow read-through of register values
 * as they are being written. If read-through is allowed, a tight loop can
 * easily be formed by writing a value based on the register read without
 * buffering that same data.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset.
 * @param[in] addr_write_0 The first write address.
 * @param[in] addr_write_1 The second write address.
 * @param[in] addr_read_0 The first read address.
 * @param[in] addr_read_1 The second read address.
 * @param[in] en_write_0 The first write enable.
 * @param[in] en_write_1 The second write enable.
 * @param[in] data_write_0 Data to write to the first address.
 * @param[in] data_write_1 Data to write to the second address.
 * @param[out] data_read_0 Data to read from the first address.
 * @param[out] data_read_1 Data to read from the second address.
 * @param[out] err Indicates simultaneous writes to the same address.
 * When an error is indicated, only the first value is written, and any read-
 * through values take the first write.
 * @note 2024-06-14 Tested for the following width/depth combos:
 *       Full: 8x{1-4}, {1-8}x4, 16x4, 24x4, 32x4, 32x5
 */
module RegisterFile#(parameter WIDTH = 32, parameter DEPTH = 5, parameter READ_THRU = 0)
                    (input clk, input rst,
                    input [DEPTH - 1:0] addr_write_0, input [DEPTH - 1:0] addr_write_1,
                    input [DEPTH - 1:0] addr_read_0, input [DEPTH - 1:0] addr_read_1,
                    input en_write_0, input en_write_1,
                    input [WIDTH - 1:0] data_write_0, input [WIDTH - 1:0] data_write_1,
                    output [WIDTH - 1:0] data_read_0, output [WIDTH - 1:0] data_read_1, output err
                    );
   
   // ----- Constants -----

   // ----- Local Variables -----
   integer i;            /**< Loop counter.*/
   reg     err_internal; /**< Internal error value.*/
   reg [WIDTH-1:0] registers [0:(2 ** DEPTH) - 1]; /**< Register data.*/

   // ----- Combinatorial Logic -----

   // Assign output values.
   // For simultaneous writes, directly pass the written value (when enabled).
   generate
      if (READ_THRU > 0) begin
         assign data_read_0 = ((addr_read_0 == addr_write_0) & en_write_0) ? data_write_0 :
            (((addr_read_0 == addr_write_1) & en_write_1) ? data_write_1 : registers[addr_read_0]);
         assign data_read_1 = ((addr_read_1 == addr_write_0) & en_write_0) ? data_write_0 :
            (((addr_read_1 == addr_write_1) & en_write_1) ? data_write_1 : registers[addr_read_1]);
      end else begin
         assign data_read_0 = registers[addr_read_0];
         assign data_read_1 = registers[addr_read_1];
      end
   endgenerate
   
   // Forward error value to output.
   assign err = err_internal | (en_write_0 & en_write_1 & (addr_write_0 == addr_write_1));

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the register file.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : file_registers
      
      // Initialize to zeroes on reset
      if (rst) begin
         err_internal <= 1'b0;
         for (i = 0; i < (2 ** DEPTH); i = i + 1) begin
            registers[i] <= {WIDTH{1'b0}};
         end
      
      // Otherwise update the register state
      end else begin
      
         // Determine error conditions
         err_internal <= (en_write_0 & en_write_1 & (addr_write_0 == addr_write_1));
         
         // By default, keep the register value.
         for (i = 0; i < (2 ** DEPTH); i = i + 1) begin
            registers[i] <= registers[i];
         end
         
         // Iterate through write requests.
         // Lower indices get priority, so are written last.
         if (en_write_1) begin
            registers[addr_write_1] <= data_write_1;
         end
         if (en_write_0) begin
            registers[addr_write_0] <= data_write_0;
         end
      end
   end

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * Synchronous N-bit up/down counter, with loadable reset value.
 * @tparam[in] WIDTH The width of data stored in the counter.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset.
 * @param[in] dir The count direction (0 = down, 1 = up).
 * @param[in] def The default value, used on reset and overflow.
 * @param[out] val The current count value.
 * @param[out] ovf A count overflow indicator (0 = normal, 1 = overflow).
 * @note 2024-06-16 Tested bit-widths:
 *       Full:   1-20
 */
module CounterSync#(parameter WIDTH = 32)
                   (input clk, input rst, input dir, output ovf,
                   input [WIDTH - 1:0] def, output [WIDTH - 1:0] val);
   
   // ----- Constants -----

   // ----- Local Variables -----
   genvar  j;            /**< Assignment loop counter.*/
   integer i;            /**< Loop counter.*/
   reg     ovf_internal; /**< Internal overflow value.*/
   wire [WIDTH - 1:0] all_up;       /**< Indicates when all previous values count up.*/
   wire [WIDTH - 1:0] all_down;     /**< Indicates when all previous values count down.*/
   reg  [WIDTH - 1:0] val_internal; /**< Internal count value.*/
   
   // ----- Combinatorial Logic -----
   
   // Determine when previous values flip, and so this one can.
   generate
      assign all_up[0]   = dir;
      assign all_down[0] = ~dir;
      for (j = 1; j < WIDTH; j = j + 1) begin : up_and_down
         assign all_up[j]   =  val_internal[j - 1] & all_up[j - 1];
         assign all_down[j] = ~val_internal[j - 1] & all_down[j - 1];
      end
   endgenerate
   
   // Forward counter values to output.
   assign val = val_internal;
   assign ovf = ovf_internal;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the synchronous counter.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : count_registers
      
      // Initialize to default value on reset
      if (rst) begin
         val_internal <= def;
         ovf_internal <= 1'b0;
      
      // Otherwise update the register state
      end else begin
      
         // Determine overflow conditions
         ovf_internal <= ( val_internal[WIDTH - 1] & all_up[WIDTH - 1]) |
                         (~val_internal[WIDTH - 1] & all_down[WIDTH - 1]);
         
         // Update counter values
         for (i = 0; i < WIDTH; i = i + 1) begin
            val_internal[i] <= val_internal[i] ^ (all_up[i] | all_down[i]);
         end
         
         // Reset to the default value rather than wrapping
         if (( val_internal[WIDTH - 1] & all_up[WIDTH - 1]) |
             (~val_internal[WIDTH - 1] & all_down[WIDTH - 1])) begin
            val_internal <= def;
         end
      end
   end

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * Configurable pulse width modulation peripheral.
 * @note Bit widths of (N < 2) are not supported, as a signed integer
 *       comparator with this limitation is utilized for PWM period
 *       and duty comparison.
 * @tparam[in] WIDTH The width of data stored in the internal counter.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset.
 * @param[in] dir The count direction (0 = down, 1 = up).
 * @param[in] period_0 The initial/minimum period for counting.
 * @param[in] period_f The final/maximum period for counting.
 * @param[in] period_d The duty swap period.
 * @param[out] val The current count value.
 * @param[out] pwm The pulse width modulated value.
 * @param[out] ovf A count overflow indicator (0 = normal, 1 = overflow).
 * @note 2024-06-21 Tested bit-widths:
 *       Full:   2-7
 *       This module takes very significant time to test for N >= 6 due to all
 *       parameters being full-coverage tested (leading to O(N^4) runtime)
 */
module PulseWidthModulator#(parameter WIDTH = 32)
                           (input clk, input rst, input dir, output pwm, output ovf,
                           input [WIDTH - 1:0] period_0, input [WIDTH - 1:0] period_f,
                           input [WIDTH - 1:0] period_d, output [WIDTH - 1:0] val);
   
   // ----- Constants -----

   // ----- Local Variables -----
   wire count_gt;       /**< Count is greater than final count.*/
   wire count_lt;       /**< Count is less than final count.*/
   wire count_eq;       /**< Count is equal to final count.*/
   wire count_complete; /**< Counting has completed for this period.*/
   wire duty_gt;        /**< Count is greater than duty point.*/
   wire duty_eq;        /**< Count is equal to duty point.*/
   wire rst_all;        /**< Combination of all counter reset sources.*/
   wire ovf_count;      /**< Overflow signal from counter.*/
   wire [WIDTH - 1:0] period_init;  /**< Initial period for counting.*/
   wire [WIDTH - 1:0] period_final; /**< Final period for counting.*/
   wire [WIDTH - 1:0] val_internal; /**< Internal count value.*/
   
   // ----- Combinatorial Logic -----
   
   // Determine the direction-appropriate start and end period values
   assign period_init  = (dir) ? period_0 : period_f;
   assign period_final = (dir) ? period_f : period_0;
   
   // Determine when counting has completed based on the count comparator
   assign count_complete = (dir & (count_gt | count_eq)) |
                           (~dir & (count_lt | count_eq));
   
   // Determine when to reset the counter (normal vs. complete)
   assign rst_all = rst | count_complete;
   
   // Determine when the PWM has overflowed (normal vs. complete)
   assign ovf = ovf_count | count_complete;
   
   // Set output values
   assign pwm = duty_gt | duty_eq;
   assign val = val_internal;

   // ----- Register Logic -----

   // ----- Sub-Modules -----
   
   /** Counter underlying the PWM calculation.*/
   CounterSync #(.WIDTH(WIDTH))
               counter(.clk(clk), .rst(rst_all), .dir(dir), .ovf(ovf_count),
                       .def(period_init), .val(val_internal));

   /** Comparator for period calculations.*/
   ComparatorKS #(.WIDTH(WIDTH))
                cmp_period(.a(val_internal), .b(period_final), .sign(1'b0),
                           .gt(count_gt), .lt(count_lt), .eq(count_eq));

   /** Comparator for duty calculations.*/
   ComparatorKS #(.WIDTH(WIDTH))
                cmp_duty(.a(val_internal), .b(period_d), .sign(1'b0),
                         .gt(duty_gt), .lt(), .eq(duty_eq));

// ----- End of Module -----
endmodule

/**
 * N-bit parallel-to/from-serial shift register.
 * @tparam[in] WIDTH The width of data stored in the shift register.
 * @tparam[in] DEPTH The depth of the clock divider.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module aborts
 * in-progress data transfers.
 * @param[in] cdiv A clock divider counter, with zero being no division
 * and values greater than zero indicating the number of clock cycles
 * between shift transactions. In other words, the slow clock frequency
 * f.slow = f.clk / (cdiv + 1). Hence also the "clock ready" check
 * is against cdiv rather than (cdiv - 1).
 * @param[in] cpha The capture phase for shifting in data (0 = first
 * clock edge after trigger, 1 = second clock edge after trigger).
 * @param[in] dir The shift direction (0 = MSB first, 1 = LSB first).
 * @param[in] trigger When active, allows a new word transfer.
 * @param[in] s_in Serial data input.
 * @param[in] p_in Parallel data input.
 * @param[out] s_out Serial data output.
 * @param[out] p_out Parallel data output.
 * @param[out] active Indicates when the module is actively transmitting.
 * @param[out] ready_next The next data word can be set.
 * @param[out] done_curr The last word was completed.
 * @note 2025-01-29 Tested bit-widths:
 *       Full:   1-12
 */
module ShiftRegister#(parameter WIDTH = 32, parameter DEPTH = 8)
                     (input clk, input rst, input [DEPTH - 1:0] cdiv,
                     input cpha, input dir, input trigger,
                     input s_in, input [WIDTH - 1:0] p_in,
                     output s_out, output [WIDTH - 1:0] p_out, output active,
                     output ready_next, output done_curr);
   
   // ----- Constants -----
   localparam WIDTH_LESS_SAFE = (WIDTH == 1) ? 0 : WIDTH - 2;       /**< Safe (WIDTH - 2) value.*/
   localparam ONE_SAFE        = (WIDTH == 1) ? 0 : 1;               /**< Safe (1) value.*/
   localparam [DEPTH - 1:0] DEPTH_ZERO = {DEPTH{1'b0}};             /**< Zero for depth values.*/
   localparam [DEPTH - 1:0] DEPTH_ONE  = {{DEPTH - 1{1'b0}}, 1'b1}; /**< One for depth values.*/

   // ----- Local Variables -----
   integer i;               /**< Loop count integer.*/
   wire    active_internal; /**< Internal indication of activity.*/
   wire    active_new;      /**< New activity for a clock cycle.*/
   wire    clk_ready;       /**< Slow clock region update ready.*/
   reg     active_last;     /**< Final active signal for done detection.*/
   wire    start_early;     /**< Allows back-to-back shifting.*/
   reg     done_int;        /**< Internal done signal.*/
   wire [DEPTH - 1:0] cdiv_half;      /**< Half of cdiv, fixed for a depth of 1.*/
   reg  [DEPTH - 1:0] clk_slow;       /**< Slow clock counter.*/
   reg  [WIDTH - 1:0] active_shift;   /**< State of transfer at each bit.*/
   reg  [WIDTH - 1:0] serial_shift;   /**< Shift data register.*/
   reg  [WIDTH - 1:0] parallel_shift; /**< Parallel data register.*/
   reg  [WIDTH - 1:0] parallel_buf;   /**< Buffered parallel data register.*/
   
   // ----- Combinatorial Logic -----
   
   // Determine half of cdiv
   generate
      if (DEPTH == 1) begin
         assign cdiv_half = DEPTH_ZERO;
      end else begin
         assign cdiv_half = {1'b0, cdiv[DEPTH - 1:1]};
      end
   endgenerate
   
   // Determine when back-to-back triggers are run
   generate
      if (WIDTH == 1) begin
         assign start_early = active_shift[WIDTH - 1] & trigger;
      end else begin
         assign start_early = ~active_shift[WIDTH_LESS_SAFE] & active_shift[WIDTH - 1] & trigger;
      end
   endgenerate
   
   // Activity is determined by the final value in the state registers.
   assign active_new       = (trigger & ~active_internal) | start_early;
   assign active_internal  = active_shift[WIDTH - 1];
   generate
      if (WIDTH == 1) begin
         assign ready_next = 1'b1; // Always active at width 1!
      end else begin
         assign ready_next = ~active_internal | (~active_shift[WIDTH_LESS_SAFE] & active_internal);
      end
   endgenerate
   assign done_curr        = done_int;
   
   // Assign outputs
   assign s_out  = active_internal & serial_shift[WIDTH - 1];
   assign p_out  = done_int ? parallel_shift : parallel_buf;
   assign active = active_internal;
   
   // Determine when the slow clock is ready or overridden by a new trigger
   assign clk_ready = (clk_slow >= cdiv) | (trigger & ~active_internal);

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the shift register.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : count_registers
      
      // Initialize to default value on reset
      if (rst) begin
         clk_slow       <= DEPTH_ZERO;
         active_shift   <= {WIDTH{1'b0}};
         serial_shift   <= {WIDTH{1'b0}};
         parallel_shift <= {WIDTH{1'b0}};
         parallel_buf   <= {WIDTH{1'b0}};
         active_last    <= 1'b0;
         done_int       <= 1'b0;
      
      // Otherwise update the register state
      end else begin
      
         // Keep values
         active_shift   <= active_shift;
         serial_shift   <= serial_shift;
         parallel_shift <= parallel_shift;
         active_last    <= active_last;
         done_int       <= 1'b0;
         
         // Determine when to latch the output data.
         parallel_buf <= (done_int | (clk_ready & ready_next & active_internal)) ?
                         parallel_shift : parallel_buf;
      
         // Always update the slow clock counter
         clk_slow <= (clk_ready) ? DEPTH_ZERO : (clk_slow + DEPTH_ONE);
         
         // Update serial data on slow clock edges
         if (clk_ready) begin
         
            // Initial active shift has no previous state.
            active_shift[0] <= active_new;
            
            // Last active value
            active_last <= active_internal;
            if (WIDTH == 1) begin
               done_int <= active_internal; // Always immediately done when active in width 1!
            end else begin
               done_int <= active_internal & ~active_shift[WIDTH_LESS_SAFE];
            end
            
            // Initial serial shift takes in serial input rather than
            // the previous value.
            serial_shift[0] <= (~active_internal | start_early) ?
                               ((dir) ? p_in[WIDTH - 1] : p_in[0]) : s_in;
            
            // Update all values beyond the initial bit.
            for (i = 1; i < WIDTH; i = i + 1) begin
               active_shift[i] <= active_new | active_shift[i - 1];
               serial_shift[i] <= (~active_internal | start_early) ? ((dir) ? p_in[(WIDTH - 1) - i] : p_in[i]) :
                                  serial_shift[i - 1];
            end
         end
         
         // Capture parallel data on the appropriate slow clock edge
         if (((clk_slow == cdiv_half) & ~cpha) |
             ((clk_slow >= cdiv) & cpha)) begin
         
            // Only latch the parallel data on the end of activity.
            // Note that latching the data in this way means that the final
            // serial data update needs to be duplicated here.
            if ((active_internal | active_new) & (WIDTH == 1)) begin
                // For width of 1, the data is always the newest shift in data.
               parallel_shift[0] <= s_in;
            end else if (active_internal | active_new) begin
            
               // For widths greater than one, handle direction for the endpoint values.
               parallel_shift[0]         <= (dir) ? parallel_shift[ONE_SAFE] : s_in;
               if (WIDTH != 1) begin // Do not do this for width 1!
                  parallel_shift[WIDTH - 1] <= (dir) ? s_in : parallel_shift[WIDTH_LESS_SAFE];
               end
               
               // Update all values beyond the initial bit.
               for (i = 1; i < WIDTH - 1; i = i + 1) begin
                  parallel_shift[i] <= (dir) ? parallel_shift[i + 1] : parallel_shift[i - 1];
               end
            
            // Otherwise, keep the last parallel shift data value latched.
            end else begin
               parallel_shift <= parallel_shift;
            end
         end
      end
   end

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * N-bit linear feedback shift register, with both Fibonacci and
 * Galois modes.
 * @tparam[in] WIDTH The width of data stored in the shift register.
 * @tparam[in] GALOIS If non-zero, uses Galois mode. Otherwise, uses
 * Fibonacci mode.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module reloads the
 * default value.
 * @param[in] en Enables the shift register to step to the next value.
 * @param[in] taps Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] def The default value, used on reset.
 * @param[out] data The most recent output data.
 * @note 2024-06-25 Tested bit-widths:
 *       Full:   1-13 (including manual validation of m-sequence count)
 *       This module takes very significant time to test for N >= 12 due to
 *       the worst case of running every bit value for every tap value,
 *       leading to O(n^2) runtime.
 */
module LinearFeedbackShiftRegister#(parameter WIDTH = 32, parameter GALOIS = 0)
                     (input clk, input rst, input en,
                     input [WIDTH - 1:0] taps, input [WIDTH - 1:0] def,
                     output [WIDTH - 1:0] data);
   
   // ----- Constants -----
   localparam WIDTH_LESS_SAFE = (WIDTH == 1) ? 0 : WIDTH - 2; /**< Safe (WIDTH - 2) value.*/

   // ----- Local Variables -----
   integer i; /**< Loop count integer.*/
   reg  [WIDTH - 1:0] data_internal; /**< Shift data register.*/
   
   // ----- Combinatorial Logic -----
   
   // Assign outputs
   assign data = data_internal;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the LFSR.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : count_registers
      
      // Initialize to default value on reset
      if (rst) begin
         data_internal <= def;
      
      // Otherwise update the register state
      end else if (en) begin
         
         // Galois mode for register updates
         if (GALOIS > 0) begin
         
            // Wrap the final bit around to the first bit.
            // Galois mode: Treat as any other bit, less the final bit mask.
            data_internal[0] <= data_internal[WIDTH - 1];
            
            // Update all values beyond the initial bit.
            // Galois mode: each bit can be modified by the taps.
            for (i = 1; i < WIDTH; i = i + 1) begin
               data_internal[i] <= data_internal[i - 1] ^ (taps[WIDTH - 1 - i] & data_internal[WIDTH - 1]);
            end
            
         // Fibonacci mode for register updates
         end else begin
         
            // Wrap the final bit around to the first bit.
            // Fibonacci mode: XOR all tapped bits together for this.
            data_internal[0] <= ^(data_internal & {1'b1, taps[WIDTH_LESS_SAFE:0]});
            
            // Update all values beyond the initial bit.
            // Fibonacci mode: simple shift register.
            for (i = 1; i < WIDTH; i = i + 1) begin
               data_internal[i] <= data_internal[i - 1];
            end
         end
      
      // Not enabled, hold value
      end else begin
         data_internal <= data_internal;
      end
   end

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/**
 * N-bit alternating step generator based on three linear feedback
 * shift registers.
 * @tparam[in] WIDTH The width of data stored in the generator.
 * @tparam[in] GALOIS If non-zero, uses Galois mode. Otherwise, uses
 * Fibonacci mode.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module reloads the
 * default value.
 * @param[in] en Enables the shift register to step to the next value.
 * @param[in] taps0 Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] taps1 Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] taps2 Which taps to enable for the shift register. Note
 * that the MSB is always treated as a 1, regardless of actual value.
 * @param[in] def0 The default value, used on reset.
 * @param[in] def1 The default value, used on reset.
 * @param[in] def2 The default value, used on reset.
 * @param[out] data The most recent output data.
 * @note 2024-06-26 Tested bit-widths:
 *       Full:   3-16
 * Note that bit widths 1-2 were tested and failed. This is expected,
 * as those bit widths have no valid m-sequences to allow for useful
 * ASG sequence generation. "Successfully failed," one might say.
 */
module AlternatingStepGenerator#(parameter WIDTH = 32, parameter GALOIS = 0)
                     (input clk, input rst, input en,
                     input [WIDTH - 1:0] taps0, input [WIDTH - 1:0] taps1, input [WIDTH - 1:0] taps2,
                     input [WIDTH - 1:0] def0, input [WIDTH - 1:0] def1, input [WIDTH - 1:0] def2,
                     output [WIDTH - 1:0] data);
   
   // ----- Constants -----
   localparam WIDTH_LESS_SAFE = (WIDTH == 1) ? 0 : WIDTH - 2; /**< Safe (WIDTH - 2) value.*/

   // ----- Local Variables -----
   integer i;   /**< Loop count integer.*/
   wire    en0; /**< Enable for the first LFSR.*/
   wire    en1; /**< Enable for the second LFSR.*/
   wire [WIDTH - 1:0] data0;         /**< Shift data register for first LFSR.*/
   wire [WIDTH - 1:0] data1;         /**< Shift data register for second LFSR.*/
   wire [WIDTH - 1:0] data2;         /**< Shift data register for third LFSR.*/
   reg  [WIDTH - 1:0] data_internal; /**< Combined data output.*/
   
   // ----- Combinatorial Logic -----
   
   // Assign enables from via the controller LFSR's LSB.
   assign en0 = data2[0];
   assign en1 = ~data2[0];
   
   // Assign outputs.
   assign data = data_internal;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the alternating step generator.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : count_registers
      
      // Initialize to default value on reset
      if (rst) begin
         data_internal <= def2;
      
      // Otherwise update the register state
      end else if (en) begin
         
         // Combine peripheral LFSR LSBs for new LSB.
         data_internal[0] <= data0[0] ^ data1[0];
         
         // Shift all other bits up.
         for (i = 1; i < WIDTH; i = i + 1) begin
            data_internal[i] <= data_internal[i - 1];
         end
      
      // Not enabled, hold value
      end else begin
         data_internal <= data_internal;
      end
   end

   // ----- Sub-Modules -----
   
   /** First linear feedback shift register.*/
   LinearFeedbackShiftRegister #(.WIDTH(WIDTH), .GALOIS(GALOIS))
                     lsfr0 (.clk(clk), .rst(rst), .en(en0),
                     .taps(taps0), .def(def0), .data(data0));
   /** Second linear feedback shift register.*/
   LinearFeedbackShiftRegister #(.WIDTH(WIDTH), .GALOIS(GALOIS))
                     lsfr1 (.clk(clk), .rst(rst), .en(en1),
                     .taps(taps1), .def(def1), .data(data1));
   /** Third linear feedback shift register.*/
   LinearFeedbackShiftRegister #(.WIDTH(WIDTH), .GALOIS(GALOIS))
                     lsfr2 (.clk(clk), .rst(rst), .en(en),
                     .taps(taps2), .def(def2), .data(data2));

// ----- End of Module -----
endmodule

/**
 * N-bit controller-side serial peripheral interface (SPI) peripheral.
 * This is the third iteration of this module, since apparently
 * doing proper clock division was something I never wanted to do
 * correctly. The Shift Register and UART modules follow suit.
 *   - Version 1 - A recreation of the H. Neeman's Digital version
 *     using a naive clock divider that was not synthesizable. Did
 *     not support true CPHA.
 *   - Version 2 - An update to use a synthesizable clock divider,
 *     but still clunky and did not support true CPHA.
 *   - Version 3 - A rewrite from the Shift Register level, performing
 *     the clock division at the Shift Register level and therefore
 *     properly supporting things like CPHA and not requiring any
 *     weird clock stretching or "one pulse out of 100" type
 *     behaviors for clock division.
 * @tparam[in] WIDTH The width of data stored in the peripheral.
 * @tparam[in] DEPTH The depth of clock division allowed.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module aborts
 * in-progress data transfers.
 * @param[in] dir The SPI transmission direction (0 = MSB first,
 * 1 = LSB first).
 * @param[in] trigger When active, allows a new word transfer.
 * @param[in] cpol SPI clock polarity.
 * @param[in] cpha SPI clock phase for shifting in data.
 * @param[in] cdiv The main clock to SPI clock division choice.
 * @param[in] cipo Controller input, peripheral output serial data.
 * @param[in] data_co Controller output parallel data.
 * @param[out] copi Controller output, peripheral input serial data.
 * @param[out] data_po Peripheral output parallel data.
 * @param[out] sck The SPI data clock.
 * @param[out] n_cs Indicates when the module is actively transmitting.
 * @param[out] ready_next The next data word can be set.
 * @param[out] done_curr The last word was completed.
 * @note 2025-01-29 Tested bit-widths:
 *       Full:   1-12
 */
module ControllerSPI#(parameter WIDTH = 8, parameter DEPTH = 3)
                      (input clk, input rst, input dir, input trigger, input cpol, input cpha,
                      input [DEPTH - 1:0] cdiv, input cipo, input [WIDTH - 1:0] data_co,
                      output copi, output [WIDTH - 1:0] data_po, output sck, output n_cs,
                      output ready_next, output done_curr);
   
   // ----- Constants -----
   localparam [DEPTH - 1:0] DEPTH_ZERO = {DEPTH{1'b0}};             /**< Zero for depth values.*/
   localparam [DEPTH - 1:0] DEPTH_ONE  = {{DEPTH - 1{1'b0}}, 1'b1}; /**< One for depth values.*/

   // ----- Local Variables -----
   wire active;      /**< Active indicator from the shift register.*/
   wire clk_ready;   /**< Slow clock is ready to reset.*/
   reg  sck_int;     /**< Internal serial clock signal.*/
   wire [DEPTH - 1:0] cdiv_half; /**< Half of cdiv, fixed for a depth of 1.*/
   reg  [DEPTH - 1:0] clk_slow;  /**< Slow clock.*/
   
   // ----- Combinatorial Logic -----
   
   // Determine half of cdiv
   generate
      if (DEPTH == 1) begin
         assign cdiv_half = DEPTH_ZERO;
      end else begin
         assign cdiv_half = {1'b0, cdiv[DEPTH - 1:1]};
      end
   endgenerate
   
   // Set up serial clock and hardware chip select
   assign n_cs = ~active;
   assign sck  = (cdiv == DEPTH_ZERO) ? ((active & ~clk) ^ cpol) : sck_int;
   
   // Determine when the slow clock is ready or overridden by a new trigger
   assign clk_ready = (clk_slow >= cdiv) | (trigger & ~active);

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the controller-side SPI.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : spi_registers
      
      // Initialize to default value on reset
      if (rst) begin
         clk_slow    <= DEPTH_ZERO;
         sck_int     <= 1'b0;
      
      // Otherwise update the register state
      end else begin
      
         // By default, keep last latch value states
         sck_int  <= sck_int;
         
         // Handle the slow clock counter and any actions based on the
         // roll-over of the slow clock counter.
         clk_slow <= (clk_ready) ? DEPTH_ZERO : (clk_slow + DEPTH_ONE);
         if (~active) begin
            sck_int  <= cpol;
         end else if (clk_ready) begin
            sck_int  <= active ^ ~cpol;
         end else if (clk_slow == cdiv_half) begin
            sck_int  <= ~cpol;
         end
      end
   end

   // ----- Sub-Modules -----
   
   /** Underlying shift register for performing serial/parallel conversions.*/
   ShiftRegister #(.WIDTH(WIDTH), .DEPTH(DEPTH))
                sr_mod(.clk(clk), .rst(rst),
                .cdiv(cdiv), .cpha(cpha), .dir(dir), .trigger(trigger),
                .s_in(cipo), .p_in(data_co),
                .s_out(copi), .p_out(data_po), .active(active),
                .ready_next(ready_next), .done_curr(done_curr));

// ----- End of Module -----
endmodule

/**
 * N-bit universal asynchronous receiver/transmitter (UART).
 * Includes hardware flow control (RTS/CTS), choice of stop
 * bits (1 or 2), and choice of parity (none, even, or odd).
 * Following with standard UART configuration, data is always
 * transmitted/received LSB first.
 *
 * A single UART frame is, in sequence, Idle (high), Start (low),
 * Data (LSB first), Parity (0-1 bits), Stop (1-2 bits high), Idle (high).
 *
 * Because a fixed-width shift register design is used, a fixed
 * data width is used. As such, either 2 stop bits or 1 stop bit and
 * 1 parity bit are used. If 1 stop bit and no parity bit is selected,
 * an idle bit is stuffed into the word, essentially acting as 2 stop
 * bits.
 *
 * If flow control is enabled, transmits will be queued until CTS
 * is high, and RTS will be asserted any time the module is enabled
 * (as this module has no configuration for half-duplex operation).
 *
 * @tparam[in] WIDTH The (base) width of data stored in the peripheral.
 * @tparam[in] DEPTH The depth of clock division allowed.
 * @param[in] clk The system clock.
 * @param[in] rst The system reset. Resetting the module aborts
 * in-progress data transfers.
 * @param[in] trigger When active, allows a new word transmit.
 * @param[in] flow When active, enables hardware flow control.
 * @param[in] stop_parity Stop bit and parity type (00 = 1 stop no parity,
 * 01 = 2 stop no parity, 10 = 1 stop even parity, 11 = 1 stop odd parity).
 * @param[in] cdiv The main clock to UART clock division choice.
 * @param[in] rxd Device receive (endpoint transmit).
 * @param[in] cts Device clear to send (endpoint ready to receive).
 * @param[in] data_tx Device output parallel data.
 * @param[out] txd Device transmit (endpoint receive).
 * @param[out] rts Device ready to receive (endpoint clear to send).
 * @param[out] data_rx Device input parallel data.
 * @param[out] data_new New data word received, parity passed.
 * @param[out] err_parity New data word received, parity failed.
 * @param[out] data_done Current data word transmitted.
 * @note 2025-01-30 Tested bit-widths:
 *       Full:   1-12
 */
module UART#(parameter WIDTH = 8, parameter DEPTH = 3)
            (input clk, input rst, input trigger, output err_parity,
             input [DEPTH - 1:0] cdiv, input flow, input [1:0] stop_parity,
             input rxd, input cts, output [WIDTH - 1:0] data_rx, output new_rx,
             output txd, output rts, input [WIDTH - 1:0] data_tx, output ready_tx, output done_tx);
   
   // ----- Constants -----
   localparam [DEPTH - 1:0] DEPTH_ZERO = {DEPTH{1'b0}};             /**< Zero for depth values.*/
   localparam [DEPTH - 1:0] DEPTH_ONE  = {{DEPTH - 1{1'b0}}, 1'b1}; /**< One for depth values.*/
   localparam WIDTH_ACT = WIDTH + 3; /**< Three extra bits are start, parity, stop.*/

   // ----- Local Variables -----
   wire clk_ready;      /**< Slow clock is ready to reset.*/
   wire txd_internal;   /**< Internal Tx signal without idle value handling.*/
   wire active_tx;      /**< Active indicator from the Tx shift register.*/
   wire active_rx;      /**< Receive is active.*/
   reg  rxd_buf;        /**< Buffered Rx input for detecting start of frame.*/
   reg  rxd_buf2;       /**< Second buffered Rx input for detecting start of frame.*/
   wire trigger_tx;     /**< Trigger for new transmit frame.*/
   wire trigger_rx;     /**< Trigger for new receive frame.*/
   wire trigger_rx_int; /**< Internal trigger for new receive frame.*/
   reg  trigger_rx_buf; /**< Buffered trigger for new receive frame.*/
   wire parity_tx;      /**< Transmit word parity bit.*/
   wire parity_rx;      /**< Receive word parity bit.*/
   wire parity_ck;      /**< Parity check against receive frame's parity bit.*/
   wire new_rx_int;     /**< Slow clock new data indicator, to split out errors.*/
   reg  [DEPTH - 1:0]     clk_slow;       /**< Slow clock.*/
   wire [WIDTH_ACT - 1:0] data_tx_actual; /**< Fully defined transmit data frame.*/
   wire [WIDTH_ACT - 2:0] data_rx_actual; /**< Fully defined receive data frame.*/
   
   // ----- Combinatorial Logic -----
   
   // Determine when the slow clock is ready or overridden by a new trigger
   assign clk_ready = (clk_slow >= cdiv) | trigger_rx_int;
   
   // Determine transmit value with idle value
   assign txd = txd_internal | ~active_tx;
   
   // Determine hardware flow control
   assign rts = flow & ~rst;
   
   // Determine Tx and Rx triggers
   assign trigger_tx     = trigger & (cts | ~flow);
   assign trigger_rx_int = rxd_buf2 & ~rxd_buf & ~active_rx & ~trigger_rx_buf;
   assign trigger_rx     = (trigger_rx_buf & clk_slow >= cdiv) |
                           (trigger_rx_int & cdiv == DEPTH_ZERO);
   
   // Calculate parity
   assign parity_tx = (^data_tx) ^ stop_parity[0];
   assign parity_rx = (^data_rx) ^ stop_parity[0];
   assign parity_ck = ~(parity_rx ^ data_rx_actual[WIDTH]) | ~stop_parity[1];
   
   // Determine parity check results
   assign new_rx     = new_rx_int & parity_ck;
   assign err_parity = new_rx_int & ~parity_ck;
   
   // Pack the transmit data based on the parity settings.
   assign data_tx_actual = stop_parity[1] ? {1'b1, parity_tx, data_tx, 1'b0} :
                           {2'b11, data_tx, 1'b0};
   
   // Unpack the receive data.
   assign data_rx = data_rx_actual[WIDTH - 1:0];

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the UART.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : uart_registers
      
      // Initialize to default value on reset
      if (rst) begin
         rxd_buf        <= 1'b1;
         rxd_buf2       <= 1'b1;
         trigger_rx_buf <= 1'b0;
         clk_slow       <= DEPTH_ZERO;
      
      // Otherwise update the register state
      end else begin
      
         // Static buffer updates
         rxd_buf  <= rxd;
         rxd_buf2 <= rxd_buf;
         
         // Handle updates to the Rx trigger buffer
         trigger_rx_buf <= (trigger_rx_int) ? 1'b1 :
                           (trigger_rx) ? 1'b0 : trigger_rx_buf;
         
         // Handle the slow clock counter.
         // Because this is only used to buffer the receive trigger,
         // and it uses buffered Rx line data, restart it at one.
         clk_slow <= (clk_ready) ? DEPTH_ONE : (clk_slow + DEPTH_ONE);
      end
   end

   // ----- Sub-Modules -----
   
   /** Underlying transmit shift register for performing serial/parallel conversions.*/
   ShiftRegister #(.WIDTH(WIDTH_ACT), .DEPTH(DEPTH))
             sr_tx(.clk(clk), .rst(rst),
                   .cdiv(cdiv), .cpha(1'b0), .dir(1'b1), .trigger(trigger_tx),
                   .s_in(1'b0), .p_in(data_tx_actual),
                   .s_out(txd_internal), .p_out(), .active(active_tx),
                   .ready_next(ready_tx), .done_curr(done_tx));
   /** Underlying receive shift register for performing serial/parallel conversions.*/
   ShiftRegister #(.WIDTH(WIDTH_ACT - 1), .DEPTH(DEPTH)) // Less one because the start bit detection eats the first bit
             sr_rx(.clk(clk), .rst(rst),
                   .cdiv(cdiv), .cpha(1'b0), .dir(1'b1), .trigger(trigger_rx),
                   .s_in(rxd_buf), .p_in({WIDTH_ACT - 1{1'b0}}),
                   .s_out(), .p_out(data_rx_actual), .active(active_rx),
                   .ready_next(), .done_curr(new_rx_int));

// ----- End of Module -----
endmodule

/* Helper Modules ********************************************************** */

/* End of File ************************************************************* */
