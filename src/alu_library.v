/* ************************************************************************* *
 * @file      alu_library.v
 * @author    nitelich
 * @since     2024-05-02
 * @copyright Copyright &copy; 2025 nitelich, all rights reserved
 * @brief
 *
 * A collection of generic bit width arithmetic and logic modules, using
 * only "low level" primitive functions such as bitwise logic, multiplexers
 * (as inferred from ternary expressions), and other simple combinatorial
 * logic.
 *
 * @details
 *
 * Contains implementations of the following high-level modules:
 *   - Unsigned/Signed Increment
 *   - Unsigned/Signed Adder/Subtractor
 *   - Unsigned/Signed Comparator
 *   - Reduced arithmetic/logic barrel shift/rotate
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
 *   v0.1.2 - Fixing some Altera warnings
 *            Started 2024-08-23, Completed 2024-08-23
 *   v0.1.1 - Simplifying some bit-by-bit calculations to slice calculations.
 *            Started 2024-05-16, Completed 2024-05-16
 *   v0.1.0 - Initial feature complete version.
 *            Started 2024-05-02, Completed 2024-05-12
 */
`define VERSION_ALU "0.1.2"

// Include the shared definitions file.
`include "shared_definitions.v"

/* Helper Macros *********************************************************** */

/* Primary Modules ********************************************************* */

/**
 * N-bit Kogge-Stone signed/unsigned adder/subtractor.
 * Performs N-bit integer addition or subtraction with either
 * signed or unsigned numbers.
 * @note Bit widths of (N < 2) are not supported. For that small
 *       of adders, sign has no meaning, and so the overflow
 *       logic breaks down.
 * @tparam[in] WIDTH The bit width N.
 * @param[in] a The first N-bit integer to add/subtract.
 * @param[in] b The second N-bit integer to add/subtract.
 * @param[in] sub Which operation to use (add = 0, sub = 1).
 * @param[in] sign The signedness to use (unsigned = 0, signed = 1).
 * @param[out] res The N-bit integer sum or difference of a and b.
 * @param[out] ovf Indicates overflow (good = 0, overflow = 1).
 * @note 2024-05-16 Tested bit-widths:
 *       Full:   2-8
 *       Random: 16, 24, 32, and 64; 2 ** 18 random values covered
 */
module AddSubKS#(parameter WIDTH = 32)
                (input [WIDTH - 1:0] a, input [WIDTH - 1:0] b,
                 input sub, input sign,
                 output [WIDTH - 1:0] res, output ovf);

   // ----- Constants -----
   localparam LOGWIDTH = $clog2(WIDTH); /**< Base-2 log of bit width, for generic hookup.*/

   // ----- Local Variables -----
   genvar gen_cnt;                   /**< Generation counter for generation loops.*/
   wire [WIDTH - 1:0] b_inv;         /**< Inverted second input for subtraction.*/
   wire [WIDTH - 1:0] g[0:LOGWIDTH]; /**< Stage-by-stage generate values.*/
   wire [WIDTH - 1:0] p[0:LOGWIDTH]; /**< Stage-by-stage propagate values.*/

   // ----- Combinatorial Logic -----

   // Inversion is performed as part of an optimized complement
   // when performing a subtraction.
   assign b_inv = (sub) ? (~b) : (b);

   // First stage full adder. The full adder is used as part of an
   // optimized complement when performing a subtraction. Otherwise,
   // a half adder could be used here.
   assign p[0][0] = a[0] ^ b_inv[0] ^ sub;
   assign g[0][0] = (a[0] & b_inv[0]) | (a[0] & sub) | (b_inv[0] & sub);
   
   // First stage half adders for all bits beyond the first.
   // These half adders initialize the propagate-generate logic.
   assign p[0][WIDTH - 1:1] = a[WIDTH - 1:1] ^ b_inv[WIDTH - 1:1];
   assign g[0][WIDTH - 1:1] = a[WIDTH - 1:1] & b_inv[WIDTH - 1:1];

   // Generate addition via propagate-generate logic
   generate

      // Iterate over all generations after the first...
      for (gen_cnt = 1; gen_cnt <= LOGWIDTH; gen_cnt = gen_cnt + 1) begin : pg_generations

         // Calculate over all low bits that are done calculating in this generation...
         // Because the propagate-generate logic is done, pass these values forward.
         assign p[gen_cnt][(2 ** (gen_cnt - 1)) - 1:0] = p[gen_cnt - 1][(2 ** (gen_cnt - 1)) - 1:0];
         assign g[gen_cnt][(2 ** (gen_cnt - 1)) - 1:0] = g[gen_cnt - 1][(2 ** (gen_cnt - 1)) - 1:0];
         
         // Calculate over all high bits to further the propagate-generate logic.
         // This uses an unrolled PropagateGenerate submodule, so see that submodule
         // design for details.
         // @note The final stage propagate values are not used. These could be
         //       pre-optimized prior to synthesis, if desired.
         assign p[gen_cnt][WIDTH - 1:2 ** (gen_cnt - 1)] = p[gen_cnt - 1][WIDTH - 1:2 ** (gen_cnt - 1)] &
            p[gen_cnt - 1][WIDTH - (2 ** (gen_cnt - 1)) - 1:0];
         assign g[gen_cnt][WIDTH - 1:2 ** (gen_cnt - 1)] = g[gen_cnt - 1][WIDTH - 1:2 ** (gen_cnt - 1)] |
            (p[gen_cnt - 1][WIDTH - 1:2 ** (gen_cnt - 1)] & g[gen_cnt - 1][WIDTH - (2 ** (gen_cnt - 1)) - 1:0]);
      end
   endgenerate
   
   // The final calculation is:
   //   s(i) = p(i, 0) XOR g(i - 1, log2(N))
   // For bit zero, the value of g() is known to be 0.
   assign res[0] = p[0][0];
   assign res[WIDTH - 1:1] = p[0][WIDTH - 1:1] ^ g[LOGWIDTH][WIDTH - 2:0];

   // Determine the final overflow logic. First, for unsigned logic:
   //   Overflow if final generate (carry), inverted meaning when subtracting
   //   ovf = g(N - 1, log2(N)) XOR sub
   // Second, for signed logic:
   //   Overflow if the final two carry output values do not match.
   //   This overflow condition is derived from making a table of
   //   overflow situations, rather than from basic intuition.
   //   ovf = g(N - 1, log2(N)) XOR g(N - 2, log2(N))
   assign ovf = (sign) ? (g[LOGWIDTH][WIDTH - 1] ^ g[LOGWIDTH][WIDTH - 2]) :
                         (g[LOGWIDTH][WIDTH - 1] ^ sub);

   // ----- Register Logic -----

   // ----- Sub-Modules -----

   // Generate block to cause an error on invalid widths.
   generate
      if (WIDTH < 2) begin : error_width
         IllegalParameterWidthTooSmall non_existing_module();
      end
   endgenerate

// ----- End of Module -----
endmodule

/**
 * N-bit Kogge-Stone signed/unsigned incrementer.
 * Performs N-bit integer increment with either signed or unsigned
 * numbers.
 * @note Bit widths of (N < 2) are not supported. For that small
 *       of adders, sign has no meaning, and so the overflow
 *       logic breaks down.
 * @tparam[in] WIDTH The bit width N.
 * @param[in] a The N-bit integer to increment.
 * @param[in] sign The signedness to use (unsigned = 0, signed = 1).
 * @param[out] res The N-bit integer result.
 * @param[out] ovf Indicates overflow (good = 0, overflow = 1).
 * @note 2024-05-16 Tested bit-widths:
 *       Full:   2-8, 16
 *       Random: 24, 32, and 64; 2 ** 18 random values covered
 */
module IncrementKS#(parameter WIDTH = 32)
                   (input [WIDTH - 1:0] a, input sign,
                    output [WIDTH - 1:0] res, output ovf);

   // ----- Constants -----
   localparam LOGWIDTH = $clog2(WIDTH); /**< Base-2 log of bit width, for generic hookup.*/

   // ----- Local Variables -----
   genvar gen_cnt;                   /**< Generation counter for generation loops.*/
   wire [WIDTH - 1:0] g[0:LOGWIDTH]; /**< Stage-by-stage generate values.*/
   wire [WIDTH - 1:0] p[0:LOGWIDTH]; /**< Stage-by-stage propagate values.*/

   // ----- Combinatorial Logic -----

   // First stage half adder, performing the main addition of an increment.
   // Because this is the only non-zero "second input" to the addition,
   // this half adder reduces to (s[i]=~a[i], c[i]=a[i]).
   assign p[0][0] = ~a[0];
   assign g[0][0] = a[0];
   
   // Because all bits other than the first are zero for the "second input" to
   // the addition, all half adders of a normal adder reduce to (s[i]=0, c[i]=a[i]).
   assign p[0][WIDTH - 1:1] = a[WIDTH - 1:1];
   assign g[0][WIDTH - 1:1] = 1'b0;

   // Generate all additional stage(s) propagate-generate logic
   generate

      // Iterate over all generations after the first...
      for (gen_cnt = 1; gen_cnt <= LOGWIDTH; gen_cnt = gen_cnt + 1) begin : pg_generations_assign

         // Calculate over all low bits that are done calculating in this generation...
         // Because the propagate-generate logic is done, pass these values forward.
         assign p[gen_cnt][(2 ** (gen_cnt - 1)) - 1:0] = p[gen_cnt - 1][(2 ** (gen_cnt - 1)) - 1:0];
         assign g[gen_cnt][(2 ** (gen_cnt - 1)) - 1:0] = g[gen_cnt - 1][(2 ** (gen_cnt - 1)) - 1:0];
         
         // Calculate over all middle bits to further the propagate-generate logic.
         // This uses an unrolled PropagateGenerate submodule, so see that submodule
         // design for details.
         // Because we know that higher generate bits are defined as zero, they
         // are removed from the calculation here to simplify it.
         // Note for bit widths that are not exact powers of two, the bits up to
         // that next power of two are cut off here as part of generation.
         if (WIDTH > 2 ** gen_cnt) begin
            assign p[gen_cnt][(2 ** gen_cnt) - 1:2 ** (gen_cnt - 1)] =
               p[gen_cnt - 1][(2 ** gen_cnt) - 1:2 ** (gen_cnt - 1)] &
               p[gen_cnt - 1][(2 ** (gen_cnt - 1)) - 1:0];
            assign g[gen_cnt][(2 ** gen_cnt) - 1:2 ** (gen_cnt - 1)] =
               p[gen_cnt - 1][(2 ** gen_cnt) - 1:2 ** (gen_cnt - 1)] &
               g[gen_cnt - 1][(2 ** (gen_cnt - 1)) - 1:0];
         end else begin
            assign p[gen_cnt][WIDTH - 1:2 ** (gen_cnt - 1)] =
               p[gen_cnt - 1][WIDTH - 1:2 ** (gen_cnt - 1)] &
               p[gen_cnt - 1][WIDTH - (2 ** (gen_cnt - 1)) - 1:0];
            assign g[gen_cnt][WIDTH - 1:2 ** (gen_cnt - 1)] =
               p[gen_cnt - 1][WIDTH - 1:2 ** (gen_cnt - 1)] &
               g[gen_cnt - 1][WIDTH - (2 ** (gen_cnt - 1)) - 1:0];
         end
         
         // Calculate over all high bits that have propagate-generate logic reducible to
         // a single AND gate for the propagate...
         if (WIDTH > 2 ** gen_cnt) begin
            assign p[gen_cnt][WIDTH - 1:2 ** gen_cnt] = p[gen_cnt - 1][WIDTH - 1:2 ** gen_cnt] &
               p[gen_cnt - 1][WIDTH - (2 ** (gen_cnt - 1)) - 1:2 ** (gen_cnt - 1)];
            assign g[gen_cnt][WIDTH - 1:2 ** gen_cnt] = g[gen_cnt - 1][WIDTH - 1:2 ** gen_cnt];
         end
      end
   endgenerate

   // The final calculation is:
   //   s(i) = p(i, 0) XOR g(i - 1, log2(N))
   // For bit zero, the value of g() is known to be 0.
   assign res[0] = p[0][0];
   assign res[WIDTH - 1:1] = p[0][WIDTH - 1:1] ^ g[LOGWIDTH][WIDTH - 2:0];

   // Determine the final overflow logic. First, for unsigned logic:
   //   Overflow if final generate (carry), inverted meaning when subtracting
   //   ovf = g(N - 1, log2(N)) XOR sub
   // Second, for signed logic:
   //   Overflow if the final two carry output values do not match.
   //   This overflow condition is derived from making a table of
   //   overflow situations, rather than from basic intuition.
   //   ovf = g(N - 1, log2(N)) XOR g(N - 2, log2(N))
   assign ovf = (sign) ? (g[LOGWIDTH][WIDTH - 1] ^ g[LOGWIDTH][WIDTH - 2]) :
                         (g[LOGWIDTH][WIDTH - 1]);

   // ----- Register Logic -----

   // ----- Sub-Modules -----

   // Generate block to cause an error on invalid widths.
   generate
      if (WIDTH < 2) begin : error_width
         IllegalParameterWidthTooSmall non_existing_module();
      end
   endgenerate

// ----- End of Module -----
endmodule

/**
 * N-bit Kogge-Stone signed/unsigned comparator.
 * Performs N-bit integer comparisons with either signed or
 * unsigned numbers.
 * @note Bit widths of (N < 2) are not supported. For that small
 *       of adders, sign has no meaning, and so the overflow
 *       logic breaks down.
 * @tparam[in] WIDTH The bit width N.
 * @param[in] a The first N-bit integer to compare.
 * @param[in] b The second N-bit integer to compare.
 * @param[in] sign The signedness to use (unsigned = 0, signed = 1).
 * @param[out] gt Indicates when (a > b).
 * @param[out] eq Indicates when (a == b).
 * @param[out] lt Indicates when (a < b).
 * @note 2024-05-16 Tested bit-widths:
 *       Full:   2-8
 *       Random: 16, 24, 32, and 64; 2 ** 18 random values covered
 */
module ComparatorKS#(parameter WIDTH = 32)
                    (input [WIDTH - 1:0] a, input [WIDTH - 1:0] b,
                     input sign, output gt, output lt, output eq);

   // ----- Constants -----
   localparam LOGWIDTH  = $clog2(WIDTH);   /**< Base-2 log of bit width, for generic hookup.*/
   localparam FULLWIDTH = (2 ** LOGWIDTH); /**< Full width after power of two extension.*/

   // ----- Local Variables -----
   genvar bit_cnt;                       /**< Bit counter for generation loops.*/
   genvar gen_cnt;                       /**< Generation counter for generation loops.*/
   wire [FULLWIDTH - 1:0] a_ext;         /**< Sign-extended first value.*/
   wire [FULLWIDTH - 1:0] b_ext;         /**< Sign-extended second value.*/
   wire [FULLWIDTH - 1:0] g[0:LOGWIDTH]; /**< Stage-by-stage generate values.*/
   wire [FULLWIDTH - 1:0] p[0:LOGWIDTH]; /**< Stage-by-stage propagate values.*/
   wire [FULLWIDTH - 1:0] e[0:LOGWIDTH - 1]; /**< Stage-by-stage equality values.*/

   // ----- Combinatorial Logic -----
   
   // First stage full adder. The full adder is used as part of an
   // optimized complement when performing a subtraction. Otherwise,
   // a half adder could be used here.
   assign p[0][0] = a_ext[0] ^ b_ext[0];
   assign g[0][0] = (a_ext[0] & (~b_ext[0])) | a_ext[0] | (~b_ext[0]);
   
   // Generate sign-extended inputs
   generate
   
      // Input is already a power of two, no need for extension.
      if (FULLWIDTH == WIDTH) begin
         assign a_ext = a;
         assign b_ext = b;
      
      // Input is not a power of two, extend to the nearest power of two.
      // Extension uses zeroes to not inflate the final result.
      end else begin
         assign a_ext[FULLWIDTH - 1:WIDTH] = {FULLWIDTH - WIDTH{sign ? a[WIDTH - 1] : 1'b0}};
         assign a_ext[WIDTH - 1:0] = a;
         assign b_ext[FULLWIDTH - 1:WIDTH] = {FULLWIDTH - WIDTH{sign ? b[WIDTH - 1] : 1'b0}};
         assign b_ext[WIDTH - 1:0] = b;
      end
   endgenerate
   
   // Calculate over all bits to generate the half adders
   // First stage half adders. Creates the initial state of generate
   // and propagate signals.
   assign p[0][FULLWIDTH - 1:1] = a_ext[FULLWIDTH - 1:1] ^ (~b_ext[FULLWIDTH - 1:1]);
   assign g[0][FULLWIDTH - 1:1] = a_ext[FULLWIDTH - 1:1] & (~b_ext[FULLWIDTH - 1:1]);

   // Generate all equality checks
   generate
   
      // Iterate over all initial bits and AND them together in stages...
      assign e[0][0] = ~p[0][0] & p[0][1];
      for (bit_cnt = 1; bit_cnt < 2 ** (LOGWIDTH - 1); bit_cnt = bit_cnt + 1) begin : pg_bits_low
         assign e[0][bit_cnt] = p[0][2 * bit_cnt] & p[0][(2 * bit_cnt) + 1];
      end

      // Iterate over all generations after the first...
      for (gen_cnt = 1; gen_cnt < LOGWIDTH; gen_cnt = gen_cnt + 1) begin : eq_checks

         // Iterate over all bits and AND them together in stages...
         for (bit_cnt = 0; bit_cnt < 2 ** (LOGWIDTH - gen_cnt - 1); bit_cnt = bit_cnt + 1) begin : pg_bits_low
            assign e[gen_cnt][bit_cnt] = e[gen_cnt - 1][2 * bit_cnt] & e[gen_cnt - 1][(2 * bit_cnt) + 1];
         end
      end

      // Iterate over all generations after the first...
      for (gen_cnt = 1; gen_cnt <= LOGWIDTH; gen_cnt = gen_cnt + 1) begin : pg_generations_submodule

         // Iterate over all high bits to further the propagate-generate logic.
         // Only the propagate-generate logic necessary to determine "overflow"
         // is necessary for comparison, so only that logic is present here.
         // @note The final stage propagate values are not used. These could be
         //       pre-optimized prior to synthesis, if desired.
         for (bit_cnt = (2 ** gen_cnt) - 1; bit_cnt < FULLWIDTH; bit_cnt = bit_cnt + (2 ** gen_cnt)) begin : pg_bits_high
            assign p[gen_cnt][bit_cnt] = p[gen_cnt - 1][bit_cnt] &
               p[gen_cnt - 1][bit_cnt - (2 ** (gen_cnt - 1))];
            assign g[gen_cnt][bit_cnt] = g[gen_cnt - 1][bit_cnt] |
               (p[gen_cnt - 1][bit_cnt] & g[gen_cnt - 1][bit_cnt - (2 ** (gen_cnt - 1))]);
         end
      end
   endgenerate

   // Determine the final comparison logic.
   // Equality is the easiest: Because the half/full adder sum outputs XOR the inputs,
   // one of which was inverted, check for all sum bits to be equal.
   assign eq = e[LOGWIDTH - 1][0];
   // Less than is next: Check for overflow from (A - B). If overflow, A < B.
   assign lt = (sign) ? (g[LOGWIDTH][FULLWIDTH - 1] ^ p[0][FULLWIDTH - 1]) :
                        (~g[LOGWIDTH][FULLWIDTH - 1]);
   // Greater than is last, but simple: gt = lt NOR eq
   assign gt = ~(lt | eq);

   // ----- Register Logic -----

   // ----- Sub-Modules -----

   // Generate block to cause an error on invalid widths.
   generate
      if (WIDTH < 2) begin : error_width
         IllegalParameterWidthTooSmall non_existing_module();
      end
   endgenerate

// ----- End of Module -----
endmodule

/**
 * N-bit shift-only barrel shifter.
 * Performs N-bit logical/arithmetic shifts.
 * @note This module should be preferred when the bit width is not a power of
 *       two. In those cases, correct rotate logic requires an (expensive)
 *       modulo operation.
 * @tparam[in] WIDTH The bit width N.
 * @param[in] a The N-bit value to shift.
 * @param[in] amount The N-bit amount to shift.
 * @param[in] dir The direction to shift (left = 0, right = 1).
 * @param[in] arith For right shifts, the mode (logic = 0, arithmetic = 1).
 * @param[out] res The shifted N-bit value.
 * @param[out] lost The bits lost to any shift, as though into a
 * consecutive N-bit value.
 * @note 2024-05-10 Tested bit-widths:
 *       Full:   1-8
 *       Random: 16, 24, 32, and 64; 2 ** 18 random values covered
 */
module BarrelShiftReduced#(parameter WIDTH = 32)
                          (input [WIDTH - 1:0] a, input [WIDTH - 1:0] amount,
                           input dir, input arith,
                           output [WIDTH - 1:0] res, output [WIDTH - 1:0] lost);

   // ----- Constants -----
   localparam LOGWIDTH = $clog2(WIDTH); /**< Base-2 log of bit width, for generic hookup.*/

   // ----- Local Variables -----
   genvar bit_cnt;                   /**< Bit counter for generation loops.*/
   genvar gen_cnt;                   /**< Generation counter for generation loops.*/
   wire   shift_gen0;                /**< Flag for shifting an extra time for right shifts.*/
   wire   high_bits;                 /**< Flag indicating whether the high bits are non-zero.*/
   wire   arith_bit;                 /**< Arithmetic extension bit.*/
   wire [WIDTH - 1:0] shift_gen;     /**< Flags for shifting each power of two generation.*/
   wire [WIDTH - 1:0] mask_gen;      /**< Flags for masking each power of two generation.*/
   wire [WIDTH - 1:0] mask_final;    /**< Final mask flag for each bit.*/
   wire [WIDTH - 1:0] shift_val[0:LOGWIDTH];    /**< Stage-by-stage generate values.*/
   wire [WIDTH - 1:0] mask_val[0:LOGWIDTH - 1]; /**< Stage-by-stage propagate values.*/
   wire [WIDTH - 1:0] amount_fix  /* synthesis keep */; /**< Direction-fixed shift amount, where necessary.*/
   wire [WIDTH - 1:0] WIDTH_SIZED /* synthesis keep */; /**< Sized width value.*/

   // ----- Combinatorial Logic -----
   
   // Determine what the arithmetic extension bit should be.
   // Arithmetic extension bit is the high bit, but only on arithmetic right shifts.
   assign arith_bit = a[WIDTH - 1] & arith & dir;

   // Generate shift and mask logic...
   generate
   
      // Determine whether the high bits of the shift amount are nonzero.
      if ((2 ** LOGWIDTH) == WIDTH) begin
         assign high_bits  = amount[WIDTH - 1:LOGWIDTH] != 0;
      end else begin
         assign WIDTH_SIZED = WIDTH;
         assign high_bits   = (amount >= WIDTH_SIZED) ? 1'b1 : 1'b0;
         assign amount_fix  = dir ? (WIDTH_SIZED - amount) : amount;
      end
   
      // Determine whether an extra shift is needed for right shift.
      // Extra shift occurs on direction being right when not a maximal shift.
      // For widths that are not a power of two, the amount is corrected
      // using a subtraction.
      if ((2 ** LOGWIDTH) == WIDTH) begin
         assign shift_gen0 = dir & ~high_bits;
      end else begin
         assign shift_gen0 = 1'b0;
      end

      // Generate the initial bit shift for right shifts.
      // The extra initial shift is needed for the opposite to the default direction
      // because a shift right by N is the same as a shift left by WIDTH - N.
      // The rest of that setup is handled by the masking logic.
      for (bit_cnt = 0; bit_cnt < WIDTH; bit_cnt = bit_cnt + 1) begin : shift_initial_gen
         assign shift_val[0][bit_cnt] = (shift_gen0) ? a[(bit_cnt + WIDTH - 1) % WIDTH] : a[bit_cnt];
      end
      
      // Iterate over all power of two generations...
      for (gen_cnt = 0; gen_cnt < LOGWIDTH; gen_cnt = gen_cnt + 1) begin : shift_mask_gen
      
         // Determine whether to shift this generation. These are wrapping shifts.
         // The inclusion of swapping meaning for the opposite direction completes the
         // logic of a shift right by N being the same as a shift left by WIDTH - N.
         // In other words, the extra shift plus this conditional inversion via XOR give
         // us an optimized complement.
         // For widths that are not a power of two, the amount is corrected
         // using a subtraction.
         if ((2 ** LOGWIDTH) == WIDTH) begin
            assign shift_gen[gen_cnt] = (dir ^ amount[gen_cnt]) & ~high_bits;
         end else begin
            assign shift_gen[gen_cnt] = amount_fix[gen_cnt] & ~high_bits;
         end

         // Generate each bit's shift amount.
         for (bit_cnt = 0; bit_cnt < WIDTH; bit_cnt = bit_cnt + 1) begin : shift_gens_other
            assign shift_val[gen_cnt + 1][bit_cnt] = (shift_gen[gen_cnt]) ?
               shift_val[gen_cnt][(bit_cnt + WIDTH - (2 ** gen_cnt)) % WIDTH] : shift_val[gen_cnt][bit_cnt];
         end
         
         // Determine whether to mask this generation
         assign mask_gen[gen_cnt] = amount[gen_cnt] | high_bits;

         // Generate each bit's mask value.
         assign mask_val[gen_cnt][(2 ** gen_cnt) - 1] = mask_gen[gen_cnt];
         for (bit_cnt = 0; bit_cnt < (2 ** gen_cnt) - 1; bit_cnt = bit_cnt + 1) begin : mask_gens_other
            assign mask_val[gen_cnt][bit_cnt] = mask_gen[gen_cnt] | mask_val[gen_cnt - 1][bit_cnt];
            if (bit_cnt + (2 ** gen_cnt) < WIDTH) begin
               assign mask_val[gen_cnt][bit_cnt + (2 ** gen_cnt)] = mask_gen[gen_cnt] & mask_val[gen_cnt - 1][bit_cnt];
            end
         end
      end
      
      // Determine the final mask value
      assign mask_val[LOGWIDTH - 1][WIDTH - 1] = high_bits;
      for (bit_cnt = 0; bit_cnt < WIDTH; bit_cnt = bit_cnt + 1) begin : mask_final_gen
         assign mask_final[bit_cnt] = (dir) ? mask_val[LOGWIDTH - 1][WIDTH - bit_cnt - 1]: mask_val[LOGWIDTH - 1][bit_cnt];
      end
      
      // Determine the final and lost values
      // Output values are one of three things:
      //   Normal values when not masked,
      //   Arithmetic extension values when masked, or
      //   Arithmetic extension that matches the input value (masking doesn't matter)
      // Lost values are just any input bits that are masked.
      for (bit_cnt = 0; bit_cnt < WIDTH; bit_cnt = bit_cnt + 1) begin : output_gen
         assign res[bit_cnt] = (~mask_final[bit_cnt] & shift_val[LOGWIDTH][bit_cnt] & ~arith_bit) |
                               (arith_bit & mask_final[bit_cnt]) |
                               (arith_bit & shift_val[LOGWIDTH][bit_cnt]);
         assign lost[bit_cnt] = shift_val[LOGWIDTH][bit_cnt] & mask_final[bit_cnt];
      end
   endgenerate

   // ----- Register Logic -----

   // ----- Sub-Modules -----

// ----- End of Module -----
endmodule

/* Helper Modules ********************************************************** */

/* End of File ************************************************************* */
