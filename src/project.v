/* ************************************************************************* *
 * @file      cpu_library.v
 * @author    nitelich
 * @since     2024-07-01
 * @copyright Copyright &copy; 2025 nitelich, all rights reserved
 * @brief
 *
 * A set of microcontroller/central processing unit definitions based
 * around lower level libraries, in turn built on only primitive operations.
 *
 * @details
 *
 * Contains implementations of the following high-level modules:
 *   - 8-bit "RISCY Jr." CPU
 *   - 8-bit "RISCY Jr." MCU targeting Tiny Tapeout
 *
 * These high-level modules require a number of helper low-level modules:
 *   - "RISCY Jr." memory manager targeting Tiny Tapeout
 *
 * Modules are written with some amount of customization, though as top
 * level MCU/CPU definitions, they are intended to work for a fixed
 * data/address bit width.
 *
 * No potential future extensions identified as of this writing.
 *
 * ************************************************************************* */

/**
 * Set a simulation timescale of 1ns per "tick" and a display resolution
 * of 100ps.
 */
`timescale 1ns/1ns

/**
 * Library version history:
 *   v0.1.0 - Initial feature complete version.
 *            Started 2024-07-01, Completed 2025-07-27
 */
`define VERSION_CPU "0.1.0"

// Include the shared definitions file.
`include "shared_definitions.v"

/* Helper Macros *********************************************************** */

/** PSRNG default value modifier 1. This simplifies PSRNG config.*/
`define PSRNG_DEF_XOR_1    8'h44
/** PSRNG default value modifier 2. This simplifies PSRNG config.*/
`define PSRNG_DEF_XOR_2    8'hCC

/* Primary Modules ********************************************************* */

/**
 * RISCY Jr. CPU implementation. Contains 16 instructions and 4 registers.
 *   - 0/1 = AND_/NAND Inverting/Non-Inverting Logical And
 *   - 2/3 = SLL_/SRL_ Shift Right/Left Logical
 *   - 4/5 = UADD/SADD Unsigned/Signed Add
 *   - 6/7 = USUB/SSUB Unsigned/Signed Subtract
 *   - 8/9 = RJMP/AJMP Relative Jump on Zero/Absolute Preserving Jump
 *   - A/B = LDIL/LDIU Load Immediate Lower/Upper Nybble
 *   - C/D = SAVE/LOAD Register to Memory/Memory to Register
 *   - E/F = CTLP/CTLM Control Process/Control Memory
 * 
 * All instructions other than LDIL, LDIU, and CTLP, and CTLM take two
 * registers as operands: the output register and the first operand register.
 * Where a second operand exists, it is assumed to be $R0.
 * Note that the "output" register is repurposed as a second operand for the
 * RJMP instruction.
 * Note that the second operand ($R0) for AJMP indicates the memory bank, and
 * is overwritten by the return memory bank.
 * 
 * Instructions LDIL and LDIU take in a 4-bit immediate value.
 * 
 * The CTLP instruction takes in a 4-bit action mask for the processor, where
 * the higher bit actions take precedence over the lower bits. Enabling no
 * flags is equivalent to a "no op" instruction.
 *   - 8 = Halt
 *   - 4 = Wait for Interrupt
 *   - 2 = Disable Interrupts
 *   - 1 = Enable Interrupts
 *
 * The CTLM instruction takes one register address and two bits of immediate
 * data. The address indicates the data memory page to load if no immediate
 * bits are set. If either immediate bits are set, this function returns from
 * any active interrupt, completing the interrupt. If either immediate bits
 * are set and no interrupt is active, no action is taken.
 * It is suggested that page 0x00 indicate internal memory, but this is not
 * required by this module.
 *
 * Note that program and data memories may be separated (Harvard) or
 * combined (von Neumann) architectures, as this module does not care as long
 * as both memories can be accessed simultaneously.
 *
 * @param[in] clk The module clock.
 * @param[in] rst The module reset.
 * @param[in] in_hold An indication that the CPU is holding internally, and
 * the memory manager should not perform any operations.
 * @param[in] hold_ext An external hold indication, intended to be from memory
 * accesses that take longer than a single cycle.
 * @param[in] irq An interrupt request signal. The IRQ should be held by the
 * requesting peripheral until either the corresponding ISF is raised or the
 * interrupt source is no longer valid (i.e. the interrupt was missed).
 * @param[in] irq_src The source vector of the interrupt. The vector table
 * exists on memory page 0x00. Due to the difficulty of loading far (off-page)
 * jumps, a limited number of interrupt slots are given such that they all can
 * be conceivably loaded on memory page 0x00.
 * @param[in] prog The program data from the current program address on the
 * current program memory page.
 * @param[in] data_in The variable data in from the current data address on the
 * current data memory page.
 * @param[out] active An indication that the CPU is actively executing. The CPU
 * is not actively executing when it is halted or sleeping.
 * @param[out] isf Flag indicating that a requested interrupt is being serviced,
 * and so the IRQ can be dropped.
 * @param[out] p_address The program address.
 * @param[out] data_out The variable data out to the current data address on
 * the current data memory page.
 * @param[out] d_address The data address.
 * @param[out] data_read An indication to read variable data for use on the next
 * clock cycle. This implies all memory operations require at least two clock
 * cycles.
 * @param[out] data_write An indication to write variable data on the next
 * clock cycle. This implies all memory operations require at least two clock
 * cycles.
 * @param[out] branch_taken Indicates whether or not a branch was taken for a
 * conditional jump, such that a memory manager can load the appropriate next
 * program address. This counts on both relative and absolute jumps.
 * @param[out] arith_ovf Indicates when arithmetic overflow occurs.
 * @param[out] n_reg With the peripheral output extension, intended for
 * a MiSTer core variant of this module, normal register data.
 * @param[out] i_reg With the peripheral output extension, intended for
 * a MiSTer core variant of this module, interrupt register data.
 * @note 2024-08-29 Tested with "tdat_riscy_jr_rom.txt" program.
 */
module RiscyJrCPU(input clk, input rst, output in_hold, input hold_ext, output active,
                  input irq, input [`RJR_IRQ - 1:0] irq_src, output isf,
                  input [`RJR_WIDTH - 1:0] prog, output [`RJR_ADDR - 1:0] p_address,
                  input [`RJR_WIDTH - 1:0] data_in, output [`RJR_WIDTH - 1:0] data_out,
                  output [`RJR_ADDR - 1:0] d_address, output data_read, output data_write,
                  output branch_taken, output arith_ovf
                  );

   // ----- Constants -----
   localparam [`RJR_WIDTH - 1:0] zeroes = {`RJR_WIDTH{1'b0}}; /**< Constant wire for data sized zero.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_AND_ = 4'h0; /**< Instruction code - AND.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_NAND = 4'h1; /**< Instruction code - NAND.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_SLL_ = 4'h2; /**< Instruction code - Shift left logical.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_SRL_ = 4'h3; /**< Instruction code - Shift right logical.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_UADD = 4'h4; /**< Instruction code - Add unsigned.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_SADD = 4'h5; /**< Instruction code - Add signed.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_USUB = 4'h6; /**< Instruction code - Sub unsigned.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_SSUB = 4'h7; /**< Instruction code - Sub signed.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_RJMP = 4'h8; /**< Instruction code - Relative jump.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_AJMP = 4'h9; /**< Instruction code - Absolute jump.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_LDIL = 4'hA; /**< Instruction code - Load immediate lower.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_LDIU = 4'hB; /**< Instruction code - Load immediate upper.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_SAVE = 4'hC; /**< Instruction code - Save to memory.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_LOAD = 4'hD; /**< Instruction code - Load from memory.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_CTLP = 4'hE; /**< Instruction code - Control process.*/
   localparam [`RJR_HWIDTH- 1:0] CODE_CTLM = 4'hF; /**< Instruction code - Control memory.*/

   // ----- Local Variables -----
   wire is_sub;        /**< For add/sub instructions, choice of add or sub.*/
   wire opt_bit;       /**< Option bit from instruction. Chooses between instruction pairs.*/
   wire rjmp_cmp;      /**< Comparison result for relative jumps.*/
   wire auto_page;     /**< Indicates when to automatically move to the next program data page.*/
   wire wr_reg_0;      /**< Indicates when to write out to the normal register.*/
   wire wr_reg_1;      /**< Indicates when to write out to the extended register for AJMP bank.*/
   wire rd_int;        /**< Internal data read value.*/
   wire wr_int;        /**< Internal data write value.*/
   wire hold_ctlp;     /**< Hold from wait or halt.*/
   wire hold_svld;     /**< Save/load memory access hold signal. Acts as hold until memory manager holds.*/
   reg  did_mem;       /**< Already did the initial memory hold for a signal.*/
   wire in_irq;        /**< Indicator of new IRQ.*/
   reg  irq_state;     /**< IRQ enable/disable state.*/
   reg  irq_prog;      /**< IRQ handling in-progress flag.*/
   reg  isf_int;       /**< Internal interrupt service flag.*/
   wire active_int;    /**< Internal activity flag.*/
   wire arith_ovf_int; /**< Arithmetic overflow interrupt flag.*/
   wire ctlm_page;     /**< CTLM instruction is page update.*/
   wire ctlm_jmp;      /**< CTLM instruction is IRQ jump.*/
   reg  branch_buf;    /**< Buffer for branch taken signal, also allowing IRQ jump indication.*/
   wire [`RJR_DEPTH - 1:0] addr_reg_a;      /**< Address to look up for the first operand.*/
   wire [`RJR_DEPTH - 1:0] addr_reg_b;      /**< Address to look up for the second operand (usually 0).*/
   wire [`RJR_DEPTH - 1:0] addr_reg_out;    /**< Address to look up for the output.*/
   wire [`RJR_HWIDTH- 1:0] inst;            /**< Instruction code from current instruction.*/
   wire [`RJR_WIDTH - 1:0] reg_a;           /**< Current value of first operand register (selected).*/
   wire [`RJR_WIDTH - 1:0] reg_a_norm;      /**< Current value of first operand register (normal operation).*/
   wire [`RJR_WIDTH - 1:0] reg_a_irq;       /**< Current value of first operand register (IRQ operation).*/
   wire [`RJR_WIDTH - 1:0] reg_b;           /**< Current value of second operand register (usually $R0) (selected).*/
   wire [`RJR_WIDTH - 1:0] reg_b_norm;      /**< Current value of second operand register (usually $R0) (normal op).*/
   wire [`RJR_WIDTH - 1:0] reg_b_irq;       /**< Current value of second operand register (usually $R0) (IRQ op).*/
   wire [`RJR_WIDTH - 1:0] reg_out_nand;    /**< Result from AND/NAND.*/
   wire [`RJR_WIDTH - 1:0] reg_out_shift;   /**< Result from shift right/left.*/
   wire [`RJR_WIDTH - 1:0] reg_out_add_sub; /**< Result from addition/subtraction.*/
   wire [`RJR_WIDTH - 1:0] reg_out;         /**< Final register out value.*/
   wire [`RJR_WIDTH - 1:0] data_imm;        /**< Immediate data from instruction.*/
   reg  [`RJR_WIDTH - 1:0] page_data;       /**< Page info for memory read/write.*/
   wire [`RJR_WIDTH - 1:0] prog_buf;        /**< Buffered program data input.*/
   wire [`RJR_ADDR - 1:0]  pc_inc;          /**< Incremented program counter value.*/
   wire [`RJR_ADDR - 1:0]  pc_rjmp;         /**< Relative jump program counter value.*/
   reg  [`RJR_ADDR - 1:0]  pc_curr;         /**< Internal program counter value.*/
   reg  [`RJR_ADDR - 1:0]  pc_irq_save;     /**< Saved program counter for IRQ return.*/

   // ----- Combinatorial Logic -----
   
   // Select between normal and IRQ handling register output
   assign reg_a = (irq_prog) ? reg_a_irq : reg_a_norm;
   assign reg_b = (irq_prog) ? reg_b_irq : reg_b_norm;
   
   // Buffer program input data to avoid metastability in simulation.
   assign prog_buf = prog | {`RJR_WIDTH{1'b0}};
   
   // Extract instruction code.
   assign inst = prog_buf[`RJR_WIDTH - 1:`RJR_HWIDTH];
   
   // Determine instruction-specific actions
   assign wr_reg_0 = (inst != CODE_RJMP) & (inst != CODE_SAVE) &
                     (inst != CODE_CTLP) & (inst != CODE_CTLM) &
                     ~hold_ext & ~hold_ctlp & ~hold_svld & ~in_irq;
   assign wr_reg_1 = (inst == CODE_AJMP) & (addr_reg_out != {`RJR_DEPTH{1'b0}}) &
                     ~hold_ext & ~hold_ctlp & ~hold_svld & ~in_irq;
   assign addr_reg_a = prog_buf[`RJR_HWIDTH - 1:`RJR_QWIDTH];
   assign addr_reg_b = ((inst == CODE_SAVE) | (inst == CODE_LOAD) | (inst == CODE_RJMP)) ?
                       prog_buf[`RJR_QWIDTH - 1:0] : {`RJR_DEPTH{1'b0}};
   assign addr_reg_out = (inst == CODE_LOAD) ? prog_buf[`RJR_HWIDTH - 1:`RJR_QWIDTH] :
                         ((inst == CODE_LDIL) | (inst == CODE_LDIU)) ? 
                         {`RJR_DEPTH{1'b0}} : prog_buf[`RJR_QWIDTH - 1:0];
   
   // Determine twiddle bits for operators
   assign is_sub    = inst[1]; // Selection of add/sub for those instructions
   assign opt_bit   = inst[0]; // Option bit for each instruction pair
   assign ctlm_page = (prog_buf[`RJR_HWIDTH - 1:0] == {`RJR_HWIDTH{1'b0}});
   assign ctlm_jmp  = (prog_buf[`RJR_QWIDTH - 1:0] != {`RJR_QWIDTH{1'b0}});
   
   // Capture immediate data from the instruction.
   assign data_imm = (opt_bit) ? {prog_buf[`RJR_HWIDTH - 1:0], reg_b[`RJR_HWIDTH - 1:0]} :
                                 {reg_b[`RJR_WIDTH - 1:`RJR_HWIDTH], prog_buf[`RJR_HWIDTH - 1:0]};
   
   // Data read and write from memory
   assign rd_int = (inst == CODE_LOAD) & ~in_irq;
   assign wr_int = (inst == CODE_SAVE) & ~in_irq;
   
   // Calculate AND/NAND result.
   assign reg_out_nand = (opt_bit) ? ~(reg_a & reg_b) : (reg_a & reg_b);
   
   // Determine the final register output value based on the instruction.
   assign reg_out =
      ((inst == CODE_AND_) | (inst == CODE_NAND)) ? reg_out_nand :
      ((inst == CODE_SRL_) | (inst == CODE_SLL_)) ? reg_out_shift :
      ((inst == CODE_UADD) | (inst == CODE_SADD) |
       (inst == CODE_USUB) | (inst == CODE_SSUB)) ? reg_out_add_sub :
      ((inst == CODE_RJMP) | (inst == CODE_AJMP)) ? pc_inc[`RJR_WIDTH - 1:0] :
      ((inst == CODE_LDIL) | (inst == CODE_LDIU)) ? data_imm :
      ((inst == CODE_SAVE) | (inst == CODE_LOAD)) ? data_in : zeroes;
   
   // Determine when overflow should be flagged
   assign arith_ovf = arith_ovf_int &
      ((inst == CODE_UADD) | (inst == CODE_SADD) |
       (inst == CODE_USUB) | (inst == CODE_SSUB));
   
   // Calculate zero comparison for relative jump.
   assign rjmp_cmp = (reg_a == zeroes);
   
   // Branches are considered "taken" on both relative and absolute jumps.
   assign branch_taken = branch_buf;
   
   // Determine when the processor has entered an IRQ
   assign in_irq = irq & irq_state & ~irq_prog & ~((inst == CODE_CTLP) & data_imm[3]);
   
   // Determine when the module is active
   assign active_int = ~(hold_ext | rst |
                         hold_svld | hold_ctlp);
   
   // Determine when to signal an internal hold
   assign hold_ctlp = (inst == CODE_CTLP) & (data_imm[3] | data_imm[2]);
   assign hold_svld = ((inst == CODE_LOAD) | (inst == CODE_SAVE)) & ~did_mem;
   assign in_hold   = (hold_ctlp | hold_svld) & ~in_irq;
   
   // Save passthrough values to output ports
   assign data_read  = rd_int;
   assign data_write = wr_int;
   assign p_address  = pc_curr;
   assign d_address  = {page_data, reg_b}; // Always comes from register B value
   assign data_out   = reg_a; // Always comes from register A value
   assign isf        = isf_int;
   assign active     = active_int;

   // ----- Register Logic -----
   
   /**
    * Primary registered process for the CPU core.
    * Handles register updates.
    * @param[in] clk The register clock.
    */
   always @(posedge clk) begin : cpu_registers
      
      // Initialize to zeroes on reset
      if (rst) begin
         did_mem     <= 1'b0;
         page_data   <= zeroes;
         pc_curr     <= {`RJR_ADDR{1'b0}};
         pc_irq_save <= {`RJR_ADDR{1'b0}};
         irq_state   <= 1'b1;
         isf_int     <= 1'b0;
         irq_prog    <= 1'b0;
         branch_buf  <= 1'b1; // Initial address is considered a "branch"
         
      // On (valid) interrupt, skip many of the normal things
      end else if (in_irq) begin
      
         // Hold all normal signals
         page_data <= page_data;
         irq_state <= irq_state;
         
         // Special handling for memory and hold signals
         did_mem   <= 1'b0; // No memory access in IRQ handling
         
         // Jump to the interrupt table location.
         // Because there is no "absolute jump immediate" equivalent, a fairly
         // wide depth is given for each interrupt handler entry.
         // Note that IRQ source 0 acts as a pseudo soft reset.
         // For the return address, skip to the next address on "wait for
         // interrupt" instructions, as the interrupt completes them.
         pc_curr     <= {{`RJR_ADDR - `RJR_IRQ - `RJR_IRQ_T{1'b0}}, irq_src, {`RJR_IRQ_T{1'b0}}};
         pc_irq_save <= ((inst == CODE_CTLP) & data_imm[2]) ? pc_inc : pc_curr;
         isf_int     <= 1'b1;
         irq_prog    <= (irq_src != {`RJR_IRQ{1'b0}}); // No IRQ reg use on reset IRQ!
         branch_buf  <= 1'b1; // IRQ is a taken branch
         
      // On hold, keep the program counter current and do not
      // perform any memory accesses
      end else if (hold_ctlp | hold_svld | hold_ext) begin
      
         // Hold all normal signals
         isf_int     <= 1'b0; // No interrupt being serviced.
         branch_buf  <= 1'b0; // The memory manager should have saved this state if needed.
         irq_prog    <= irq_prog;
         page_data   <= page_data;
         pc_curr     <= pc_curr;
         pc_irq_save <= pc_irq_save;
         irq_state   <= irq_state;
         
         // For external holds due to memory, make sure to
         // continue to request the memory access
         did_mem  <= 1'b1; // Did the memory hold - do not cycle state back and forth
      
      // Otherwise update the register state
      end else begin
      
         // Determine next program counter value
         pc_curr <= ((inst == CODE_CTLM) & ctlm_jmp & irq_prog) ? pc_irq_save :
                    (inst == CODE_AJMP) ? {reg_b, reg_a} :
                    ((inst == CODE_RJMP) & rjmp_cmp) ? pc_rjmp : pc_inc;
         
         // Retain the saved PC for IRQs
         pc_irq_save <= pc_irq_save;
         
         // Determine memory holds
         did_mem <= ((inst == CODE_LOAD) | (inst == CODE_SAVE)) & ~did_mem;
         
         // Determine when branches are taken
         branch_buf <= (rjmp_cmp & (inst == CODE_RJMP)) | (inst == CODE_AJMP) |
                       ((inst == CODE_CTLM) & ctlm_jmp & irq_prog);
         
         // Determine memory pages and interrupt returns
         if ((inst == CODE_CTLM) & ctlm_page) begin
            page_data <= reg_a;
            irq_prog  <= irq_prog;
         end else if (inst == CODE_CTLM) begin
            page_data <= page_data;
            irq_prog  <= 1'b0;
         end else begin
            page_data <= page_data;
            irq_prog  <= irq_prog;
         end
         
         // Determine interrupt state
         if ((inst == CODE_CTLP) & (data_imm[1])) begin
            irq_state <= 1'b0;
         end else if ((inst == CODE_CTLP) & (data_imm[0])) begin
            irq_state <= 1'b1;
         end else begin
            irq_state <= irq_state;
         end
         
         // No interrupt being serviced.
         isf_int   <= 1'b0;
      end
   end

   // ----- Sub-Modules -----
   
   /** Basic auto-increment of the program counter.*/
   IncrementKS #(.WIDTH(`RJR_ADDR))
            pc_autoinc(.a(pc_curr), .sign(1'b0),
                      .res(pc_inc), .ovf());
   
   /** Addition/subtraction submodule, for calculating relative PC jump amounts.*/
   AddSubKS #(.WIDTH(`RJR_ADDR))
            pc_addsub(.a(pc_curr), .b({{`RJR_WIDTH{reg_b[`RJR_WIDTH - 1]}}, reg_b}),
                      .sub(1'b0), .sign(1'b1), .res(pc_rjmp), .ovf());
   
   /** Barrel shift submodule, for calculating shift right/left results.*/
   BarrelShiftReduced #(.WIDTH(`RJR_WIDTH))
            alu_shift(.a(reg_a), .amount(reg_b), .dir(opt_bit),
                      .arith(1'b0), .res(reg_out_shift), .lost());
   
   /** Addition/subtraction submodule, for calculating add/sub results.*/
   AddSubKS #(.WIDTH(`RJR_WIDTH))
            alu_addsub(.a(reg_a), .b(reg_b), .sub(is_sub),
                      .sign(opt_bit), .res(reg_out_add_sub), .ovf(arith_ovf_int));

   /** Normal operation register file submodule, for representing the core registers.*/
   RegisterFile #(.WIDTH(`RJR_WIDTH), .DEPTH(`RJR_DEPTH))
            register_n(.clk(clk), .rst(rst),
                      .addr_write_0(addr_reg_out),
                      .addr_write_1({`RJR_DEPTH{1'b0}}),
                      .addr_read_0(addr_reg_a), .addr_read_1(addr_reg_b),
                      .en_write_0(wr_reg_0 & ~irq_prog), .en_write_1(wr_reg_1 & ~irq_prog),
                      .data_write_0(reg_out), .data_write_1(pc_curr[`RJR_ADDR - 1: `RJR_WIDTH]),
                      .data_read_0(reg_a_norm), .data_read_1(reg_b_norm), .err()
                      );

   /** Nested interrupt register file submodule, for representing the core registers.*/
   RegisterFile #(.WIDTH(`RJR_WIDTH), .DEPTH(`RJR_DEPTH))
            register_i(.clk(clk), .rst(rst),
                      .addr_write_0(addr_reg_out),
                      .addr_write_1({`RJR_DEPTH{1'b0}}),
                      .addr_read_0(addr_reg_a), .addr_read_1(addr_reg_b),
                      .en_write_0(wr_reg_0 & irq_prog), .en_write_1(wr_reg_1 & irq_prog),
                      .data_write_0(reg_out), .data_write_1(pc_curr[`RJR_ADDR - 1: `RJR_WIDTH]),
                      .data_read_0(reg_a_irq), .data_read_1(reg_b_irq), .err()
                      );

// ----- End of Module -----
endmodule

/**
 * Tiny Tapeout implementation of a MCU with a RISCY Jr. CPU core.
 *
 * This module uses the RP2040 on the Tiny Tapeout PCB for SPI RAM emulation.
 * By emulating 128kB (of a maximum 512kB) of SPI RAM in this way, the RP2040
 * can act as a Harvard architecture memory source, supplying 64kB of RAM and
 * 64kB of "ROM" to the RISCY Jr. core. To minimize the on-silicon ROM/RAM,
 * each byte of program and memory data is read or written only when requested.
 * For minimum latency of program data, the next/AJMP address is predictively
 * read while the current address is executing. "Cache miss" latency is only
 * taken on RJMP branches taken (no prediction is implemented) and IRQs.
 *
 * In practice, only 64kB of SPI RAM is accessed, with the lower 32kB being ROM
 * and the upper 32kB being RAM.
 *
 * SPI RAM operation is handled completely without CPU intervention. Data is
 * always MSB first. The behavior is to prioritize predictive ROM read, then
 * memory read/write, then cache miss ROM read (with delay).
 *   1. Assert hold to the CPU core, and SPI CS
 *   2. Send the appropriate command, write 0x02 or read 0x03
 *   3. Send the memory address to read/write
 *   4. Send the memory value (in case of writes) and read the memory
 *      value (in case of reads)
 *   5. Deassert SPI CS for one clock cycle
 *   6. Deassert hold to the CPU core
 *
 * ROM Map:
 *   - 0x0000 to 0x003F - Interrupt table. Each interrupt gets 8 instructions
 *     to allow for the limited instruction set to generate a valid jump. An
 *     efficient set of operations is: LDIU+LDIL handler addr, AND to copy
 *     addr to $R1, LDIU+LDIL handler bank, AJMP to handler, CTLM IRQ return,
 *     and then one extra instruction for a NO OP.
 *   - 0x0040 to 0x7FFF - Normal program data.
 *   - 0x8000 to 0xFFFF - Wrap of program data due to ROM/RAM address sharing.
 * RAM Map:
 *   - 0x0000 to 0x7FFF - Wrap of RAM data due to ROM/RAM address sharing.
 *                        Note that peripheral control memory is NOT wrapped,
 *                        and so the last bytes of pure RAM can be accessed here.
 *   - 0x8000 to 0xFFEF - Normal memory, default values undefined.
 *   - 0xFFF0 to 0xFFFF - Peripheral control memory, described below...
 *   - 0xFFF0 - RW - (Bit 0) PWM 0 enable, default value 0.
 *                   (Bit 1) PWM 1 enable, default value 0.
 *                   (Bit 2) PSRNG enable, default value 0.
 *                   (Bit 3) UART HW flow control enable, default value 0.
 *                   (Bits 5:4) UART stop/parity bit setting, default value 00 (none).
 *                   Other bits unused.
 *                   (Bit 7) UART parity error. Writing this bit clears any pending error.
 *   - 0xFFF1 - RW - (Bits 3:0) GPI value. No effect on writes.
 *                   (Bits 7:4) GPO value, default value 0.
 *   - 0xFFF2 - RW - PWM 0 half period, default value 0x10 (freq: clk / 32)
 *   - 0xFFF3 - RW - PWM 0 half duty, default value 0x08 (50% for default period)
 *   - 0xFFF4 - RW - PWM 1 half period, default value 0x20 (freq: (clk / 16) / 64)
 *   - 0xFFF5 - RW - PWM 1 half duty, default value 0x10 (50% for default period)
 *   - 0xFFF6 - R  - PSRNG data, may be cleared when the PSRNG is disabled.
 *   - 0xFFF7 - RW - PSRNG key 1, default value 0xE1
 *   - 0xFFF8 - RW - PSRNG key 2, default value 0x36
 *   - 0xFFF9 - RW - PSRNG key 3, default value 0xFA
 *   - 0xFFFA - RW - PSRNG initial value 1, default value 0x23. This initial
 *                   value is used for the first LFSR, and the other two
 *                   initial values are chosen by (init2 = init1 ^ 0x44) and
 *                   (init3 = init 1 ^ 0xCC). The values 0x00, 0x44, and 0xCC
 *                   should be avoided here to avoid putting the corresponding
 *                   LFSR of the PSRNG into a zero loop.
 *   - 0xFFFB - RW - UART last transmitted data. Writing this location starts
 *                   a transmit.
 *   - 0xFFFC - R  - UART last received data.
 *   - 0xFFFD - RW - UART clock half period, default value 1
 *                   (for 128kbaud at a 256kHz clk)
 *   - 0xFFFE - RW - (Bits 6:1) IRQ enable, default value 0. Bits 0 and 7 are
 *                   always active and writing to them has no effect.
 *   - 0xFFFF -  W - Writing an active bit invokes the corresponding IRQ.
 * Note that all "half periods" are (clk / (2 * T.half)), where a value of
 * zero will cause an unchanging clock, and so give a minimum usable divisor
 * of 2 and a maximum divisor of 510.
 * Note that peripherals operate independently of processing holds.
 *
 * This MCU includes 8 interrupts. Interrupts are configured as follows:
 *   - IRQ0 = Reset
 *     The equivalent of jumping to ROM address 0x0000. The CPU is reset,
 *     but memory retains its value.
 *   - IRQ1 = System Tick
 *     If enabled, this interrupt is called once every "second" assuming
 *     a clk of 256kHz. In other words, every 256000 clock cycles. A
 *     second tick is used instead of a millisecond tick due to the overall
 *     low throughput due primarily to memory latency.
 *   - IRQ2 = UART Transmit Complete
 *     If enabled, this interrupt will trigger when a UART byte transmission
 *     is completed by that peripheral.
 *   - IRQ3 = UART Receive Complete
 *     If enabled, this interrupt will trigger when a UART byte reception is
 *     completed by that peripheral.
 *   - IRQ4 = EXT 0 Rising Edge
 *     If enabled, this interrupt will trigger on the rising edge (given the
 *     registered value over two clock cycles) of the GPI 0/EXT 0
 *   - IRQ5 = EXT 1 Rising Edge
 *     If enabled, this interrupt will trigger on the rising edge (given the
 *     registered value over two clock cycles) of the GPI 1/EXT 1
 *   - IRQ6 = Trap on Overflow
 *     If enabled, this interrupt will trigger when addition or subtraction
 *     results in an overflow. Overflow happens a lot, so enable at your
 *     own risk.
 *   - IRQ7 = Processing Fault
 *     Always enabled. This interrupt will trigger when an internal fault
 *     condition occurs. It is suggested that the IRQ table entry for this
 *     interrupt halt the CPU core.
 * @param[in] ui_in Dedicated inputs. Utilized as follows:
 *   - ui[0] = GPI 0/EXT 0
 *     General purpose input, readable in code. May be set as an external
 *     interrupt signal.
 *   - ui[1] = GPI 1/EXT 1
 *     General purpose input, readable in code. May be set as an external
 *     interrupt signal.
 *   - ui[2] = GPI 2
 *     General purpose input, readable in code.
 *   - ui[3] = GPI 3
 *     General purpose input, readable in code.
 *   - ui[4] = SPI Clock to CPU Clock Divider LSB
 *   - ui[5] = SPI Clock to CPU Clock Divider MSB
 *     By default, the SPI Clock is run 36x faster than the CPU clock. These
 *     two inputs allow the division to increase to 38x, 40x, or 42x faster.
 *   - ui[6] = External Hold
 *     When asserted, the CPU core ignores its clock and instead progresses
 *     on the rising edge of the external step signal. Note that peripherals
 *     will still utilize the system clock. Think of this as "debug" mode.
 *   - ui[7] = External Step
 *     When an external hold is active, acts as a single step for the CPU
 *     core, essentially a manual clock signal.
 * @param[out] uo_out Dedicated outputs. Utilized as follows:
 *   - uo[0] = GPO 0
 *     General purpose output, controllable in code.
 *   - uo[1] = GPO 1
 *     General purpose output, controllable in code.
 *   - uo[2] = GPO 2
 *     General purpose output, controllable in code.
 *   - uo[3] = GPO 3
 *     General purpose output, controllable in code.
 *   - uo[4] = PWM 0
 *     PWM output, channel 0, as currently configured. If not enabled,
 *     a low output is given.
 *   - uo[5] = PWM 1
 *     PWM output, channel 0, as currently configured. If not enabled,
 *     a low output is given.
 *   - uo[6] = PSRNG
 *     Current output of an "alternating step generator" style pseudo-random
 *     number generator. When enabled, clocks a new random bit each clock
 *     cycle. Not intended as a true random number generator, so don't expect
 *     mathematically or cryptographically secure randomness!
 *   - uo[7] = CPU activity indicator
 *     Dedicated output for indicating CPU activity. The CPU core is inactive
 *     when halted, waiting for interrupts, waiting for memory accesses, or
 *     signaled by a peripheral to wait.
 * @param[in] uio_in Bidirectional pins when in input mode. See uio_oe for
 * implementation details of the bidirectional pins.
 * @param[out] uio_out Bidirectional pins when in output mode. See uio_oe for
 * implementation details of the bidirectional pins.
 * @param[out] uio_oe Bidirectional pins' direction (0 = input, 1 = output).
 * Bidirectional pins are utilized as follows:
 *   - uio[0] = RP2040 GPIO21 = SPI RAM CS (always output)
 *     The SPI chip select for the emulated SPI RAM.
 *   - uio[1] = RP2040 GPIO22 = SPI RAM COPI (always output)
 *     The SPI controller-out, peripheral-in for the emulated SPI RAM. The
 *     emulated SPI RAM uses SPI modes 0 or 3, so output data is latched on
 *     the rising edge of the SCK signal.
 *   - uio[2] = RP2040 GPIO23 = SPI RAM CIPO (always input)
 *     The SPI controller-in, peripheral-out for the emulated SPI RAM. The
 *     emulated SPI RAM uses SPI modes 0 or 3, so input data is latched on
 *     the rising edge of the SCK signal.
 *   - uio[3] = RP2040 GPIO24 = SPI RAM SCK (always output)
 *     The SPI clock for the emulated SPI RAM. Max speed, considering all
 *     operations, is RP2040 MCLK / 10. The base RP2040 MCLK is 12MHz, so
 *     the maximum SPI operating frequency should be 1.2MHz. Note that different
 *     maximum frequencies (up to 12MHz) may be valid if a PLL is used for
 *     faster internal RP2040 clocking. To reach the target clock of the MCU,
 *     256kHz, and requiring 4 bytes (32 bits) to be transmitted over
 *     SPI for each data read/write + 4 control clock cycles, requires a SPI
 *     clock speed of 9.216MHz to read a memory location in one clock cycle,
 *     allowing any non-RJMP, non-memory access instruction, non-IRQ to be
 *     predictively read from ROM "just in time."
 *     Of course, the external hold signal from the memory manager will
 *     hold execution in the case of speed mismatch.
 *   - uio[4] = UART CTS (always input)
 *     UART clear-to-send input from the DCE. Optional use.
 *   - uio[5] = UART TXD (always output)
 *     UART transmit output from the DTE to the DCE, based on the current
 *     baud rate configuration. Transmitted data is clk aligned to this CPU.
 *   - uio[6] = UART RXD (always input)
 *     UART receive input from the DCE to the DTE, based on the current
 *     baud rate configuration. Received data is taken as the majority value
 *     of each frame of data across bit sub-frames, with ties being resolved
 *     as active bits.
 *   - uio[7] = UART RTS (always output)
 *     UART ready-to-send output from the DTE. Optional use.
 * @param[in] ena Enable pin for this module (over other modules in the Tiny
 * Tapeout silicon). Note that, in normal operation, this value is 1 any time
 * this module is powered.
 * @param[in] clk Master clock. While any value between 3Hz and 50MHz may be
 * generated and used for this signal (a theoretical max clock rate for the
 * CPU core is unknown as of this writing), a value of 9.216MHz should be
 * used to allow for proper clocking of the SPI peripheral at 9.216MHz and
 * corresponding CPU speed of 256kHz. Other speeds may be used, bounded by
 * a 12MHz maximum SPI speed (12MHz master clock), or utilization of UART
 * and system tick peripherals. UART operation relies upon being able to
 * achieve well-defined baud rates within some percentage of precision, and
 * the 256kHz CPU clock is defined because 128kHz is a well-defined baud rate.
 * System tick operation assumes a 256kHz CPU clock to simplify timer design,
 * and will run fast or slow based on adjusting the CPU clock.
 * @param[in] rst_n Active low master reset.
 * @note 2025-07-27 Tested via MiSTer core and simulation using both
 *                  tdat_riscy_jr_rom and tdat_riscy_jr_periph_rom test programs.
 */
module tt_um_nitelich_riscyjr (input  wire [7:0] ui_in,   output wire [7:0] uo_out,
      input wire [7:0] uio_in, output wire [7:0] uio_out, output wire [7:0] uio_oe,
      input wire ena,          input  wire       clk,     input  wire       rst_n
      );

   // ----- Constants -----
   localparam [`RJR_WIDTH - 1:0] zeroes = {`RJR_WIDTH{1'b0}}; /**< Constant wire for data sized zero.*/
   // ----- Sized constants for peripheral register addressing
   localparam [`RJR_HWIDTH- 1:0] REG_PEREN = 4'h0; /**< Register - Peripheral Enable.*/
   localparam [`RJR_HWIDTH- 1:0] REG_GPIO  = 4'h1; /**< Register - GPIO Control.*/
   localparam [`RJR_HWIDTH- 1:0] REG_PWM0T = 4'h2; /**< Register - PWM 0 Half Period.*/
   localparam [`RJR_HWIDTH- 1:0] REG_PWM0D = 4'h3; /**< Register - PWM 0 Half Duty.*/
   localparam [`RJR_HWIDTH- 1:0] REG_PWM1T = 4'h4; /**< Register - PWM 1 Half Period.*/
   localparam [`RJR_HWIDTH- 1:0] REG_PWM1D = 4'h5; /**< Register - PWM 1 Half Duty.*/
   localparam [`RJR_HWIDTH- 1:0] REG_RNGD  = 4'h6; /**< Register - PSRNG Data.*/
   localparam [`RJR_HWIDTH- 1:0] REG_RNGK1 = 4'h7; /**< Register - PSRNG Key 1.*/
   localparam [`RJR_HWIDTH- 1:0] REG_RNGK2 = 4'h8; /**< Register - PSRNG Key 2.*/
   localparam [`RJR_HWIDTH- 1:0] REG_RNGK3 = 4'h9; /**< Register - PSRNG Key 3.*/
   localparam [`RJR_HWIDTH- 1:0] REG_RNGI  = 4'hA; /**< Register - PSRNG Initial Value.*/
   localparam [`RJR_HWIDTH- 1:0] REG_URTTX = 4'hB; /**< Register - UART Tx Data.*/
   localparam [`RJR_HWIDTH- 1:0] REG_URTRX = 4'hC; /**< Register - UART Rx Data.*/
   localparam [`RJR_HWIDTH- 1:0] REG_URTT0 = 4'hD; /**< Register - UART Half Period.*/
   localparam [`RJR_HWIDTH- 1:0] REG_IRQEN = 4'hE; /**< Register - IRQ Enable.*/
   localparam [`RJR_HWIDTH- 1:0] REG_IRQHL = 4'hF; /**< Register - IRQ Hold/Invoke.*/

   // ----- Local Variables -----
   integer i;          /**< Counter for loops.*/
   wire clk_cpu;       /**< Clock for CPU core and peripherals.*/
   wire clk_mmu;       /**< Tick for MMU "slow clock" update.*/
   reg  clk_cpu_tick;  /**< Clock for CPU core and peripherals has ticked.*/
   reg  clk_cpu_tick_buff; /**< Buffered clock for CPU core to align clock usage.*/
   wire rst_master;    /**< Master external reset.*/
   wire a_data_periph; /**< Is the data address in the peripheral memory space?*/
   wire r_data;        /**< Data read from CPU core.*/
   wire r_data_mem;    /**< Data read directed to memory manager.*/
   wire w_data;        /**< Data write from CPU core.*/
   wire w_data_mem;    /**< Data write directed to memory manager.*/
   wire w_data_periph; /**< Data write directed to peripheral memory.*/
   wire uart_write;    /**< UART transmit start flag.*/
   wire uart_parity;   /**< UART parity error flag.*/
   wire hold_mem;      /**< Hold from memory manager.*/
   wire hold_any;      /**< Hold from any source.*/
   wire in_hold;       /**< Internal hold indication from CPU core.*/
   reg  step_val;      /**< Registered value of single step input.*/
   reg  step_do;       /**< Indicates a rising edge in the single step for one clock cycle.*/
   wire irq_any;       /**< Any IRQ sources are active.*/
   wire isf;           /**< Interrupt serviced flag from the CPU core.*/
   wire branch_taken;  /**< RJMP taken flag.*/
   wire arith_ovf;     /**< Arithmetic overflow from CPU core.*/
   reg  [2:0]              rst;              /**< Staged internal resets.*/
   wire [`RJR_IRQ - 1:0]   irq_src;          /**< Current IRQ source.*/
   wire [`RJR_HWIDTH- 1:0] a_periph;         /**< Peripheral memory address.*/
   reg  [`S2C_WIDTH - 1:0] clk_cpu_count;    /**< Count for the slow clock.*/
   wire [`S2C_WIDTH - 1:0] spi_to_cpu_div;   /**< SPI domain to CPU domain clock divider.*/
   wire [`RJR_WIDTH - 1:0] irq;              /**< Transient IRQ flags.*/
   reg  [`RJR_WIDTH - 1:0] irq_hold;         /**< Registered and held IRQ flags.*/
   wire [`RJR_WIDTH - 1:0] psrng_data;       /**< Current PSRNG data.*/
   reg  [`RJR_WIDTH - 1:0] registers [0:15]; /**< Peripheral configuration registers.*/
   wire [`RJR_WIDTH - 1:0] d_prog;           /**< Program data from memory manager.*/
   wire [`RJR_WIDTH - 1:0] dr_data;          /**< Data read to CPU core.*/
   wire [`RJR_WIDTH - 1:0] dr_data_mem;      /**< Data read from memory manager.*/
   wire [`RJR_WIDTH - 1:0] dr_data_periph;   /**< Data read from peripheral registers.*/
   wire [`RJR_WIDTH - 1:0] dw_data;          /**< Data written from CPU core.*/
   wire [`RJR_WIDTH - 1:0] uart_rx_data;     /**< Receive data from UART peripheral.*/
   wire [`RJR_ADDR - 1:0]  a_prog;           /**< Address of program data.*/
   wire [`RJR_ADDR - 1:0]  a_data;           /**< Address of memory data.*/

   // ----- Combinatorial Logic -----
   
   // All UIO pin directions are static:
   //   MSB [RTS (out), RXD (in),  TXD (out),  CTS (in),
   //        SCK (out), CIPO (in), COPI (out), nCS (out)
   // Also assign unused UIO outputs to 0.
   assign uio_oe = 8'b10101011;
   assign uio_out[2] = 1'b0;
   assign uio_out[4] = 1'b0;
   assign uio_out[6] = 1'b0;
   
   // Set general purpose outputs
   assign uo_out[3:0] = registers[REG_GPIO][7:4];
   assign uo_out[6] = psrng_data[0];
   
   // Determine master reset
   assign rst_master = ~rst_n | ~ena;
   
   // Determine the SPI to CPU clock divider, which can be tweaked by GPIs.
   // Note that the LSB is truncated to allow for easy 50% duty from a PWM module.
   assign spi_to_cpu_div = 6'b100100 + {3'b0, ui_in[5:4], 1'b0};
   
   // Determine when to hold the CPU
   assign hold_any = hold_mem | ~step_do;
   
   // Clear unused interrupt flags.
   // These interrupts are detected directly on related state changes.
   assign irq[7:4] = 4'b0000;
   assign irq[0] = 1'b0;
   
   // Determine interrupt handling signals.
   // Note that the default interrupt source is a fault.
   assign irq_any = |(registers[REG_IRQHL] & registers[REG_IRQEN]);
   assign irq_src = registers[REG_IRQHL][0] & registers[REG_IRQEN][0] ? 3'd0 :
                    registers[REG_IRQHL][1] & registers[REG_IRQEN][1] ? 3'd1 :
                    registers[REG_IRQHL][2] & registers[REG_IRQEN][2] ? 3'd2 :
                    registers[REG_IRQHL][3] & registers[REG_IRQEN][3] ? 3'd3 :
                    registers[REG_IRQHL][4] & registers[REG_IRQEN][4] ? 3'd4 :
                    registers[REG_IRQHL][5] & registers[REG_IRQEN][5] ? 3'd5 :
                    registers[REG_IRQHL][6] & registers[REG_IRQEN][6] ? 3'd6 : 3'd7;
   
   // Handle mapping of main vs. peripheral memory accesses.
   assign a_data_periph  = (a_data[`RJR_ADDR - 1:`RJR_HWIDTH] == {`RJR_ADDR - `RJR_HWIDTH{1'b1}});
   assign r_data_mem     = r_data & ~a_data_periph;
   assign w_data_mem     = w_data & ~a_data_periph;
   assign w_data_periph  = w_data &  a_data_periph & ~hold_any & ~in_hold;
   assign uart_write     = w_data_periph & (a_periph == REG_URTTX);
   assign dr_data        = a_data_periph ? dr_data_periph : dr_data_mem;
   assign a_periph       = a_data[`RJR_HWIDTH - 1:0];
   assign dr_data_periph = registers[a_periph];
   
   // Mask to get the CPU clock for slow peripherals
   assign clk_cpu = clk_cpu_tick_buff;
   assign clk_mmu = clk_cpu_tick; // Leads by one clock cycle as a level vs. edge item

   // ----- Register Logic -----
   
   /**
    * Primary registered process for fast clock updates for
    * the MCU. Intended primarily for reset timing.
    * @param[in] clk The fast register clock.
    */
   always @(posedge clk) begin : reset_and_glue
      
      // Initialize all signals on reset
      if (rst_master) begin
      
         // Initialize reset counter
         rst <= 3'b111;
         
         // Initialize clock counter
         clk_cpu_tick      <= 1'b0;
         clk_cpu_tick_buff <= 1'b0;
         clk_cpu_count     <= {`S2C_WIDTH{1'b0}};
      
         // Initialize everything other than peripheral registers to defaults
         step_val   <= 1'b0;
         step_do    <= 1'b0;
         
         // Initialize peripheral registers to defaults
         registers[REG_PEREN] <= zeroes; // Enables
                                  // MSB [UART Err  = RO; Unused = N/A;   UART Stop = 1bit; UART Par = None;
                                  //      UART Flow = SW; PSRNG En = Dis; PWM 1 En = Dis;   PWM 0 En = Dis]  LSB
         registers[REG_GPIO ] <= zeroes; // GPIO => MSB [GPO[3:0] = 0; GPI[3:0] = RO] LSB
         registers[REG_PWM0T] <= 8'h10;  // PWM0 Period => 16 = clk / 32
         registers[REG_PWM0D] <= 8'h08;  // PWM0 Duty => 8 = clk / 16
         registers[REG_PWM1T] <= 8'h20;  // PWM1 Period => 32 = clk / 64
         registers[REG_PWM1D] <= 8'h10;  // PWM1 Duty => 16 = clk / 32
         registers[REG_RNGD ] <= zeroes; // PSRNG data => RO, WRITTEN BY PERIPHERAL
         registers[REG_RNGK1] <= 8'hE1;  // PSRNG Key 1 => 0xE1
         registers[REG_RNGK2] <= 8'h36;  // PSRNG Key 2 => 0x36
         registers[REG_RNGK3] <= 8'hFA;  // PSRNG Key 3 => 0xFA
         registers[REG_RNGI ] <= 8'h23;  // PSRNG Init => 0x23
         registers[REG_URTTX] <= zeroes; // UART Tx => 0
         registers[REG_URTRX] <= zeroes; // UART Rx => RO, WRITTEN BY PERIPHERAL
         registers[REG_URTT0] <= 8'h01;  // UART Period => 1 = clk / 2
         registers[REG_IRQEN] <= 8'h81;  // IRQ En => IRQ 7 and IRQ 0, which are RO and always enabled
         registers[REG_IRQHL] <= zeroes; // IRQ Hold => 0
      
      // Otherwise update the register state
      end else begin
      
         // Clear resets in sequence
         if (rst[0]) begin
            rst <= 3'b110;
         end else if (rst[1] & clk_cpu_tick_buff) begin
            rst <= 3'b100;
         end else if (rst[2] & clk_cpu_tick_buff) begin
            rst <= 3'b000;
         end else begin
            rst <= rst;
         end
         
         // Increment the slow clock counter.
         clk_cpu_tick_buff <= clk_cpu_tick;
         clk_cpu_tick      <= (clk_cpu_count >= spi_to_cpu_div) ? 1'b1 : 1'b0;
         clk_cpu_count     <= (clk_cpu_count >= spi_to_cpu_div) ? {`S2C_WIDTH{1'b0}} :
                              clk_cpu_count + {{`S2C_WIDTH - 1{1'b0}}, 1'b1};
         
         // Update registers and IRQs only when the MMU would update.
         // These updates should be synchronous.
         if (clk_mmu) begin
         
            // ----- External hold and step
            
            // Latch edges of the external step input to determine when to
            // do step updates.
            if (~ui_in[6]) begin
               step_val <= 1'b0;
               step_do  <= 1'b1;
            end else if (~step_val & ui_in[7]) begin
               step_val <= 1'b1;
               step_do  <= 1'b1;
            end else if (step_val & ~ui_in[7]) begin
               step_val <= 1'b0;
               step_do  <= 1'b0;
            end else if (hold_mem) begin
               // Hold steps when holding otherwise, so each step advances the PC
               step_val <= step_val;
               step_do  <= step_do;
            end else begin
               step_val <= step_val;
               step_do  <= 1'b0;
            end
            
            // ----- Peripheral register handling
            
            // By default, keep the peripheral register values.
            for (i = 0; i < `RJR_WIDTH; i = i + 1) begin
               registers[i] <= registers[i];
            end
            
            // Handle any register writes. Read-only words or bits should be
            // overwritten after this point.
            if (w_data_periph) begin
               registers[a_periph] <= dw_data;
            end
            
            // Reg 0: Update UART parity error bit and unused bit.
            registers[REG_PEREN][7] <= uart_parity;
            registers[REG_PEREN][6] <= 1'b0;
            
            // Reg 1: Update GPIs
            registers[REG_GPIO][`RJR_HWIDTH - 1:0] <= ui_in[3:0];
            
            // Reg 2 - 5: No read-only updates needed
            
            // Reg 6: Update the PRNG value
            registers[REG_RNGD] <= psrng_data;
            
            // Reg 7 - 11: No read-only updates needed
            
            // Reg 12: Always update the UART Rx value
            registers[REG_URTRX] <= uart_rx_data;
            
            // Reg 13: No read-only updates needed
            
            // Reg 14: Keep the reset and fault IRQs enabled
            registers[REG_IRQEN][7] <= 1'b1;
            registers[REG_IRQEN][0] <= 1'b1;
            
            // Reg 15: Register IRQs into the hold, and clear them when they are done.
            // All IRQs should be cleared on program address zero (a reset)
            for (i = 1; i < `RJR_WIDTH; i = i + 1) begin
               if (irq[i]) begin
                  registers[REG_IRQHL][i] <= 1'b1;
               end else if ((isf & (i == irq_src)) | (a_prog == {`RJR_ADDR{1'b0}})) begin
                  registers[REG_IRQHL][i] <= 1'b0;
               end
            end
            
            // ----- IRQ handling
            
            // IRQ 0: Only triggered by register 15 write
            // IRQ 1 - 3: Triggered by peripherals via IRQ wire
            
            // IRQ 4 - 5: Determine when EXTI should be triggered
            if (~registers[REG_GPIO][0] & ui_in[0]) begin
               registers[REG_IRQHL][4] <= 1'b1;
            end
            if (~registers[REG_GPIO][1] & ui_in[1]) begin
               registers[REG_IRQHL][5] <= 1'b1;
            end
            
            // IRQ 6: Trigger on overflow
            if (arith_ovf) begin
               registers[REG_IRQHL][6] <= 1'b1;
            end
            
            // IRQ 7: Only triggered by register 15 write
         end
      end
   end

   // ----- Sub-Modules -----
   
   // ----- Core Modules
   
   /** CPU core.*/
   RiscyJrCPU  core_cpu(.clk(clk_cpu), .rst(rst[1]), .in_hold(in_hold),
                        .hold_ext(hold_any), .active(uo_out[7]),
                        .irq(irq_any), .irq_src(irq_src), .isf(isf),
                        .prog(d_prog), .p_address(a_prog),
                        .data_in(dr_data), .data_out(dw_data),
                        .d_address(a_data), .data_read(r_data), .data_write(w_data),
                        .branch_taken(branch_taken), .arith_ovf(arith_ovf)
                        );
   /**
    * Memory management unit to translate SPI RAM into CPU core memory.
    * @note Does not handle peripheral memory control, and so signals are
    * modified for accessing that segment of memory.
    */
   RiscyJrMemoryManager
         mem_manage_cpu(.clk(clk), .rst(rst[0]), .clk_slow(clk_mmu),
                        .cpu_held(in_hold), .exe_held(~step_do), .hold(hold_mem),
                        .spi_sck(uio_out[3]), .spi_copi(uio_out[1]),
                        .spi_cipo(uio_in[2]), .spi_ncs(uio_out[0]),
                        .a_prog(a_prog), .a_data(a_data),
                        .r_data(r_data_mem), .w_data(w_data_mem), .m_prog(branch_taken),
                        .d_prog(d_prog), .dw_data(dw_data), .dr_data(dr_data_mem));
   
   // ----- Main Peripherals
   
   /**
    * Universal Asynchronous Receiver/Transmitter peripheral.
    * @note Data is latched directly from the data write, rather than
    *       buffered from the register. This mode seems to be more stable.
    */
   UART#(.WIDTH(`RJR_WIDTH), .DEPTH(`RJR_WIDTH))
            periph_uart(.clk(clk_cpu), .rst(rst[2]), .trigger(uart_write), .err_parity(uart_parity),
                        .cdiv(registers[REG_URTT0]), .flow(registers[REG_PEREN][3]),
                        .stop_parity(registers[REG_PEREN][5:4]),
                        .rxd(uio_in[6]), .cts(uio_in[4]), .data_rx(uart_rx_data), .new_rx(irq[3]),
                        .txd(uio_out[5]), .rts(uio_out[7]), .data_tx(dw_data), .ready_tx(), .done_tx(irq[2]));
   /** Pseudo-Random Number Generator peripheral utilizing an alternating step generator.*/
   AlternatingStepGenerator #(.WIDTH(`RJR_WIDTH), .GALOIS(1))
           periph_psrng(.clk(clk_cpu), .rst(rst[2]), .en(registers[REG_PEREN][2]),
                        .taps0(registers[REG_RNGK1]), .taps1(registers[REG_RNGK2]), .taps2(registers[REG_RNGK3]),
                        .def0(registers[REG_RNGI]), .def1(registers[REG_RNGI] ^ `PSRNG_DEF_XOR_1),
                        .def2(registers[REG_RNGI] ^ `PSRNG_DEF_XOR_2), .data(psrng_data));
   /** PWM peripheral 0. Range of (clk / 256) to (clk / 1).*/
   PulseWidthModulator #(.WIDTH(`RJR_WIDTH))
           periph_pwm_0(.clk(clk_cpu), .rst(rst[2] | ~registers[REG_PEREN][0]), .dir(1'b1),
                        .pwm(uo_out[4]), .ovf(),
                        .period_0(zeroes), .period_f(registers[REG_PWM0T]),
                        .period_d(registers[REG_PWM0D]), .val());
   /**
    * PWM peripheral 1. Range of (clk / 4096) to (clk / 16).
    * At a clk of 256kHz, this gives mostly audio range (62.5Hz to 16kHz).
    */
   PulseWidthModulator #(.WIDTH(`RJR_WIDTH + 4))
           periph_pwm_1(.clk(clk_cpu), .rst(rst[2] | ~registers[REG_PEREN][1]), .dir(1'b1),
                        .pwm(uo_out[5]), .ovf(),
                        .period_0({zeroes, 4'd0}), .period_f({registers[REG_PWM1T], 4'hF}),
                        .period_d({registers[REG_PWM1D], 4'd0}), .val());
   
   // ----- Special Peripherals
   
   /**
    * Wall-time one second counter.
    * Utilizes the starting value to count 256k rather than (2 ^ 18) clock cycles.
    * @note Assumes a clock speed of 256kHz for proper operation, of course.
    * @note The alternate default value of 262,000 is for testing, so that
    *       simulation times can remain feasible. The value of 6,144 is intended
    *       for normal operation.
    */
   CounterSync #(.WIDTH(18))
      periph_systick(.clk(clk_cpu), .rst(rst[2]), .dir(1'b1), .ovf(irq[1]),
                     .def(18'd6144), .val());

// ----- End of Module -----
endmodule

/* Helper Modules ********************************************************** */

/**
 * Memory manager for the RISCY Jr. Tiny Tapeout MCU implementation.
 * Acts as connector between the external SPI RAM and the internal memory.
 * Note that this memory manager does not handle peripheral memory, and so
 * memory reads and writes to that area should have the r_data and w_data
 * inputs masked to not spuriously request the data from the external RAM.
 *
 * This module assumes, as a special setup and hold condition, that predicted
 * program address, data address, and data to write are all valid within
 * one SPI transaction (presumably 1/4 of a CPU clock cycle) of the start of
 * a clock cycle. This saves on a full slow clock delay for latching that
 * data, but may need to be specially accommodated in constraints!
 *
 * Note that the SPI SRAM only emulates 64kB of memory. As such, half is given
 * each to ROM and RAM. Because ROM and RAM are accessed separately, values
 * wrap at the 32kB boundary.
 *
 * In the unlikely instance of multiple memory access flags being set at once,
 * the priority for handling is RAM read, then RAM write, then cache miss.
 * 
 * @param[in] clk The clock for the SPI transactions.
 * @param[in] rst The module reset.
 * @param[in] clk_slow The clock for the CPU core, used for caching one word
 * each of program and data.
 * @param[in] cpu_held The CPU core is holding internally - do not update
 * program data until a cache miss occurs.
 * @param[in] exe_held The CPU core is paused by external means (e.g.,
 * single-step mode).
 * @param[out] hold An indication to the CPU core to wait for memory to be
 * finish being read. In the circumstance of SPI-to-CPU speed imbalance, this
 * hold may be asserted with great frequency.
 * @param[out] spi_sck External SPI clock signal.
 * @param[out] spi_copi Serial data output line to external SPI.
 * @param[in] spi_cipo Serial data input line from external SPI.
 * @param[out] spi_ncs External SPI active low chip select.
 * @param[in] a_prog Address for the next program word.
 * @param[in] a_data Address for the next data word.
 * @param[in] r_data Request for reading a new data word. Takes precedence
 * over writing out a data word.
 * @param[in] w_data Request for writing a data word out.
 * @param[in] m_prog Indication of a cache miss for program data, either on a
 * relative jump whose branch was taken or an IRQ.
 * @param[out] d_prog Cached program word.
 * @param[in] dw_data Data word to write next.
 * @param[out] dr_data Cached data word (most recently read).
 * @note 2025-07-23 Tested for 8-bits with all combos of RAM/cache miss.
 *                  This test updated for cache misses taking precedence
 *                  over memory accesses.
 */
module RiscyJrMemoryManager(input clk, input rst, input clk_slow,
                            input cpu_held, input exe_held, output hold,
                            output spi_sck, output spi_copi, input spi_cipo, output spi_ncs,
                            input [`RJR_ADDR - 1:0] a_prog, input [`RJR_ADDR - 1:0] a_data,
                            input r_data, input w_data, input m_prog,
                            output [`RJR_WIDTH - 1:0] d_prog,
                            input [`RJR_WIDTH - 1:0] dw_data, output [`RJR_WIDTH - 1:0] dr_data);

   // ----- Constants -----

   // ----- Local Variables -----
   wire hold_new;    /**< New hold criteria.*/
   reg  hold_tran;   /**< Hold on active transmission.*/
   reg  hold_mem;    /**< Hold due to a RAM transaction.*/
   reg  hold_prog;   /**< Hold due to a cache miss.*/
   reg  did_mem;     /**< Do not hold after finishing a RAM transaction.*/
   reg  skip_step;   /**< On writes, we miss the window for latching the SPI write. Skip a turn.*/
   reg  mem_r;       /**< Memory read buffer.*/
   reg  mem_w;       /**< Memory write buffer.*/
   wire spi_trigger; /**< SPI trigger.*/
   wire spi_ready;   /**< SPI ready for next word.*/
   wire spi_done;    /**< SPI completion indication.*/
   reg  [1:0]              spi_count; /**< Count of SPI words transmitted.*/
   reg  [`RJR_WIDTH - 1:0] c_prog;    /**< Cached program data.*/
   reg  [`RJR_WIDTH - 1:0] c_data;    /**< Cached memory data.*/
   wire [`RJR_WIDTH - 1:0] spi_cmd;   /**< SPI command for next transmission.*/
   wire [`RJR_WIDTH - 1:0] spi_rsp;   /**< SPI response from the last transmission.*/
   wire [`RJR_ADDR  - 1:0] r_prog;    /**< Requested "next" program address.*/

   // ----- Combinatorial Logic -----
   
   // Set output values from internal values
   assign dr_data = c_data;
   assign d_prog  = c_prog;
   
   // Set external holds.
   // The exact timing of the transmission hold was hard-won.
   // Test benches look best including the trigger clock cycle,
   // but that causes a race condition freeze in hardware.
   assign hold    = hold_prog | hold_mem |               // Normal memory hold reasons
                    (hold_tran & ~(spi_ncs & spi_done)); // Transmission active, less final SPI cycle
   
   // Latch SPI trigger when its ready for new data
   // After the transaction completes, do not trigger again until
   // the next falling edge, in case of a super slow CPU clock.
   assign spi_trigger = (spi_ready & (spi_count != 2'd3)) |
                        (clk_slow & (spi_count == 2'd3));
   
   // New hold criteria:
   //  - New RAM access or cache miss
   //  - No existing hold in progress
   //  - No completed hold pending program update that allows continuation
   assign hold_new = (r_data | w_data | m_prog) & ~hold_prog & ~hold_mem & ~did_mem;
   
   // Requesting NEXT instruction, choose between:
   //  - Re-request, on program hold
   //  - Request next, otherwise
   assign r_prog = hold_prog ? a_prog : (a_prog + {{`RJR_ADDR - 1{1'b0}}, 1'b1});
   
   // Construct the next SPI command.
   // Because the SPI count is delayed compared to the SPI trigger
   // by one clock cycle, the count for each word is barrel shifted
   // by a single count. I don't really like this as a solution,
   // but the SPI trigger needs to be directly calculated from the
   // SPI ready signal to have back-to-back SPI transactions.
   assign spi_cmd = (spi_count == 2'd3) ? (((w_data & ~r_data & ~m_prog & hold_new) | (mem_w & hold_mem)) ?
                       `SPI_CMD_WRITE : `SPI_CMD_READ) :                      // SPI command
                    (spi_count == 2'd0) ? (hold_mem ?
                       {1'b1, a_data[`RJR_ADDR - 2:`RJR_WIDTH]} :             // Addr upper - RAM is upper half
                       {1'b0, r_prog[`RJR_ADDR - 2:`RJR_WIDTH]}) :            // Addr upper - ROM is lower half
                    (spi_count == 2'd1) ? (hold_mem ?
                       a_data[`RJR_WIDTH - 1:0] : r_prog[`RJR_WIDTH - 1:0]) : // Addr lower
                    dw_data;                                                  // Data for writes

   // ----- Register Logic -----

   /**
    * Primary registered process for the MM of the RISCY Jr. CPU.
    * @param[in] clk The main clock
    */
   always @(posedge clk) begin : memory_manager
      
      // Initialize to defaults on reset
      if (rst) begin
         spi_count <= 2'd3;
         mem_r     <= 1'b0;
         mem_w     <= 1'b0;
         hold_tran <= 1'b1;
         hold_mem  <= 1'b0;
         hold_prog <= 1'b1; // Address zero is a "cache miss"
         did_mem   <= 1'b0;
         skip_step <= 1'b0;
         c_prog    <= {`RJR_WIDTH{1'b0}};
         c_data    <= {`RJR_WIDTH{1'b0}};
      
      // Otherwise update the register state
      end else begin
         
         // Hold items by default
         spi_count <= spi_count;
         c_prog    <= c_prog;
         c_data    <= c_data;
         hold_tran <= hold_tran;
         hold_mem  <= hold_mem;
         hold_prog <= hold_prog;
         did_mem   <= did_mem;
         mem_r     <= mem_r;
         mem_w     <= mem_w;
   
         // Update data corresponding with triggers
         if (spi_trigger) begin
            spi_count <= (spi_count == 2'd3) ? 2'd0 : (spi_count + 2'd1);
         end
         
         // If a RAM access or cache miss happens, mark the RAM access and/or
         // cache miss. Note that a RAM access necessarily causes a cache miss
         // because we can't update the next program data until after the RAM
         // access is completed.
         // Uses instantaneous value rather than buffered values so that the
         // holds are valid the same clock cycle as the buffered values.
         // Note that, because program data is prioritized in the case of a
         // cache miss, RAM read/write signals may be blanked. This protects
         // against spurious reads/writes on SAVE/LOAD simultaneous with a
         // taken RJMP or a new IRQ.
         if (hold_new) begin
            mem_r     <= ~m_prog & r_data;
            mem_w     <= ~m_prog & w_data & ~r_data;   // Prioritize reads
            skip_step <= (w_data & ~r_data & ~spi_trigger) | // Skip a step when window is missed
                         (r_data & (spi_count != 3'd0) & ~spi_trigger);
            hold_mem  <= ~m_prog &  (r_data | w_data);
            hold_prog <=  m_prog | ~(r_data | w_data);
         end
         
         // Determine when to hold in case of still-active transmission.
         if (spi_ncs & spi_done) begin
            hold_tran <= 1'b0;
         end else if (spi_trigger) begin
            hold_tran <= 1'b1;
         end
         
         // Update "slow clock" items on clock roll-over
         if (clk_slow & ~hold_tran) begin
         
            // Memory read/write and holds only persist if skipping a step
            hold_mem  <= hold_mem  &  skip_step;
            hold_prog <= hold_prog &  skip_step;
            did_mem   <= hold_mem  & ~skip_step;
            mem_r     <= mem_r     &  skip_step;
            mem_w     <= mem_w     &  skip_step;
            
            // If the window was missed for a write, just clear that signal
            // and start the next sequence again.
            if (skip_step) begin
               skip_step <= 1'b0;
            // Otherwise, update read memory...
            end else if (mem_r & hold_mem) begin
               c_data    <= spi_rsp;
            // ...take no action on memory writes (that's in the SPI command)...
            end else if (mem_w & hold_mem) begin
               // No action
               
            // ... or finally update ROM on ROM reads.
            end else if (~(cpu_held | exe_held) | hold_prog) begin
               c_prog    <= spi_rsp;
            end
         end
      end
   end

   // ----- Sub-Modules -----
   
   /**
    * SPI controller for SPI RAM.
    * Because the SPI RAM defines the "fast" clock speed, no clock division
    * is used for the module.
    * Direction is always MSB first (dir = 1'b0).
    * SPI mode 0 (CPOL = 0, CPHA = 0) or 3 (CPOL = 1, CPHA = 0) are supported.
    * Using SPI mode 0 here arbitrarily.
    */
   ControllerSPI #(.WIDTH(`RJR_WIDTH), .DEPTH(1))
             spi_ram (.clk(clk), .rst(rst), .dir(1'b0), .trigger(spi_trigger), .cpol(1'b0), .cpha(1'b0),
                     .cdiv(1'b0), .cipo(spi_cipo), .data_co(spi_cmd),
                     .copi(spi_copi), .data_po(spi_rsp), .sck(spi_sck), .n_cs(spi_ncs),
                     .ready_next(spi_ready), .done_curr(spi_done));

// ----- End of Module -----
endmodule

/* End of File ************************************************************* */
