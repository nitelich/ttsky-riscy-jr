/* ************************************************************************* *
 * @file      shared_definitions.v
 * @author    nitelich
 * @since     2024-08-22
 * @copyright Copyright &copy; 2025 nitelich, all rights reserved
 * @brief
 *
 * A collection of standard definitions across all Verilog modules.
 *
 * @details
 *
 * Contains definitions related to the following:
 *   - Common sizes of things
 *   - Testing delays
 *   - Testing structural definitions (max errors, random test size, etc.)
 *   - Testing macros (abs, etc.)
 *
 * No potential future extensions identified at this time.
 *
 * ************************************************************************* */

/* Include Guard *********************************************************** */

`ifndef SHARED_DEFINITIONS
/** Include guard for this file.*/
`define SHARED_DEFINITIONS

/* Common Macros *********************************************************** */

// ----- RISCY Jr. constants

/** Data/address width for the RISCY Jr. CPU.*/
`define RJR_WIDTH          8
/** Log2 of data/address width for the RISCY Jr. CPU.*/
`define RJR_LOG_WIDTH      $clog2(`RJR_WIDTH)
/** Half width for the RISCY Jr. CPU.*/
`define RJR_HWIDTH         (`RJR_WIDTH / 2)
/** Quarter width for the RISCY Jr. CPU.*/
`define RJR_QWIDTH         (`RJR_WIDTH / 4)
/** Interrupt depth for the RISCY Jr. CPU.*/
`define RJR_IRQ            3
/** IRQ table entry depth for a single IRQ for the RISCY Jr. CPU.*/
`define RJR_IRQ_T          3
/** Register depth for the RISCY Jr. CPU.*/
`define RJR_DEPTH          2
/** Extended memory addressing for the RISCY Jr. CPU.*/
`define RJR_ADDR           16

/** VGA color depth for RISCY Jr. CPU.*/
`define RJR_CDEPTH         4

// ----- SPI RAM commands

/** SPI RAM command for a memory read.*/
`define SPI_CMD_READ       8'h03
/** SPI RAM command for a memory write.*/
`define SPI_CMD_WRITE      8'h02
/** Size of SPI domain to CPU core domain divider.*/
`define S2C_WIDTH          6

// End of include guard
`endif

/* End of File ************************************************************* */
