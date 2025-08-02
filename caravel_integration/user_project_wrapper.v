/*
 * SPDX-FileCopyrightText: 2025 Plutomaster28
 * SPDX-License-Identifier: Apache-2.0
 *
 * MCU-32X Caravel User Project Wrapper
 * Integrates the MCU-32X processor with the Caravel harness
 */

`default_nettype none

module user_project_wrapper #(
    parameter BITS = 32
) (
`ifdef USE_POWER_PINS
    inout vdda1,	// User area 1 3.3V supply
    inout vdda2,	// User area 2 3.3V supply
    inout vssa1,	// User area 1 analog ground
    inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    inout vssd2,	// User area 2 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // Analog (direct connection to GPIO pad---use with caution)
    // Note that analog I/O is not available on the 7 lowest-numbered
    // GPIO pads, and so the analog_io indexing is offset from the
    // GPIO indexing by 7 (also upper 2 GPIOs are not available).
    inout [`MPRJ_IO_PADS-10:0] analog_io,

    // Independent clock (on independent integer divider)
    input   user_clock2,

    // User maskable interrupt signals
    output [2:0] user_irq
);

    // Internal signals for MCU-32X
    wire mcu_clk;
    wire mcu_reset;
    wire [7:0] result_low;
    wire [7:0] address_low;
    wire mem_read;
    wire mem_write;
    
    // Clock and reset from Caravel
    assign mcu_clk = wb_clk_i;
    assign mcu_reset = wb_rst_i;
    
    // Instantiate MCU-32X
    MCU32X mcu32x_core (
        .clk(mcu_clk),
        .reset(mcu_reset),
        .result_low(result_low),
        .address_low(address_low),
        .mem_read(mem_read),
        .mem_write(mem_write)
    );

    // Connect MCU-32X outputs to GPIO pins
    assign io_out[7:0] = result_low;     // GPIO[7:0] = ALU result
    assign io_out[15:8] = address_low;   // GPIO[15:8] = Address
    assign io_out[16] = mem_read;        // GPIO[16] = Memory read
    assign io_out[17] = mem_write;       // GPIO[17] = Memory write
    
    // Set remaining GPIOs as outputs (driven low)
    assign io_out[`MPRJ_IO_PADS-1:18] = 0;
    
    // All GPIOs configured as outputs
    assign io_oeb = 0;

    // Wishbone interface (simple read-only for monitoring)
    reg [31:0] wbs_dat_reg;
    reg wbs_ack_reg;
    
    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            wbs_dat_reg <= 0;
            wbs_ack_reg <= 0;
        end else begin
            wbs_ack_reg <= wbs_stb_i && wbs_cyc_i;
            if (wbs_stb_i && wbs_cyc_i && !wbs_we_i) begin
                case (wbs_adr_i[7:0])
                    8'h00: wbs_dat_reg <= {24'h0, result_low};
                    8'h04: wbs_dat_reg <= {24'h0, address_low};
                    8'h08: wbs_dat_reg <= {30'h0, mem_write, mem_read};
                    default: wbs_dat_reg <= 32'h0;
                endcase
            end
        end
    end
    
    assign wbs_dat_o = wbs_dat_reg;
    assign wbs_ack_o = wbs_ack_reg;

    // Logic Analyzer connections (for debugging)
    assign la_data_out[7:0] = result_low;
    assign la_data_out[15:8] = address_low;
    assign la_data_out[16] = mem_read;
    assign la_data_out[17] = mem_write;
    assign la_data_out[127:18] = 0;

    // No user interrupts for now
    assign user_irq = 3'b0;

endmodule

`default_nettype wire
