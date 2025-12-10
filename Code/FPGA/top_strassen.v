`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Top-level for Nexys A7-100T
//   CLK : 100 MHz clock (E3)
//   RST : reset (btnC, active high)
//   SW  : start switch (J15, sw[0])
//   LED[15:0] : basic status (LED[0] = done, others = cycle_count bits)
//   SEG[6:0], AN[7:0] : 7-seg display (cycle_count[31:0] in hex)
// -----------------------------------------------------------------------------
module top_strassen (
    input  wire       CLK,
    input  wire       RST,
    input  wire       SW,
    output wire [15:0] LED,
    output wire [6:0] SEG,
    output wire [7:0] AN
);

    // edge detector for SW to generate a one-cycle start pulse
    reg sw_d0, sw_d1;
    always @(posedge CLK) begin
        if (RST) begin
            sw_d0 <= 1'b0;
            sw_d1 <= 1'b0;
        end else begin
            sw_d0 <= SW;
            sw_d1 <= sw_d0;
        end
    end

    wire start_pulse = sw_d0 & ~sw_d1;

    wire        core_done;
    wire [63:0] cycle_count;
    wire signed [31:0] C00, C01, C02, C03,
                       C10, C11, C12, C13,
                       C20, C21, C22, C23,
                       C30, C31, C32, C33;

    // 4x4 Strassen core
    mat4x4_strassen #(
        .DATA_W(16),
        .ACC_W (32)
    ) u_core (
        .clk (CLK),
        .rst (RST),
        .start(start_pulse),
        .done (core_done),
        .cycle_count(cycle_count),
        .C00(C00), .C01(C01), .C02(C02), .C03(C03),
        .C10(C10), .C11(C11), .C12(C12), .C13(C13),
        .C20(C20), .C21(C21), .C22(C22), .C23(C23),
        .C30(C30), .C31(C31), .C32(C32), .C33(C33)
    );

    // LEDs: done + lower 15 bits of cycle_count
    assign LED[0]  = core_done;
    assign LED[15:1] = cycle_count[15:1];

    // 7-seg shows lower 32 bits of cycle_count
    sevenseg8_hex_driver u_disp (
        .clk  (CLK),
        .rst  (RST),
        .value(cycle_count[31:0]),
        .seg  (SEG),
        .an   (AN)
    );

endmodule

