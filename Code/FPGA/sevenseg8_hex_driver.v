`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// 7-segment display driver for 8 digits (active-low AN, active-low SEG).
// Displays 8 hex nibbles of 'value'.
// -----------------------------------------------------------------------------

module sevenseg8_hex_driver (
    input  wire       clk,      // 100 MHz
    input  wire       rst,
    input  wire [31:0] value,
    output wire [6:0] seg,
    output wire [7:0] an
);

    reg [15:0] refresh_cnt;
    reg  [2:0] digit_sel;
    reg  [3:0] cur_nibble;
    reg  [7:0] an_reg;

    // Refresh clock divider
    always @(posedge clk) begin
        if (rst) begin
            refresh_cnt <= 16'd0;
            digit_sel   <= 3'd0;
        end else begin
            refresh_cnt <= refresh_cnt + 1'b1;
            if (refresh_cnt == 16'd0)
                digit_sel <= digit_sel + 1'b1;
        end
    end

    // pick nibble
    always @* begin
        case (digit_sel)
            3'd0: cur_nibble = value[3:0];
            3'd1: cur_nibble = value[7:4];
            3'd2: cur_nibble = value[11:8];
            3'd3: cur_nibble = value[15:12];
            3'd4: cur_nibble = value[19:16];
            3'd5: cur_nibble = value[23:20];
            3'd6: cur_nibble = value[27:24];
            3'd7: cur_nibble = value[31:28];
            default: cur_nibble = 4'h0;
        endcase
    end

    // active-low anode selection
    always @* begin
        an_reg = 8'b1111_1111;
        an_reg[digit_sel] = 1'b0;  // turn on selected digit
    end

    assign an = an_reg;

    hex_to_7seg u_hex (
        .hex(cur_nibble),
        .seg(seg)
    );

endmodule

