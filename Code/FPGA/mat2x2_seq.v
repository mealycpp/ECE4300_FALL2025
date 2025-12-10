`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// 2x2 sequential matrix multiply (naive)
// C = A * B, where A,B,C are 2x2.
// Each run performs 8 multiplies and 4 adds over several cycles.
// -----------------------------------------------------------------------------

module mat2x2_seq #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire                     clk,
    input  wire                     rst,     // synchronous, active-high
    input  wire                     start,   // one-cycle pulse when idle
    input  wire signed [DATA_W-1:0] a11, a12, a21, a22,
    input  wire signed [DATA_W-1:0] b11, b12, b21, b22,
    output reg  signed [ACC_W-1:0]  c11, c12, c21, c22,
    output reg                      busy,
    output reg                      done     // 1-cycle pulse
);

    localparam S_IDLE = 4'd0;
    localparam S_0    = 4'd1;
    localparam S_1    = 4'd2;
    localparam S_2    = 4'd3;
    localparam S_3    = 4'd4;
    localparam S_4    = 4'd5;
    localparam S_5    = 4'd6;
    localparam S_6    = 4'd7;
    localparam S_7    = 4'd8;
    localparam S_DONE = 4'd9;

    reg [3:0] state;

    reg signed [DATA_W-1:0] A11, A12, A21, A22;
    reg signed [DATA_W-1:0] B11, B12, B21, B22;

    reg signed [ACC_W-1:0] acc;

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            acc   <= {ACC_W{1'b0}};
            c11   <= {ACC_W{1'b0}};
            c12   <= {ACC_W{1'b0}};
            c21   <= {ACC_W{1'b0}};
            c22   <= {ACC_W{1'b0}};
        end else begin
            done <= 1'b0;

            case (state)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        A11 <= a11; A12 <= a12;
                        A21 <= a21; A22 <= a22;
                        B11 <= b11; B12 <= b12;
                        B21 <= b21; B22 <= b22;
                        acc   <= {ACC_W{1'b0}};
                        busy  <= 1'b1;
                        state <= S_0;
                    end
                end

                // c11 = A11*B11 + A12*B21
                S_0: begin
                    acc   <= A11 * B11;
                    state <= S_1;
                end

                S_1: begin
                    c11   <= acc + A12 * B21;
                    state <= S_2;
                end

                // c12 = A11*B12 + A12*B22
                S_2: begin
                    acc   <= A11 * B12;
                    state <= S_3;
                end

                S_3: begin
                    c12   <= acc + A12 * B22;
                    state <= S_4;
                end

                // c21 = A21*B11 + A22*B21
                S_4: begin
                    acc   <= A21 * B11;
                    state <= S_5;
                end

                S_5: begin
                    c21   <= acc + A22 * B21;
                    state <= S_6;
                end

                // c22 = A21*B12 + A22*B22
                S_6: begin
                    acc   <= A21 * B12;
                    state <= S_7;
                end

                S_7: begin
                    c22   <= acc + A22 * B22;
                    state <= S_DONE;
                end

                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

