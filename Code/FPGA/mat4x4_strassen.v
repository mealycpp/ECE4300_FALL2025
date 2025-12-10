`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// 4x4 Strassen-style matrix multiply using 2x2 base multiply
// Same M1..M7 structure and C11..C22 formulas as the CPU mulDC code.
// Also exposes cycle_count and final C matrix.
// -----------------------------------------------------------------------------

module mat4x4_strassen #(
    parameter DATA_W = 16,
    parameter ACC_W  = 32
)(
    input  wire clk,
    input  wire rst,
    input  wire start,         // one-cycle pulse
    output reg  done,          // high in DONE state
    output reg  [63:0] cycle_count,  // cycles while core is busy

    // 4x4 result matrix C (for debugging / ILA)
    output reg signed [ACC_W-1:0] C00, C01, C02, C03,
                                  C10, C11, C12, C13,
                                  C20, C21, C22, C23,
                                  C30, C31, C32, C33
);

    // 4x4 A,B matrices
    reg signed [DATA_W-1:0] A[0:3][0:3];
    reg signed [DATA_W-1:0] B[0:3][0:3];

    // 2x2 intermediates
    reg signed [DATA_W-1:0] Ares[0:1][0:1];
    reg signed [DATA_W-1:0] Bres[0:1][0:1];

    // 2x2 M1..M7 results (ACC_W for accumulated precision)
    reg signed [ACC_W-1:0] M1_00, M1_01, M1_10, M1_11;
    reg signed [ACC_W-1:0] M2_00, M2_01, M2_10, M2_11;
    reg signed [ACC_W-1:0] M3_00, M3_01, M3_10, M3_11;
    reg signed [ACC_W-1:0] M4_00, M4_01, M4_10, M4_11;
    reg signed [ACC_W-1:0] M5_00, M5_01, M5_10, M5_11;
    reg signed [ACC_W-1:0] M6_00, M6_01, M6_10, M6_11;
    reg signed [ACC_W-1:0] M7_00, M7_01, M7_10, M7_11;

    // connection to 2x2 multiplier
    reg                      sub_start;
    wire                     sub_busy;
    wire                     sub_done;
    reg  signed [DATA_W-1:0] sub_a11, sub_a12, sub_a21, sub_a22;
    reg  signed [DATA_W-1:0] sub_b11, sub_b12, sub_b21, sub_b22;
    wire signed [ACC_W-1:0]  sub_c11, sub_c12, sub_c21, sub_c22;

    mat2x2_seq #(
        .DATA_W(DATA_W),
        .ACC_W (ACC_W)
    ) u_sub (
        .clk(clk),
        .rst(rst),
        .start(sub_start),
        .a11(sub_a11), .a12(sub_a12),
        .a21(sub_a21), .a22(sub_a22),
        .b11(sub_b11), .b12(sub_b12),
        .b21(sub_b21), .b22(sub_b22),
        .c11(sub_c11), .c12(sub_c12),
        .c21(sub_c21), .c22(sub_c22),
        .busy(sub_busy),
        .done(sub_done)
    );

    // FSM states
    localparam S_IDLE     = 5'd0;
    localparam S_LOAD_AB  = 5'd1;
    localparam S_M1_PREP  = 5'd2;
    localparam S_M1_WAIT  = 5'd3;
    localparam S_M2_PREP  = 5'd4;
    localparam S_M2_WAIT  = 5'd5;
    localparam S_M3_PREP  = 5'd6;
    localparam S_M3_WAIT  = 5'd7;
    localparam S_M4_PREP  = 5'd8;
    localparam S_M4_WAIT  = 5'd9;
    localparam S_M5_PREP  = 5'd10;
    localparam S_M5_WAIT  = 5'd11;
    localparam S_M6_PREP  = 5'd12;
    localparam S_M6_WAIT  = 5'd13;
    localparam S_M7_PREP  = 5'd14;
    localparam S_M7_WAIT  = 5'd15;
    localparam S_COMB1    = 5'd16;
    localparam S_COMB2    = 5'd17;
    localparam S_COMB3    = 5'd18;
    localparam S_COMB4    = 5'd19;
    localparam S_DONE     = 5'd20;

    reg [4:0] state;
    reg       core_busy;

    integer i, j;

    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            done        <= 1'b0;
            sub_start   <= 1'b0;
            core_busy   <= 1'b0;
            cycle_count <= 64'd0;

            // Clear C outputs
            C00 <= 0; C01 <= 0; C02 <= 0; C03 <= 0;
            C10 <= 0; C11 <= 0; C12 <= 0; C13 <= 0;
            C20 <= 0; C21 <= 0; C22 <= 0; C23 <= 0;
            C30 <= 0; C31 <= 0; C32 <= 0; C33 <= 0;

            // Clear A,B
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    A[i][j] <= 0;
                    B[i][j] <= 0;
                end
            end

            // clear M1..M7
            M1_00 <= 0; M1_01 <= 0; M1_10 <= 0; M1_11 <= 0;
            M2_00 <= 0; M2_01 <= 0; M2_10 <= 0; M2_11 <= 0;
            M3_00 <= 0; M3_01 <= 0; M3_10 <= 0; M3_11 <= 0;
            M4_00 <= 0; M4_01 <= 0; M4_10 <= 0; M4_11 <= 0;
            M5_00 <= 0; M5_01 <= 0; M5_10 <= 0; M5_11 <= 0;
            M6_00 <= 0; M6_01 <= 0; M6_10 <= 0; M6_11 <= 0;
            M7_00 <= 0; M7_01 <= 0; M7_10 <= 0; M7_11 <= 0;

        end else begin
            sub_start <= 1'b0;  // default

            // count cycles while core is busy
            if (core_busy)
                cycle_count <= cycle_count + 1'b1;

            case (state)
                S_IDLE: begin
                    done      <= 1'b0;
                    core_busy <= 1'b0;
                    if (start) begin
                        cycle_count <= 64'd0;
                        core_busy   <= 1'b1;
                        state       <= S_LOAD_AB;
                    end
                end

                // Load an example A and B (you can replace with your own load)
                S_LOAD_AB: begin
                    // A 4x4
                    A[0][0] <= 1;  A[0][1] <= 2;  A[0][2] <= 3;  A[0][3] <= 4;
                    A[1][0] <= 5;  A[1][1] <= 6;  A[1][2] <= 7;  A[1][3] <= 8;
                    A[2][0] <= 2;  A[2][1] <= 0;  A[2][2] <= 1;  A[2][3] <= 3;
                    A[3][0] <= 1;  A[3][1] <= 1;  A[3][2] <= 1;  A[3][3] <= 1;

                    // B 4x4
                    B[0][0] <= 1;  B[0][1] <= 0;  B[0][2] <= 2;  B[0][3] <= 1;
                    B[1][0] <= 0;  B[1][1] <= 1;  B[1][2] <= 0;  B[1][3] <= 2;
                    B[2][0] <= 1;  B[2][1] <= 1;  B[2][2] <= 1;  B[2][3] <= 1;
                    B[3][0] <= 2;  B[3][1] <= 0;  B[3][2] <= 0;  B[3][3] <= 1;

                    state <= S_M1_PREP;
                end

                // -----------------------------------------------------------------
                // M1 = (A11 + A22) * (B11 + B22)
                // A11: rows 0-1, cols 0-1
                // A22: rows 2-3, cols 2-3
                // B11: rows 0-1, cols 0-1
                // B22: rows 2-3, cols 2-3
                // -----------------------------------------------------------------
                S_M1_PREP: begin
                    Ares[0][0] <= A[0][0] + A[2][2];
                    Ares[0][1] <= A[0][1] + A[2][3];
                    Ares[1][0] <= A[1][0] + A[3][2];
                    Ares[1][1] <= A[1][1] + A[3][3];

                    Bres[0][0] <= B[0][0] + B[2][2];
                    Bres[0][1] <= B[0][1] + B[2][3];
                    Bres[1][0] <= B[1][0] + B[3][2];
                    Bres[1][1] <= B[1][1] + B[3][3];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M1_WAIT;
                end

                S_M1_WAIT: begin
                    if (sub_done) begin
                        M1_00 <= sub_c11; M1_01 <= sub_c12;
                        M1_10 <= sub_c21; M1_11 <= sub_c22;
                        state <= S_M2_PREP;
                    end
                end

                // -----------------------------------------------------------------
                // M2 = (A21 + A22) * B11
                // A21: rows 2-3, cols 0-1
                // A22: rows 2-3, cols 2-3
                // B11: rows 0-1, cols 0-1
                // -----------------------------------------------------------------
                S_M2_PREP: begin
                    Ares[0][0] <= A[2][0] + A[2][2];
                    Ares[0][1] <= A[2][1] + A[2][3];
                    Ares[1][0] <= A[3][0] + A[3][2];
                    Ares[1][1] <= A[3][1] + A[3][3];

                    Bres[0][0] <= B[0][0];
                    Bres[0][1] <= B[0][1];
                    Bres[1][0] <= B[1][0];
                    Bres[1][1] <= B[1][1];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M2_WAIT;
                end

                S_M2_WAIT: begin
                    if (sub_done) begin
                        M2_00 <= sub_c11; M2_01 <= sub_c12;
                        M2_10 <= sub_c21; M2_11 <= sub_c22;
                        state <= S_M3_PREP;
                    end
                end

                // -----------------------------------------------------------------
                // M3 = A11 * (B12 - B22)
                // A11: rows 0-1, cols 0-1
                // B12: rows 0-1, cols 2-3
                // B22: rows 2-3, cols 2-3
                // -----------------------------------------------------------------
                S_M3_PREP: begin
                    Ares[0][0] <= A[0][0];
                    Ares[0][1] <= A[0][1];
                    Ares[1][0] <= A[1][0];
                    Ares[1][1] <= A[1][1];

                    Bres[0][0] <= B[0][2] - B[2][2];
                    Bres[0][1] <= B[0][3] - B[2][3];
                    Bres[1][0] <= B[1][2] - B[3][2];
                    Bres[1][1] <= B[1][3] - B[3][3];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M3_WAIT;
                end

                S_M3_WAIT: begin
                    if (sub_done) begin
                        M3_00 <= sub_c11; M3_01 <= sub_c12;
                        M3_10 <= sub_c21; M3_11 <= sub_c22;
                        state <= S_M4_PREP;
                    end
                end

                // -----------------------------------------------------------------
                // M4 = A22 * (B21 - B11)
                // A22: rows 2-3, cols 2-3
                // B21: rows 2-3, cols 0-1
                // B11: rows 0-1, cols 0-1
                // -----------------------------------------------------------------
                S_M4_PREP: begin
                    Ares[0][0] <= A[2][2];
                    Ares[0][1] <= A[2][3];
                    Ares[1][0] <= A[3][2];
                    Ares[1][1] <= A[3][3];

                    Bres[0][0] <= B[2][0] - B[0][0];
                    Bres[0][1] <= B[2][1] - B[0][1];
                    Bres[1][0] <= B[3][0] - B[1][0];
                    Bres[1][1] <= B[3][1] - B[1][1];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M4_WAIT;
                end

                S_M4_WAIT: begin
                    if (sub_done) begin
                        M4_00 <= sub_c11; M4_01 <= sub_c12;
                        M4_10 <= sub_c21; M4_11 <= sub_c22;
                        state <= S_M5_PREP;
                    end
                end

                // -----------------------------------------------------------------
                // M5 = (A11 + A12) * B22
                // A11: rows 0-1, cols 0-1
                // A12: rows 0-1, cols 2-3
                // B22: rows 2-3, cols 2-3
                // -----------------------------------------------------------------
                S_M5_PREP: begin
                    Ares[0][0] <= A[0][0] + A[0][2];
                    Ares[0][1] <= A[0][1] + A[0][3];
                    Ares[1][0] <= A[1][0] + A[1][2];
                    Ares[1][1] <= A[1][1] + A[1][3];

                    Bres[0][0] <= B[2][2];
                    Bres[0][1] <= B[2][3];
                    Bres[1][0] <= B[3][2];
                    Bres[1][1] <= B[3][3];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M5_WAIT;
                end

                S_M5_WAIT: begin
                    if (sub_done) begin
                        M5_00 <= sub_c11; M5_01 <= sub_c12;
                        M5_10 <= sub_c21; M5_11 <= sub_c22;
                        state <= S_M6_PREP;
                    end
                end

                // -----------------------------------------------------------------
                // M6 = (A21 - A11) * (B11 + B12)
                // A21: rows 2-3, cols 0-1
                // A11: rows 0-1, cols 0-1
                // B11: rows 0-1, cols 0-1
                // B12: rows 0-1, cols 2-3
                // -----------------------------------------------------------------
                S_M6_PREP: begin
                    Ares[0][0] <= A[2][0] - A[0][0];
                    Ares[0][1] <= A[2][1] - A[0][1];
                    Ares[1][0] <= A[3][0] - A[1][0];
                    Ares[1][1] <= A[3][1] - A[1][1];

                    Bres[0][0] <= B[0][0] + B[0][2];
                    Bres[0][1] <= B[0][1] + B[0][3];
                    Bres[1][0] <= B[1][0] + B[1][2];
                    Bres[1][1] <= B[1][1] + B[1][3];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M6_WAIT;
                end

                S_M6_WAIT: begin
                    if (sub_done) begin
                        M6_00 <= sub_c11; M6_01 <= sub_c12;
                        M6_10 <= sub_c21; M6_11 <= sub_c22;
                        state <= S_M7_PREP;
                    end
                end

                // -----------------------------------------------------------------
                // M7 = (A12 - A22) * (B21 + B22)
                // A12: rows 0-1, cols 2-3
                // A22: rows 2-3, cols 2-3
                // B21: rows 2-3, cols 0-1
                // B22: rows 2-3, cols 2-3
                // -----------------------------------------------------------------
                S_M7_PREP: begin
                    Ares[0][0] <= A[0][2] - A[2][2];
                    Ares[0][1] <= A[0][3] - A[2][3];
                    Ares[1][0] <= A[1][2] - A[3][2];
                    Ares[1][1] <= A[1][3] - A[3][3];

                    Bres[0][0] <= B[2][0] + B[2][2];
                    Bres[0][1] <= B[2][1] + B[2][3];
                    Bres[1][0] <= B[3][0] + B[3][2];
                    Bres[1][1] <= B[3][1] + B[3][3];

                    sub_a11   <= Ares[0][0]; sub_a12 <= Ares[0][1];
                    sub_a21   <= Ares[1][0]; sub_a22 <= Ares[1][1];
                    sub_b11   <= Bres[0][0]; sub_b12 <= Bres[0][1];
                    sub_b21   <= Bres[1][0]; sub_b22 <= Bres[1][1];
                    sub_start <= 1'b1;
                    state     <= S_M7_WAIT;
                end

                S_M7_WAIT: begin
                    if (sub_done) begin
                        M7_00 <= sub_c11; M7_01 <= sub_c12;
                        M7_10 <= sub_c21; M7_11 <= sub_c22;
                        state <= S_COMB1;
                    end
                end

                // -----------------------------------------------------------------
                // Combine into C blocks:
                // C11 = M1 + M4 - M5 + M7
                // C12 = M3 + M5
                // C21 = M2 + M4
                // C22 = M1 - M2 + M3 + M6
                // -----------------------------------------------------------------
                S_COMB1: begin
                    // rows 0-1, cols 0-1
                    C00 <= M1_00 + M4_00 - M5_00 + M7_00;
                    C01 <= M1_01 + M4_01 - M5_01 + M7_01;
                    C10 <= M1_10 + M4_10 - M5_10 + M7_10;
                    C11 <= M1_11 + M4_11 - M5_11 + M7_11;
                    state <= S_COMB2;
                end

                S_COMB2: begin
                    // rows 0-1, cols 2-3
                    C02 <= M3_00 + M5_00;
                    C03 <= M3_01 + M5_01;
                    C12 <= M3_10 + M5_10;
                    C13 <= M3_11 + M5_11;
                    state <= S_COMB3;
                end

                S_COMB3: begin
                    // rows 2-3, cols 0-1
                    C20 <= M2_00 + M4_00;
                    C21 <= M2_01 + M4_01;
                    C30 <= M2_10 + M4_10;
                    C31 <= M2_11 + M4_11;
                    state <= S_COMB4;
                end

                S_COMB4: begin
                    // rows 2-3, cols 2-3
                    C22 <= M1_00 - M2_00 + M3_00 + M6_00;
                    C23 <= M1_01 - M2_01 + M3_01 + M6_01;
                    C32 <= M1_10 - M2_10 + M3_10 + M6_10;
                    C33 <= M1_11 - M2_11 + M3_11 + M6_11;

                    state <= S_DONE;
                end

                S_DONE: begin
                    done      <= 1'b1;
                    core_busy <= 1'b0;
                    // stay here until reset or a new 'start' pulse
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule

