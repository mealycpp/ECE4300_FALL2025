`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// montgomery_mul.v
// Radix-2 bit-serial Montgomery modular multiplier
//
// Computes: result = A * B * R^{-1} mod N, where R = 2^N_BITS.
//
// NOTE: This is a *Montgomery* product, not plain (A*B mod N).
// -----------------------------------------------------------------------------
module montgomery_mul #(
    parameter integer N_BITS = 2048          // must be >= 32, multiple of 32
)(
    input  wire                    clk,
    input  wire                    rst,      // synchronous, active high
    input  wire                    start,    // 1-cycle pulse

    input  wire [N_BITS-1:0]       a_in,     // operand A
    input  wire [N_BITS-1:0]       b_in,     // operand B
    input  wire [N_BITS-1:0]       n_in,     // modulus N (odd, N < R)
    input  wire [31:0]             n_prime,  // unused in this radix-2 core

    output reg  [N_BITS-1:0]       result,   // Montgomery product
    output reg                     done,     // 1-cycle pulse when result valid

    // optional debug outputs
    output reg  [2:0]              dbg_state,
    output reg  [$clog2(N_BITS):0] dbg_bit_idx
);

    // FSM states
    localparam [2:0]
        S_IDLE      = 3'd0,
        S_LOAD      = 3'd1,
        S_ADD_A     = 3'd2,
        S_ADD_N     = 3'd3,
        S_SHIFT     = 3'd4,
        S_FINAL_SUB = 3'd5,
        S_DONE      = 3'd6;

    reg [2:0]               state, next_state;

    // Internals
    reg [N_BITS:0]          T;       // accumulator (one extra bit)
    reg [N_BITS-1:0]        a_reg;
    reg [N_BITS-1:0]        b_reg;
    reg [N_BITS-1:0]        n_reg;
    reg [$clog2(N_BITS):0]  bit_idx;

    // convenience
    wire                    b_bit = b_reg[bit_idx];
    wire [N_BITS:0]         a_ext = {1'b0, a_reg};
    wire [N_BITS:0]         n_ext = {1'b0, n_reg};

    // -------------------------------------------------------------------------
    // Sequential logic
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            done        <= 1'b0;
            T           <= {(N_BITS+1){1'b0}};
            a_reg       <= {N_BITS{1'b0}};
            b_reg       <= {N_BITS{1'b0}};
            n_reg       <= {N_BITS{1'b0}};
            bit_idx     <= {($clog2(N_BITS)+1){1'b0}};
            result      <= {N_BITS{1'b0}};
            dbg_state   <= S_IDLE;
            dbg_bit_idx <= {($clog2(N_BITS)+1){1'b0}};
        end else begin
            state       <= next_state;
            done        <= 1'b0;        // default: only assert in S_DONE
            dbg_state   <= next_state;
            dbg_bit_idx <= bit_idx;

            case (state)
                S_IDLE: begin
                    // wait for start, nothing to do
                end

                S_LOAD: begin
                    a_reg   <= a_in;
                    b_reg   <= b_in;
                    n_reg   <= n_in;
                    T       <= {(N_BITS+1){1'b0}};
                    bit_idx <= {($clog2(N_BITS)+1){1'b0}}; // 0
                end

                S_ADD_A: begin
                    if (b_bit)
                        T <= T + a_ext;
                end

                S_ADD_N: begin
                    if (T[0])
                        T <= T + n_ext;
                end

                S_SHIFT: begin
                    T       <= {1'b0, T[N_BITS:1]};   // divide by 2
                    bit_idx <= bit_idx + 1'b1;
                end

                S_FINAL_SUB: begin
                    // conditional subtract if T >= N
                    if (T[N_BITS-1:0] >= n_reg)
                        T <= {1'b0, T[N_BITS-1:0] - n_reg};
                end

                S_DONE: begin
                    result <= T[N_BITS-1:0];
                    done   <= 1'b1;   // 1-cycle pulse
                end

                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Next-state logic
    // -------------------------------------------------------------------------
    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: begin
                if (start)
                    next_state = S_LOAD;
            end

            S_LOAD:      next_state = S_ADD_A;
            S_ADD_A:     next_state = S_ADD_N;
            S_ADD_N:     next_state = S_SHIFT;

            S_SHIFT: begin
                if (bit_idx == (N_BITS-1))
                    next_state = S_FINAL_SUB;
                else
                    next_state = S_ADD_A;
            end

            S_FINAL_SUB: next_state = S_DONE;

            S_DONE: begin
                // wait for start to drop before going back to IDLE
                if (!start)
                    next_state = S_IDLE;
                else
                    next_state = S_DONE;
            end

            default:      next_state = S_IDLE;
        endcase
    end

endmodule
