`timescale 1ns / 1ps
// -----------------------------------------------------------------------------
// montgomery_axi.v
// AXI4-Lite wrapper for montgomery_mul
// -----------------------------------------------------------------------------
module montgomery_axi #
(
    parameter integer N_BITS               = 2048,
    parameter integer C_S_AXI_DATA_WIDTH   = 32,
    parameter integer C_S_AXI_ADDR_WIDTH   = 12
)
(
    input  wire                             s_axi_aclk,
    input  wire                             s_axi_aresetn,

    // write address
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    s_axi_awaddr,
    input  wire                             s_axi_awvalid,
    output reg                              s_axi_awready,

    // write data
    input  wire [C_S_AXI_DATA_WIDTH-1:0]    s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                             s_axi_wvalid,
    output reg                              s_axi_wready,

    // write response
    output reg  [1:0]                       s_axi_bresp,
    output reg                              s_axi_bvalid,
    input  wire                             s_axi_bready,

    // read address
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]    s_axi_araddr,
    input  wire                             s_axi_arvalid,
    output reg                              s_axi_arready,

    // read data
    output reg [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output reg [1:0]                        s_axi_rresp,
    output reg                              s_axi_rvalid,
    input  wire                             s_axi_rready
);

    // -------------------------------------------------------------------------
    // Local params / address map
    // -------------------------------------------------------------------------
    localparam integer AXI_NWORDS = N_BITS / 32;

    localparam [C_S_AXI_ADDR_WIDTH-1:0] BASE_A       = 12'h000;   // 0x0
    localparam [C_S_AXI_ADDR_WIDTH-1:0] BASE_B       = 12'h200;   // 0x200
    localparam [C_S_AXI_ADDR_WIDTH-1:0] BASE_N       = 12'h400;   // 0x400
    localparam [C_S_AXI_ADDR_WIDTH-1:0] BASE_RES     = 12'h600;   // 0x600
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_NPRIME  = 12'h800;   // 0x800
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_CONTROL = 12'h804;   // 0x804
    localparam [C_S_AXI_ADDR_WIDTH-1:0] ADDR_STATUS  = 12'h808;   // 0x808

    localparam integer IDX_BASE_A   = BASE_A   / 4;
    localparam integer IDX_BASE_B   = BASE_B   / 4;
    localparam integer IDX_BASE_N   = BASE_N   / 4;
    localparam integer IDX_BASE_RES = BASE_RES / 4;

    // -------------------------------------------------------------------------
    // Internal registers / memories
    // -------------------------------------------------------------------------
    reg [31:0] a_mem [0:AXI_NWORDS-1];
    reg [31:0] b_mem [0:AXI_NWORDS-1];
    reg [31:0] n_mem [0:AXI_NWORDS-1];
    reg [31:0] y_mem [0:AXI_NWORDS-1];

    reg [31:0] n_prime_reg;
    reg        start_reg;   // level: 1 from CONTROL write until core_done
    reg        done_reg;    // sticky done

    // Flatten for core
    wire [N_BITS-1:0] a_vec;
    wire [N_BITS-1:0] b_vec;
    wire [N_BITS-1:0] n_vec;
    wire [N_BITS-1:0] y_vec;
    wire              core_done;

    genvar gi;
    generate
        for (gi = 0; gi < AXI_NWORDS; gi = gi + 1) begin : FLATTEN
            assign a_vec[32*gi +: 32] = a_mem[gi];
            assign b_vec[32*gi +: 32] = b_mem[gi];
            assign n_vec[32*gi +: 32] = n_mem[gi];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // AXI write handshake (independent AW/W channels)
    // -------------------------------------------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1:0] awaddr_reg;
    wire                         aw_hs = s_axi_awvalid && s_axi_awready;
    wire                         w_hs  = s_axi_wvalid  && s_axi_wready;
    wire                         wr_en = aw_hs && w_hs;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_awready <= 1'b0;
            s_axi_wready  <= 1'b0;
            awaddr_reg    <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin
            // AW channel
            if (~s_axi_awready && s_axi_awvalid) begin
                s_axi_awready <= 1'b1;
                awaddr_reg    <= s_axi_awaddr;
            end else begin
                s_axi_awready <= 1'b0;
            end

            // W channel
            if (~s_axi_wready && s_axi_wvalid) begin
                s_axi_wready <= 1'b1;
            end else begin
                s_axi_wready <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // AXI write logic
    // -------------------------------------------------------------------------
    integer i;
    integer widx;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            n_prime_reg <= 32'd0;
            start_reg   <= 1'b0;
            done_reg    <= 1'b0;
            for (i = 0; i < AXI_NWORDS; i = i + 1) begin
                a_mem[i] <= 32'd0;
                b_mem[i] <= 32'd0;
                n_mem[i] <= 32'd0;
                y_mem[i] <= 32'd0;
            end
        end else begin
            if (wr_en) begin
                widx = awaddr_reg[11:2];

                // A
                if ((widx >= IDX_BASE_A) &&
                    (widx < IDX_BASE_A + AXI_NWORDS)) begin
                    for (i = 0; i < 4; i = i + 1) begin
                        if (s_axi_wstrb[i])
                            a_mem[widx - IDX_BASE_A][8*i +: 8] <= s_axi_wdata[8*i +: 8];
                    end
                end
                // B
                else if ((widx >= IDX_BASE_B) &&
                         (widx < IDX_BASE_B + AXI_NWORDS)) begin
                    for (i = 0; i < 4; i = i + 1) begin
                        if (s_axi_wstrb[i])
                            b_mem[widx - IDX_BASE_B][8*i +: 8] <= s_axi_wdata[8*i +: 8];
                    end
                end
                // N
                else if ((widx >= IDX_BASE_N) &&
                         (widx < IDX_BASE_N + AXI_NWORDS)) begin
                    for (i = 0; i < 4; i = i + 1) begin
                        if (s_axi_wstrb[i])
                            n_mem[widx - IDX_BASE_N][8*i +: 8] <= s_axi_wdata[8*i +: 8];
                    end
                end
                // n_prime
                else if (awaddr_reg[11:0] == ADDR_NPRIME) begin
                    for (i = 0; i < 4; i = i + 1) begin
                        if (s_axi_wstrb[i])
                            n_prime_reg[8*i +: 8] <= s_axi_wdata[8*i +: 8];
                    end
                end
                // CONTROL
                else if (awaddr_reg[11:0] == ADDR_CONTROL) begin
                    // bit 0: start pulse (write 1)
                    if (s_axi_wdata[0]) begin
                        start_reg <= 1'b1;
                        done_reg  <= 1'b0;
                    end
                end
                // STATUS and result are read-only
            end

            // latch core result when done
            if (core_done) begin
                done_reg  <= 1'b1;
                start_reg <= 1'b0; // let core return to IDLE for next op
                for (i = 0; i < AXI_NWORDS; i = i + 1) begin
                    y_mem[i] <= y_vec[32*i +: 32];
                end
            end
        end
    end

    // write response
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= 2'b00;
        end else begin
            if (wr_en && ~s_axi_bvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // AXI read channel
    // -------------------------------------------------------------------------
    reg [C_S_AXI_ADDR_WIDTH-1:0] araddr_reg;
    wire                         rd_en = s_axi_arvalid && s_axi_arready && ~s_axi_rvalid;

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_arready <= 1'b0;
            araddr_reg    <= {C_S_AXI_ADDR_WIDTH{1'b0}};
        end else begin
            if (~s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
                araddr_reg    <= s_axi_araddr;
            end else begin
                s_axi_arready <= 1'b0;
            end
        end
    end

    integer ridx;
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            s_axi_rvalid <= 1'b0;
            s_axi_rresp  <= 2'b00;
            s_axi_rdata  <= {C_S_AXI_DATA_WIDTH{1'b0}};
        end else begin
            if (rd_en) begin
                ridx = araddr_reg[11:2];

                // A
                if ((ridx >= IDX_BASE_A) &&
                    (ridx < IDX_BASE_A + AXI_NWORDS)) begin
                    s_axi_rdata <= a_mem[ridx - IDX_BASE_A];
                end
                // B
                else if ((ridx >= IDX_BASE_B) &&
                         (ridx < IDX_BASE_B + AXI_NWORDS)) begin
                    s_axi_rdata <= b_mem[ridx - IDX_BASE_B];
                end
                // N
                else if ((ridx >= IDX_BASE_N) &&
                         (ridx < IDX_BASE_N + AXI_NWORDS)) begin
                    s_axi_rdata <= n_mem[ridx - IDX_BASE_N];
                end
                // n_prime
                else if (araddr_reg[11:0] == ADDR_NPRIME) begin
                    s_axi_rdata <= n_prime_reg;
                end
                // CONTROL (read as 0)
                else if (araddr_reg[11:0] == ADDR_CONTROL) begin
                    s_axi_rdata <= 32'd0;
                end
                // STATUS
                else if (araddr_reg[11:0] == ADDR_STATUS) begin
                    s_axi_rdata <= {31'd0, done_reg};
                end
                // RESULT
                else if ((ridx >= IDX_BASE_RES) &&
                         (ridx < IDX_BASE_RES + AXI_NWORDS)) begin
                    s_axi_rdata <= y_mem[ridx - IDX_BASE_RES];
                end

                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Core instance
    // -------------------------------------------------------------------------
    montgomery_mul #(
        .N_BITS (N_BITS)
    ) u_montgomery_mul (
        .clk     (s_axi_aclk),
        .rst     (~s_axi_aresetn),
        .start   (start_reg),
        .a_in    (a_vec),
        .b_in    (b_vec),
        .n_in    (n_vec),
        .n_prime (n_prime_reg),
        .result  (y_vec),
        .done    (core_done),
        .dbg_state(),
        .dbg_bit_idx()
    );


endmodule
