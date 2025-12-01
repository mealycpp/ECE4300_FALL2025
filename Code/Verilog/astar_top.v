`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2025 10:05:22 AM
// Design Name: 
// Module Name: astar_top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module astar_top #(
    parameter GRID_SIZE = 16,
    parameter COORD_BITS = 4,
    parameter MAX_NODES = 256,
    parameter MAX_CYCLES = 100000
)(
    input wire clk,
    input wire rst,
    input wire start,
    input wire [COORD_BITS-1:0] start_x,
    input wire [COORD_BITS-1:0] start_y,
    input wire [COORD_BITS-1:0] goal_x,
    input wire [COORD_BITS-1:0] goal_y,
    input wire [GRID_SIZE*GRID_SIZE-1:0] obstacle_map,
    output reg done,
    output reg path_found,
    output reg [7:0] path_length,
    output reg [31:0] cycles_taken,
    output reg [GRID_SIZE*GRID_SIZE-1:0] path_map,
    output reg timeout_error,
    output reg [15:0] nodes_expanded,  // NEW: For Python comparison
    output reg [15:0] path_cost        // NEW: For Python comparison
);

    // State machine
    localparam IDLE = 4'd0;
    localparam INIT = 4'd1;
    localparam INIT_LOOP = 4'd2;
    localparam FIND_MIN = 4'd3;
    localparam FIND_MIN_LOOP = 4'd4;
    localparam EXPAND = 4'd5;
    localparam CHECK_NEIGHBOR = 4'd6;
    localparam UPDATE_NEIGHBOR = 4'd7;
    localparam RECONSTRUCT = 4'd8;
    localparam DONE_STATE = 4'd10;

    reg [3:0] state;
    reg [31:0] cycle_counter;
    reg [8:0] loop_counter;

    // Node arrays
    reg [7:0] g_score [0:MAX_NODES-1];
    reg [7:0] f_score [0:MAX_NODES-1];
    reg [MAX_NODES-1:0] open_set;
    reg [MAX_NODES-1:0] closed_set;
    reg [7:0] parent [0:MAX_NODES-1];

    // Current processing
    reg [7:0] current_node;
    reg [COORD_BITS-1:0] current_x, current_y;
    reg [7:0] min_f_score;
    reg [7:0] min_node;
    reg [3:0] neighbor_idx;
    reg [COORD_BITS-1:0] neighbor_x, neighbor_y;
    reg [7:0] neighbor_node;
    reg [7:0] tentative_g;
    reg neighbor_valid;
    reg [7:0] move_cost;
    
    reg [7:0] start_node;
    reg [7:0] goal_node;
    reg [7:0] recon_node;
    reg [8:0] recon_counter;  // CHANGED: 9 bits for longer paths

    // Node index calculation
    function [7:0] calc_node_index;
        input [COORD_BITS-1:0] x;
        input [COORD_BITS-1:0] y;
        begin
            calc_node_index = (y << COORD_BITS) + x;
        end
    endfunction

    // FIXED: Manhattan distance scaled by 10 to match move costs
    function [7:0] manhattan;
        input [COORD_BITS-1:0] x1, y1, x2, y2;
        reg signed [COORD_BITS:0] dx, dy;
        begin
            dx = x1 - x2;
            dy = y1 - y2;
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            manhattan = (dx + dy) * 10;  // MULTIPLY BY 10!
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            done <= 0;
            path_found <= 0;
            path_length <= 0;
            cycles_taken <= 0;
            cycle_counter <= 0;
            open_set <= 0;
            closed_set <= 0;
            path_map <= 0;
            loop_counter <= 0;
            timeout_error <= 0;
            nodes_expanded <= 0;  // NEW: Initialize
            path_cost <= 0;       // NEW: Initialize
        end else begin
            // Global timeout check
            if (cycle_counter >= MAX_CYCLES && state != IDLE && state != DONE_STATE) begin
                state <= DONE_STATE;
                path_found <= 0;
                timeout_error <= 1;
            end else begin
                case (state)
                    IDLE: begin
                        done <= 0;
                        timeout_error <= 0;
                        if (start) begin
                            state <= INIT;
                            cycle_counter <= 0;
                            path_found <= 0;
                            path_map <= 0;
                            path_length <= 0;
                            loop_counter <= 0;
                            start_node <= calc_node_index(start_x, start_y);
                            goal_node <= calc_node_index(goal_x, goal_y);
                        end
                    end

                    INIT: begin
                        cycle_counter <= cycle_counter + 1;
                        loop_counter <= 0;
                        state <= INIT_LOOP;
                    end

                    INIT_LOOP: begin
                        cycle_counter <= cycle_counter + 1;
                        if (loop_counter < MAX_NODES) begin
                            g_score[loop_counter] <= 8'd255;
                            f_score[loop_counter] <= 8'd255;
                            parent[loop_counter] <= 8'd255;
                            loop_counter <= loop_counter + 1;
                        end else begin
                            g_score[start_node] <= 0;
                            f_score[start_node] <= manhattan(start_x, start_y, goal_x, goal_y);
                            open_set <= 0;
                            open_set[start_node] <= 1;
                            closed_set <= 0;
                            state <= FIND_MIN;
                        end
                    end

                    FIND_MIN: begin
                        cycle_counter <= cycle_counter + 1;
                        
                        if (open_set == 0) begin
                            state <= DONE_STATE;
                            path_found <= 0;
                        end else begin
                            min_f_score <= 8'd255;
                            min_node <= 0;
                            loop_counter <= 0;
                            state <= FIND_MIN_LOOP;
                        end
                    end

                    FIND_MIN_LOOP: begin
                        cycle_counter <= cycle_counter + 1;
                        
                        if (loop_counter < MAX_NODES) begin
                            if (open_set[loop_counter] && (f_score[loop_counter] < min_f_score)) begin
                                min_f_score <= f_score[loop_counter];
                                min_node <= loop_counter[7:0];
                            end
                            loop_counter <= loop_counter + 1;
                        end else begin
                            current_node <= min_node;
                            current_x <= min_node[COORD_BITS-1:0];
                            current_y <= min_node[7:COORD_BITS];
                            state <= EXPAND;
                        end
                    end

                    EXPAND: begin
                        cycle_counter <= cycle_counter + 1;
                        
                        // Count every node we expand (including goal)
                        nodes_expanded <= nodes_expanded + 1;
                        
                        if (current_node == goal_node) begin
                            path_found <= 1;
                            recon_node <= current_node;
                            path_length <= 0;
                            recon_counter <= 0;
                            path_cost <= g_score[current_node];  // Save final path cost
                            state <= RECONSTRUCT;
                        end else begin
                            open_set[current_node] <= 0;
                            closed_set[current_node] <= 1;
                            neighbor_idx <= 0;
                            state <= CHECK_NEIGHBOR;
                        end
                    end

                    CHECK_NEIGHBOR: begin
                        cycle_counter <= cycle_counter + 1;
                        
                        if (neighbor_idx < 8) begin  // 8 neighbors
                            neighbor_valid <= 1;
                            move_cost <= 8'd10;
                            
                            case (neighbor_idx)
                                4'd0: begin // Up
                                    neighbor_x <= current_x;
                                    if (current_y > 0) begin
                                        neighbor_y <= current_y - 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd1: begin // Down
                                    neighbor_x <= current_x;
                                    if (current_y < GRID_SIZE-1) begin
                                        neighbor_y <= current_y + 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd2: begin // Left
                                    neighbor_y <= current_y;
                                    if (current_x > 0) begin
                                        neighbor_x <= current_x - 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd3: begin // Right
                                    neighbor_y <= current_y;
                                    if (current_x < GRID_SIZE-1) begin
                                        neighbor_x <= current_x + 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd4: begin // Up-Left
                                    move_cost <= 8'd14;
                                    if (current_y > 0 && current_x > 0) begin
                                        neighbor_x <= current_x - 1;
                                        neighbor_y <= current_y - 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd5: begin // Up-Right
                                    move_cost <= 8'd14;
                                    if (current_y > 0 && current_x < GRID_SIZE-1) begin
                                        neighbor_x <= current_x + 1;
                                        neighbor_y <= current_y - 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd6: begin // Down-Left
                                    move_cost <= 8'd14;
                                    if (current_y < GRID_SIZE-1 && current_x > 0) begin
                                        neighbor_x <= current_x - 1;
                                        neighbor_y <= current_y + 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                                4'd7: begin // Down-Right
                                    move_cost <= 8'd14;
                                    if (current_y < GRID_SIZE-1 && current_x < GRID_SIZE-1) begin
                                        neighbor_x <= current_x + 1;
                                        neighbor_y <= current_y + 1;
                                    end else begin
                                        neighbor_valid <= 0;
                                    end
                                end
                            endcase
                            state <= UPDATE_NEIGHBOR;
                        end else begin
                            state <= FIND_MIN;
                        end
                    end

                    UPDATE_NEIGHBOR: begin
                        cycle_counter <= cycle_counter + 1;
                        neighbor_node <= calc_node_index(neighbor_x, neighbor_y);
                        
                        if (neighbor_valid && 
                            !obstacle_map[calc_node_index(neighbor_x, neighbor_y)] && 
                            !closed_set[calc_node_index(neighbor_x, neighbor_y)]) begin
                            
                            tentative_g <= g_score[current_node] + move_cost;
                            
                            if ((g_score[calc_node_index(neighbor_x, neighbor_y)] == 8'd255) ||
                                (g_score[current_node] + move_cost < g_score[calc_node_index(neighbor_x, neighbor_y)])) begin
                                
                                parent[calc_node_index(neighbor_x, neighbor_y)] <= current_node;
                                g_score[calc_node_index(neighbor_x, neighbor_y)] <= g_score[current_node] + move_cost;
                                f_score[calc_node_index(neighbor_x, neighbor_y)] <= 
                                    g_score[current_node] + move_cost + manhattan(neighbor_x, neighbor_y, goal_x, goal_y);
                                open_set[calc_node_index(neighbor_x, neighbor_y)] <= 1;
                            end
                        end
                        
                        neighbor_idx <= neighbor_idx + 1;
                        state <= CHECK_NEIGHBOR;
                    end

                    RECONSTRUCT: begin
                        cycle_counter <= cycle_counter + 1;
                        
                        // FIXED: Better counter limit and clearer logic
                        if (recon_counter >= 9'd400) begin
                            state <= DONE_STATE;
                        end else begin
                            path_map[recon_node] <= 1;
                            path_length <= path_length + 1;
                            recon_counter <= recon_counter + 1;
                            
                            // FIXED: Check start first
                            if (recon_node == start_node) begin
                                state <= DONE_STATE;
                            end else if (parent[recon_node] == 8'd255) begin
                                state <= DONE_STATE;
                            end else begin
                                recon_node <= parent[recon_node];
                            end
                        end
                    end

                    DONE_STATE: begin
                        done <= 1;
                        cycles_taken <= cycle_counter;
                        state <= IDLE;
                    end

                    default: state <= IDLE;
                endcase
            end
        end
    end

endmodule