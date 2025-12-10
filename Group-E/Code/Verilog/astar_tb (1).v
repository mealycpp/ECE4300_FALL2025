`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/02/2025 10:07:11 AM
// Design Name: 
// Module Name: astar_tb
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
module astar_tb();

    parameter GRID_SIZE = 16;
    parameter COORD_BITS = 4;
    parameter CLK_PERIOD = 10;

    reg clk;
    reg rst;
    reg start;
    reg [COORD_BITS-1:0] start_x, start_y;
    reg [COORD_BITS-1:0] goal_x, goal_y;
    reg [GRID_SIZE*GRID_SIZE-1:0] obstacle_map;
    wire done;
    wire path_found;
    wire [7:0] path_length;
    wire [31:0] cycles_taken;
    wire [GRID_SIZE*GRID_SIZE-1:0] path_map;
    wire timeout_error;
    wire [15:0] nodes_expanded;
    wire [15:0] path_cost;

    // Instantiate DUT
    astar_top #(
        .GRID_SIZE(GRID_SIZE),
        .COORD_BITS(COORD_BITS)
    ) uut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .start_x(start_x),
        .start_y(start_y),
        .goal_x(goal_x),
        .goal_y(goal_y),
        .obstacle_map(obstacle_map),
        .done(done),
        .path_found(path_found),
        .path_length(path_length),
        .cycles_taken(cycles_taken),
        .path_map(path_map),
        .timeout_error(timeout_error),
        .nodes_expanded(nodes_expanded),
        .path_cost(path_cost)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // File handle
    integer data_file;
    integer i, j;
    integer test_case;
    integer timeout_counter;
    
    initial begin
        data_file = $fopen("astar_results.txt", "w");
        
        $display("=======================================================================");
        $display("VERILOG A* PATHFINDING - 8-CONNECTED");
        $display("=======================================================================");
        $display("Grid Size: 16x16");
        $display("Costs: Straight=10, Diagonal=14");
        $display("Heuristic: Manhattan * 10");
        $display("Clock: %0d MHz", 1000/CLK_PERIOD);
        $display("");
        
        rst = 1;
        start = 0;
        start_x = 0;
        start_y = 0;
        goal_x = 0;
        goal_y = 0;
        obstacle_map = 0;
        
        #(CLK_PERIOD*5);
        rst = 0;
        #(CLK_PERIOD*2);

        // Run all test cases
        run_test(1, 0, 0, 7, 7, 0);
        
        build_spiral_maze();
        run_test(2, 1, 1, 14, 14, obstacle_map);
        
        build_random_maze(42);
        run_test(3, 1, 1, 14, 14, obstacle_map);
        
        build_snake_maze();
        run_test(4, 0, 0, 15, 15, obstacle_map);
        
        build_rooms_maze();
        run_test(5, 2, 2, 13, 13, obstacle_map);
        
        build_diagonal_corridor();
        run_test(6, 0, 0, 15, 15, obstacle_map);
        
        obstacle_map = 0;
        for (i = 0; i < GRID_SIZE; i = i + 1) begin
            obstacle_map[7*GRID_SIZE + i] = 1;
        end
        run_test(7, 0, 0, 15, 15, obstacle_map);

        #(CLK_PERIOD*10);
        $display("\n=======================================================================");
        $display("ALL TEST CASES COMPLETE");
        $display("=======================================================================");
        $fclose(data_file);
        $finish;
    end

    task run_test;
        input integer tc;
        input [COORD_BITS-1:0] sx, sy, gx, gy;
        input [GRID_SIZE*GRID_SIZE-1:0] obs_map;
        begin
            $display("=======================================================================");
            $display("TEST CASE %0d", tc);
            $display("=======================================================================");
            
            rst = 1;
            #(CLK_PERIOD*5);
            rst = 0;
            #(CLK_PERIOD*5);
            
            test_case = tc;
            start_x = sx;
            start_y = sy;
            goal_x = gx;
            goal_y = gy;
            obstacle_map = obs_map;
            
            start = 1;
            #(CLK_PERIOD);
            start = 0;
            
            timeout_counter = 0;
            while (!done && timeout_counter < 30000) begin
                #(CLK_PERIOD);
                timeout_counter = timeout_counter + 1;
            end
            
            if (timeout_counter >= 30000) begin
                $display("ERROR: TIMEOUT!");
            end
            
            #(CLK_PERIOD*5);
            display_results(tc);
            export_to_file(tc);
        end
    endtask

    task display_results;
        input integer tc;
        integer path_count;
        integer obstacle_count;
        real cycles_per_node;
        real time_us;
        real time_ms;
        begin
            $display("\nTest Configuration:");
            $display("  Start Position:        (%0d, %0d)", start_x, start_y);
            $display("  Goal Position:         (%0d, %0d)", goal_x, goal_y);
            $display("  Grid Size:             16x16 (256 nodes)");
            
            obstacle_count = 0;
            for (i = 0; i < GRID_SIZE*GRID_SIZE; i = i + 1) begin
                if (obstacle_map[i]) obstacle_count = obstacle_count + 1;
            end
            $display("  Obstacles:             %0d (%.1f%%)", obstacle_count, 
                     (obstacle_count * 100.0) / 256);
            
            $display("\n[METRIC 1] Path Quality:");
            $display("  Path Found:            %s", path_found ? "YES" : "NO");
            $display("  Timeout Error:         %s", timeout_error ? "YES" : "NO");
            
            if (path_found) begin
                $display("  Path Length:           %0d nodes", path_length);
                $display("  Path Cost:             %0d", path_cost);
                
                path_count = 0;
                for (i = 0; i < GRID_SIZE*GRID_SIZE; i = i + 1) begin
                    if (path_map[i]) path_count = path_count + 1;
                end
                $display("  Path Nodes Marked:     %0d", path_count);
            end
            
            $display("\n[METRIC 2] Algorithm Efficiency:");
            $display("  Nodes Expanded:        %0d", nodes_expanded);
            
            $display("\n[METRIC 3] Computation Performance:");
            $display("  Cycles Taken:          %0d", cycles_taken);
            
            if (nodes_expanded > 0) begin
                cycles_per_node = cycles_taken * 1.0 / nodes_expanded;
                $display("  Cycles per Node:       %.2f", cycles_per_node);
            end
            
            time_us = cycles_taken * CLK_PERIOD / 1000.0;
            time_ms = time_us / 1000.0;
            $display("  Time (μs):             %.2f", time_us);
            $display("  Time (ms):             %.3f", time_ms);
            
            if (nodes_expanded > 0) begin
                $display("  Time per Node:         %.2f μs/node", time_us / nodes_expanded);
            end
            
            $display("\n=======================================================================");
            
            // Grid visualization
            $display("\nGrid Visualization:");
            $display("S=Start, G=Goal, #=Obstacle, *=Path, .=Empty\n");
            for (j = 0; j < GRID_SIZE; j = j + 1) begin
                for (i = 0; i < GRID_SIZE; i = i + 1) begin
                    if (i == start_x && j == start_y)
                        $write("S ");
                    else if (i == goal_x && j == goal_y)
                        $write("G ");
                    else if (obstacle_map[j*GRID_SIZE + i])
                        $write("# ");
                    else if (path_map[j*GRID_SIZE + i])
                        $write("* ");
                    else
                        $write(". ");
                end
                $write("\n");
            end
            $display("");
        end
    endtask

    // Maze builders
    task build_spiral_maze;
        begin
            obstacle_map = 0;
            for (i = 2; i < 14; i = i + 1) begin
                obstacle_map[3*GRID_SIZE + i] = 1;
                obstacle_map[12*GRID_SIZE + i] = 1;
                obstacle_map[i*GRID_SIZE + 3] = 1;
                obstacle_map[i*GRID_SIZE + 12] = 1;
            end
            for (i = 5; i < 11; i = i + 1) begin
                obstacle_map[6*GRID_SIZE + i] = 1;
                obstacle_map[9*GRID_SIZE + i] = 1;
            end
            for (i = 6; i < 10; i = i + 1) begin
                obstacle_map[i*GRID_SIZE + 6] = 1;
                obstacle_map[i*GRID_SIZE + 10] = 1;
            end
            obstacle_map[3*GRID_SIZE + 8] = 0;
            obstacle_map[6*GRID_SIZE + 10] = 0;
            obstacle_map[9*GRID_SIZE + 6] = 0;
        end
    endtask

    task build_random_maze;
        input integer seed;
        integer rand_val;
        begin
            obstacle_map = 0;
            rand_val = seed;
            for (i = 0; i < GRID_SIZE; i = i + 1) begin
                for (j = 0; j < GRID_SIZE; j = j + 1) begin
                    rand_val = (rand_val * 1103515245 + 12345) % (1 << 31);
                    if ((i != 1 || j != 1) && (i != 14 || j != 14)) begin
                        if ((rand_val % 100) < 30) begin
                            obstacle_map[j*GRID_SIZE + i] = 1;
                        end
                    end
                end
            end
        end
    endtask

    task build_snake_maze;
        begin
            obstacle_map = 0;
            for (j = 2; j < GRID_SIZE; j = j + 3) begin
                for (i = 0; i < GRID_SIZE-1; i = i + 1) begin
                    obstacle_map[j*GRID_SIZE + i] = 1;
                end
            end
            for (j = 4; j < GRID_SIZE; j = j + 3) begin
                for (i = 1; i < GRID_SIZE; i = i + 1) begin
                    obstacle_map[j*GRID_SIZE + i] = 1;
                end
            end
        end
    endtask

    task build_rooms_maze;
        begin
            obstacle_map = 0;
            for (i = 1; i < 6; i = i + 1) begin
                obstacle_map[1*GRID_SIZE + i] = 1;
                obstacle_map[6*GRID_SIZE + i] = 1;
                obstacle_map[i*GRID_SIZE + 1] = 1;
                obstacle_map[i*GRID_SIZE + 6] = 1;
            end
            for (i = 9; i < 14; i = i + 1) begin
                obstacle_map[1*GRID_SIZE + i] = 1;
                obstacle_map[6*GRID_SIZE + i] = 1;
                obstacle_map[i*GRID_SIZE + 9] = 1;
                obstacle_map[i*GRID_SIZE + 14] = 1;
            end
            for (i = 1; i < 6; i = i + 1) begin
                obstacle_map[9*GRID_SIZE + i] = 1;
                obstacle_map[14*GRID_SIZE + i] = 1;
                obstacle_map[i*GRID_SIZE + 1] = 1;
                obstacle_map[i*GRID_SIZE + 6] = 1;
            end
            for (i = 9; i < 14; i = i + 1) begin
                obstacle_map[9*GRID_SIZE + i] = 1;
                obstacle_map[14*GRID_SIZE + i] = 1;
                obstacle_map[i*GRID_SIZE + 9] = 1;
                obstacle_map[i*GRID_SIZE + 14] = 1;
            end
            obstacle_map[3*GRID_SIZE + 6] = 0;
            obstacle_map[3*GRID_SIZE + 9] = 0;
            obstacle_map[6*GRID_SIZE + 3] = 0;
            obstacle_map[9*GRID_SIZE + 3] = 0;
            obstacle_map[11*GRID_SIZE + 6] = 0;
            obstacle_map[11*GRID_SIZE + 9] = 0;
        end
    endtask

    task build_diagonal_corridor;
        begin
            obstacle_map = 0;
            for (i = 0; i < GRID_SIZE; i = i + 1) begin
                for (j = 0; j < GRID_SIZE; j = j + 1) begin
                    if (j > i+1 || j < i-1) begin
                        obstacle_map[j*GRID_SIZE + i] = 1;
                    end
                end
            end
        end
    endtask

    task export_to_file;
        input integer tc;
        begin
            $fwrite(data_file, "TEST_CASE %0d\n", tc);
            $fwrite(data_file, "GRID_SIZE %0d\n", GRID_SIZE);
            $fwrite(data_file, "START %0d %0d\n", start_x, start_y);
            $fwrite(data_file, "GOAL %0d %0d\n", goal_x, goal_y);
            $fwrite(data_file, "PATH_FOUND %0d\n", path_found ? 1 : 0);
            $fwrite(data_file, "PATH_LENGTH %0d\n", path_length);
            $fwrite(data_file, "CYCLES %0d\n", cycles_taken);
            $fwrite(data_file, "NODES_EXPANDED %0d\n", nodes_expanded);
            $fwrite(data_file, "PATH_COST %0d\n", path_cost);
            
            $fwrite(data_file, "OBSTACLES\n");
            for (j = 0; j < GRID_SIZE; j = j + 1) begin
                for (i = 0; i < GRID_SIZE; i = i + 1) begin
                    $fwrite(data_file, "%0d ", obstacle_map[j*GRID_SIZE + i] ? 1 : 0);
                end
                $fwrite(data_file, "\n");
            end
            
            $fwrite(data_file, "PATH\n");
            for (j = 0; j < GRID_SIZE; j = j + 1) begin
                for (i = 0; i < GRID_SIZE; i = i + 1) begin
                    $fwrite(data_file, "%0d ", path_map[j*GRID_SIZE + i] ? 1 : 0);
                end
                $fwrite(data_file, "\n");
            end
            $fwrite(data_file, "END_TEST\n\n");
        end
    endtask

    initial begin
        #(CLK_PERIOD * 1000000);
        $display("ERROR: Overall simulation timeout!");
        $fclose(data_file);
        $finish;
    end

endmodule