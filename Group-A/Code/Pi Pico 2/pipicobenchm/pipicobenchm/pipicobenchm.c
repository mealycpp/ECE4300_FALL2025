#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "benchmark.h"

// =============================================================
// SHA-256 Implementation (Software based to benchmark the Core)
// =============================================================

// =============================================================
// Main Application Logic
// =============================================================

// Define Block Size (16 kB)
#define BUFFER_SIZE (16 * 1024) 
#define TEST_DURATION_SEC 3

int main() {
    stdio_init_all();

    while(!stdio_usb_connected());

    // Small delay to allow USB serial to connect
    sleep_ms(10); 

    printf("\n\n=========================================\n");
    printf("     Pico 2 RISC-V SHA-256 Benchmark      \n");
    printf("=========================================\n");


    uint32_t block_count = 0;
    
    // 2. Setup Timing
    // We want to run for 3 seconds (3,000,000 microseconds)
    uint64_t duration_us = TEST_DURATION_SEC * 1000000;
    uint64_t start_time = time_us_64();
    uint64_t end_target = start_time + duration_us;
    uint64_t current_time = start_time;

    printf("Running SHA-256 on %dkB blocks for %d seconds...\n", BUFFER_SIZE/1024, TEST_DURATION_SEC);

    // 3. Benchmark Loop
    // Loop until we exceed the target end time
    while (current_time < end_target) {
        benchmark();
        
        block_count++;
        current_time = time_us_64();
    }

    // 4. Calculate Metrics
    uint64_t actual_duration_us = current_time - start_time;
    double actual_duration_sec = (double)actual_duration_us / 1000000.0;
    
    // Total Bytes = blocks * 16384 bytes
    uint64_t total_bytes = (uint64_t)block_count * BUFFER_SIZE;
    double total_megabytes = (double)total_bytes / (1024.0 * 1024.0);
    
    // Throughput (MB/s)
    double throughput = total_megabytes / actual_duration_sec;
    
    // Latency per block (milliseconds)
    // How long did one 16KB block take on average?
    double latency_ms = (actual_duration_sec * 1000.0) / (double)block_count;


    // 5. Print Performance Output Table
    printf("\nBenchmark Results:\n");
    printf("+---------------------------+--------------------------+\n");
    printf("| %-25s | %-24s |\n", "Metric", "Value");
    printf("+---------------------------+--------------------------+\n");
    printf("| %-25s | %-24s |\n", "Block Size", "16 kB");
    printf("| %-25s | %-24d |\n", "Target Duration", TEST_DURATION_SEC);
    printf("| %-25s | %-24.4f |\n", "Actual Duration (s)", actual_duration_sec);
    printf("| %-25s | %-24lu |\n", "Total Blocks Processed", block_count);
    printf("| %-25s | %-24.2f |\n", "Total Data Processed (MB)", total_megabytes);
    printf("|---------------------------+--------------------------|\n");
    printf("| %-25s | %-24.2f |\n", "Throughput (MB/s)", throughput);
    printf("| %-25s | %-24.4f |\n", "Latency (ms/block)", latency_ms);
    printf("+---------------------------+--------------------------+\n");

    while(1) {
        tight_loop_contents();
    }
    return 0;
}