#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "uart.h"
#include "gpio.h"

// ---------------------------------------------------------------------------
// Murax UART base (from Murax.scala: APB @ 0xF0000000, UART offset 0x10000)
// ---------------------------------------------------------------------------
#define UART ((Uart_Reg*)(0xF0010000u))
#define GPIO_A    ((Gpio_Reg*)(0xF0000000))

#define BTN_CONTINUE_MASK (1u << 0) 
#define BTN_SKIP_MASK     (1u << 1)

static inline uint32_t gpio_read_inputs(void) {
    return GPIO_A->INPUT;   // <- adjust this line to match your gpio.h/hello_world
}

static inline int btn_continue_pressed(void) {
    // replace GPIO_A->INPUT with whatever hello_world uses to read pins
    return (GPIO_A->INPUT & BTN_CONTINUE_MASK) != 0;
}

static inline int btn_skip_pressed(void) {
    return (GPIO_A->INPUT & BTN_SKIP_MASK) != 0;
}

// ---------------------------------------------------------------------------
// UART helpers
// ---------------------------------------------------------------------------
static inline void uart_putc(char c) {
    uart_write(UART, (uint32_t)c);
}

static void print(const char* str) {
    while (*str) {
        uart_putc(*str++);
    }
}

static void println(const char* str) {
    print(str);
    uart_putc('\r');
    uart_putc('\n');
}

// ---------------------------------------------------------------------------
// Hex / decimal printing using uart_putc
// ---------------------------------------------------------------------------
static void puthex32(uint32_t h) {
    for (int i = 0; i < 8; i++) {
        int cur_digit = h >> 28;

        if (cur_digit < 10)
            uart_putc('0' + cur_digit);
        else
            uart_putc('A' - 10 + cur_digit);

        h <<= 4;
    }
}

static void puthex64(uint64_t x) {
    uint32_t low = (uint32_t)(x & 0xFFFFFFFFu);
    uint32_t high = (uint32_t)(x >> 32);

    puthex32(high);
    puthex32(low);
}

// Powers of 10 for 64-bit decimal printing
static const uint64_t p10_64[20] = {
    10000000000000000000ull, // 10^19
    1000000000000000000ull,  // 10^18
    100000000000000000ull,   // 10^17
    10000000000000000ull,    // 10^16
    1000000000000000ull,     // 10^15
    100000000000000ull,      // 10^14
    10000000000000ull,       // 10^13
    1000000000000ull,        // 10^12
    100000000000ull,         // 10^11
    10000000000ull,          // 10^10
    1000000000ull,           // 10^9
    100000000ull,            // 10^8
    10000000ull,             // 10^7
    1000000ull,              // 10^6
    100000ull,               // 10^5
    10000ull,                // 10^4
    1000ull,                 // 10^3
    100ull,                  // 10^2
    10ull,                   // 10^1
    1ull                     // 10^0
};

static void putdec64(uint64_t x) {
    int started = 0;
    for (int i = 0; i < 20; i++) {
        uint64_t p = p10_64[i];
        int digit = 0;
        while (x >= p) {
            x -= p;
            digit++;
        }
        if (digit || started || i == 19) {
            uart_putc('0' + digit);
            started = 1;
        }
    }
}

// ---------------------------------------------------------------------------
// 64-bit counters (mcycle/minstret) – RV32I-friendly
// ---------------------------------------------------------------------------
static inline uint64_t rdcycle64(void) {
    uint32_t hi, lo, hi2;
    __asm__ volatile ("rdcycleh %0" : "=r"(hi));
    __asm__ volatile ("rdcycle  %0" : "=r"(lo));
    __asm__ volatile ("rdcycleh %0" : "=r"(hi2));
    if (hi != hi2) {
        __asm__ volatile ("rdcycle %0" : "=r"(lo));
        hi = hi2;
    }
    return ((uint64_t)hi << 32) | lo;
}

static inline uint64_t rdinstret64(void) {
    uint32_t hi, lo, hi2;
    __asm__ volatile ("rdinstreth %0" : "=r"(hi));
    __asm__ volatile ("rdinstret  %0" : "=r"(lo));
    __asm__ volatile ("rdinstreth %0" : "=r"(hi2));
    if (hi != hi2) {
        __asm__ volatile ("rdinstret %0" : "=r"(lo));
        hi = hi2;
    }
    return ((uint64_t)hi << 32) | lo;
}

// ---------------------------------------------------------------------------
// Buttons (simple press+release wait with crude debounce)
// ---------------------------------------------------------------------------
static int wait_for_press_release(int (*is_pressed)(void)) {
    // wait for press
    while (!is_pressed()) {
        // spin
    }

    // debounce while pressed
    for (int stable = 0; stable < 10000; stable++) {
        if (!is_pressed()) {
            stable = 0;
        }
    }

    // wait for release
    while (is_pressed()) {
        // spin
    }

    // debounce release
    for (int stable = 0; stable < 10000; stable++) {
        if (is_pressed()) {
            stable = 0;
        }
    }

    return 1;
}

// ---------------------------------------------------------------------------
// Helper: measure empty loop overhead for a given ITER
// ---------------------------------------------------------------------------
static void measure_empty_loop(uint32_t ITER,
    uint64_t* cycles_out,
    uint64_t* insts_out) {
    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        __asm__ volatile("" ::: "memory");
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    *cycles_out = (c1 - c0);
    *insts_out = (i1 - i0);
}

// ---------------------------------------------------------------------------
// Test 1: Addition speed test (with loop overhead subtraction)
// ---------------------------------------------------------------------------
static void addition_speed_test(void) {
    const uint32_t ITER = 100000;

    volatile uint32_t a = 1, b = 2, c = 3;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        a += b + c;
        b += a;
        c += b;
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    // Use a/b/c so compiler can’t “get clever”
    __asm__ volatile("" : : "r"(a), "r"(b), "r"(c));

    print("\r\n=== Addition speed test ===\r\n");
    print("ITER: ");      putdec64(ITER);           print("\r\n");

    print("Empty loop cycles: "); putdec64(empty_cycles); print("\r\n");
    print("Empty loop insts : "); putdec64(empty_insts);  print("\r\n");

    print("Total cycles     : "); putdec64(total_cycles); print("\r\n");
    print("Total insts      : "); putdec64(total_insts);  print("\r\n");

    print("Body-only cycles : "); putdec64(body_cycles);  print("\r\n");
    print("Body-only insts  : "); putdec64(body_insts);   print("\r\n");

    print("Body cycles (hex): "); puthex64(body_cycles);  print("\r\n");
    print("Body insts  (hex): "); puthex64(body_insts);   print("\r\n");
}

// ---------------------------------------------------------------------------
// Test 2: ALU throughput test (add-heavy, multiple ops per iter)
// ---------------------------------------------------------------------------
static void alu_throughput_test(void) {
    const uint32_t ITER = 200000;

    volatile uint32_t a = 1, b = 2, c = 3, d = 4;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        a = a + b;
        b = b + c;
        c = c + d;
        d = d + a;
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(a), "r"(b), "r"(c), "r"(d));

    print("\r\n=== ALU Throughput Test ===\r\n");
    print("ITER: ");      putdec64(ITER);           print("\r\n");
    print("Body cycles : "); putdec64(body_cycles); print("\r\n");
    print("Body insts  : "); putdec64(body_insts);  print("\r\n");
}

// ---------------------------------------------------------------------------
// Test 3: Multiply latency test
// ---------------------------------------------------------------------------
static void multiply_latency_test(void) {
    const uint32_t ITER = 20000;

    volatile uint32_t x = 12345, y = 6789;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        x = x * y;   // stress MUL
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(x));

    print("\r\n=== Multiply Latency Test ===\r\n");
    print("ITER: ");      putdec64(ITER);           print("\r\n");
    print("Body cycles : "); putdec64(body_cycles); print("\r\n");
    print("Body insts  : "); putdec64(body_insts);  print("\r\n");
}

// ---------------------------------------------------------------------------
// Test 4: Divide latency test
// NOTE: can be slow if DIV is iterative
// ---------------------------------------------------------------------------
static void divide_latency_test(void) {
    const uint32_t ITER = 2000;

    volatile uint32_t x = 987654321u, y = 3u;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        x = x / y;   // stress DIV
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(x));

    print("\r\n=== Divide Latency Test ===\r\n");
    print("ITER: ");      putdec64(ITER);           print("\r\n");
    print("Body cycles : "); putdec64(body_cycles); print("\r\n");
    print("Body insts  : "); putdec64(body_insts);  print("\r\n");
}

// ---------------------------------------------------------------------------
// Test 5: Load/store speed test (memory access throughput)
// ---------------------------------------------------------------------------
static void load_store_speed_test(void) {
    const uint32_t ITER = 100000;

    static volatile uint32_t buf[256];
    volatile uint32_t acc = 0;
    uint32_t idx = 0;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        acc += buf[idx];          // load
        buf[idx] = acc;           // store
        idx = (idx + 1u) & 0xFFu; // keep in range [0,255]
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    // Use acc so compiler keeps the operations
    __asm__ volatile("" : : "r"(acc));

    print("\r\n=== Load/store speed test ===\r\n");
    print("ITER: ");      putdec64(ITER);            print("\r\n");

    print("Empty loop cycles: "); putdec64(empty_cycles); print("\r\n");
    print("Empty loop insts : "); putdec64(empty_insts);  print("\r\n");

    print("Total cycles     : "); putdec64(total_cycles); print("\r\n");
    print("Total insts      : "); putdec64(total_insts);  print("\r\n");

    print("Body-only cycles : "); putdec64(body_cycles);  print("\r\n");
    print("Body-only insts  : "); putdec64(body_insts);   print("\r\n");
}

// ---------------------------------------------------------------------------
// Test 6: Branch tests (always, never, 50/50)
// ---------------------------------------------------------------------------
static void branch_test_always_taken(void) {
    const uint32_t ITER = 200000;
    volatile uint32_t x = 0;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        if (1) {
            x++;
        }
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(x));

    print("\r\n=== Branch Test: Always Taken ===\r\n");
    print("ITER: ");      putdec64(ITER);            print("\r\n");
    print("Body-only cycles : "); putdec64(body_cycles);  print("\r\n");
    print("Body-only insts  : "); putdec64(body_insts);   print("\r\n");
}

static void branch_test_never_taken(void) {
    const uint32_t ITER = 200000;
    volatile uint32_t x = 0;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        if (0) {
            x++;
        }
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(x));

    print("\r\n=== Branch Test: Never Taken ===\r\n");
    print("ITER: ");      putdec64(ITER);            print("\r\n");
    print("Body-only cycles : "); putdec64(body_cycles);  print("\r\n");
    print("Body-only insts  : "); putdec64(body_insts);   print("\r\n");
}

// 50/50 branch behavior (your original style)
static void branch_behavior_test(void) {
    const uint32_t ITER = 200000;

    volatile uint32_t acc = 0;
    const uint32_t step = 1;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        if (k & 1u) {
            acc += step;
        }
        else {
            acc -= step;
        }
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(acc));

    print("\r\n=== Branch Test: 50/50 Taken ===\r\n");
    print("ITER: ");      putdec64(ITER);            print("\r\n");

    print("Empty loop cycles: "); putdec64(empty_cycles); print("\r\n");
    print("Empty loop insts : "); putdec64(empty_insts);  print("\r\n");

    print("Total cycles     : "); putdec64(total_cycles); print("\r\n");
    print("Total insts      : "); putdec64(total_insts);  print("\r\n");

    print("Body-only cycles : "); putdec64(body_cycles);  print("\r\n");
    print("Body-only insts  : "); putdec64(body_insts);   print("\r\n");
}

// ---------------------------------------------------------------------------
// Test 7: Bit-shift throughput test
// ---------------------------------------------------------------------------
static void shift_throughput_test(void) {
    const uint32_t ITER = 200000;

    volatile uint32_t x = 1;

    uint64_t empty_cycles, empty_insts;
    measure_empty_loop(ITER, &empty_cycles, &empty_insts);

    uint64_t c0 = rdcycle64();
    uint64_t i0 = rdinstret64();

    for (uint32_t k = 0; k < ITER; k++) {
        x = x << 1;
        x = x >> 1;
        x = x << 3;
    }

    uint64_t c1 = rdcycle64();
    uint64_t i1 = rdinstret64();

    uint64_t total_cycles = c1 - c0;
    uint64_t total_insts = i1 - i0;

    uint64_t body_cycles = total_cycles - empty_cycles;
    uint64_t body_insts = total_insts - empty_insts;

    __asm__ volatile("" : : "r"(x));

    print("\r\n=== Bit Shift Throughput Test ===\r\n");
    print("ITER: ");      putdec64(ITER);            print("\r\n");
    print("Body-only cycles : "); putdec64(body_cycles);  print("\r\n");
    print("Body-only insts  : "); putdec64(body_insts);   print("\r\n");
}

// ---------------------------------------------------------------------------
// Power test: heavy addition loop for ~10 seconds at 100MHz
// (same idea as your original addition_power_test)
// ---------------------------------------------------------------------------
static void addition_power_test(void) {
    volatile uint32_t a = 1, b = 2, c = 3, d = 4, e = 5, f = 6, g = 7, h = 8;
    const uint32_t ITER = 2500000;  // adjust to taste

    print("\r\n=== Addition power test (running) ===\r\n");
    print("This may look 'frozen' while it runs.\r\n");

    for (uint32_t k = 0; k < ITER; k++) {
        a = b + c;  d = e + f;  g = h + a;  b = c + d;
        c = d + e;  e = f + g;  f = g + h;  h = a + b;

        __asm__ volatile("" : "+r"(a), "+r"(b), "+r"(c),
            "+r"(d), "+r"(e), "+r"(f),
            "+r"(g), "+r"(h));
    }

    print("Addition power test complete. Measure power during the run.\r\n");
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(void) {
    GPIO_A->OUTPUT_ENABLE = 0x0000000F;
    GPIO_A->OUTPUT = 0x00000001;
    print("Booting...\r\n");

    print("Press CONTINUE to run performance tests\r\n");
    print("or SKIP to go directly to the power test.\r\n");

    while (1) {
        if (btn_continue_pressed()) {
            wait_for_press_release(btn_continue_pressed);

            // Performance tests (short, print results)
            addition_speed_test();          // basic add loop CPI
            alu_throughput_test();          // ALU throughput
            multiply_latency_test();        // MUL latency / throughput
            divide_latency_test();          // DIV latency / throughput
            load_store_speed_test();        // memory access
            branch_test_always_taken();     // branch always taken
            branch_test_never_taken();      // branch never taken
            branch_behavior_test();         // 50/50 taken
            shift_throughput_test();        // bit shifts

            break;
        } else if (btn_skip_pressed()) {
            wait_for_press_release(btn_skip_pressed);
            break;
        }
    }

    print("\r\nPress CONTINUE for addition power test\r\n");
    print("or SKIP to finish.\r\n");

    while (1) {
        if (btn_continue_pressed()) {
            wait_for_press_release(btn_continue_pressed);
            addition_power_test();
            break;
        } else if (btn_skip_pressed()) {
            wait_for_press_release(btn_skip_pressed);
            break;
        }
    }

    print("\r\nProgram done. Press reset to rerun.\r\n");
    while (1) __asm__ volatile("nop");
}
