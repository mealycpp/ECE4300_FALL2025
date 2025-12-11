#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Hardware configuration                                                     */
/* -------------------------------------------------------------------------- */

/* 2048-bit Montgomery accelerator (original core) */
#define MONT2048_BASE   XPAR_MONTGOMERY_AXI_0_BASEADDR

/* 1024-bit Montgomery accelerator (new 1024-bit IP block) */
/* If your IP name is different, adjust this macro, e.g.:
 *   #define MONT1024_BASE  XPAR_MONTGOMERY_AXI_1024_0_BASEADDR
 */
#define MONT1024_BASE   XPAR_MONTGOMERY_AXI_1024_0_BASEADDR

/* AXI register layout – must match both AXI wrappers */
#define REG_A(base,i)       ((base) + 0x000U + 4U*(i))
#define REG_B(base,i)       ((base) + 0x200U + 4U*(i))
#define REG_N(base,i)       ((base) + 0x400U + 4U*(i))
#define REG_RES(base,i)     ((base) + 0x600U + 4U*(i))
#define REG_NPRIME(base)    ((base) + 0x800U)
#define REG_CONTROL(base)   ((base) + 0x804U)
#define REG_STATUS(base)    ((base) + 0x808U)

/* word sizes */
#define NWORDS_1024     32U        /* 1024 / 32 */
#define NWORDS_2048     64U        /* 2048 / 32 */
#define MAX_WORDS       NWORDS_2048

/* benchmark runs per case */
#define NUM_RUNS        32U

/* max polls for HW done (prevents infinite hang) */
#define HW_DONE_TIMEOUT 100000000U

typedef uint32_t u32;
typedef uint64_t u64;

/* -------------------------------------------------------------------------- */
/* Global timer (Zynq ARM generic timer)                                      */
/* -------------------------------------------------------------------------- */

#define GTIMER_BASE     0xF8F00200U
#define GTIMER_CTRL     (GTIMER_BASE + 0x08U)

/* Approximate frequency in Hz (adjust if you know exact value) */
#define GTIMER_FREQ_HZ  650000000U

static void Timer_Init(void)
{
    /* enable global timer (bit 0 = EN) */
    u32 ctrl = Xil_In32(GTIMER_CTRL);
    ctrl |= 0x1U;
    Xil_Out32(GTIMER_CTRL, ctrl);

    xil_printf("[INFO] Global timer enabled, freq ~%u Hz\r\n",
               (unsigned)GTIMER_FREQ_HZ);
}

static inline u64 Timer_GetCount(void)
{
    u32 low, high0, high1;
    do {
        high0 = Xil_In32(GTIMER_BASE + 0x04U);
        low   = Xil_In32(GTIMER_BASE + 0x00U);
        high1 = Xil_In32(GTIMER_BASE + 0x04U);
    } while (high0 != high1);
    return ((u64)high1 << 32) | (u64)low;
}

static inline u64 Timer_Delta(u64 start, u64 end)
{
    if (end >= start) return end - start;
    return (0xFFFFFFFFFFFFFFFFULL - start) + 1ULL + end;
}

/* -------------------------------------------------------------------------- */
/* Toy RSA key (same for both sizes – padded with zeros)                     */
/*   n = 3233, e = 17, d = 2753                                              */
/* -------------------------------------------------------------------------- */

static u32 RSA_N[MAX_WORDS] = {
    3233U, 0U   /* remaining elements auto-zeroed */
};

#define RSA_E           17U
#define RSA_E_BITS      5       /* 17 = 0b10001 */
#define RSA_D           2753U
#define RSA_D_BITS      12

/* example plaintext m < n, padded */
static const u32 RSA_MSG[MAX_WORDS] = {
    42U, 0U
};

/* R^2 mod n for each size (Montgomery) */
static u32 RSA_R2_1024[MAX_WORDS];
static u32 RSA_R2_2048[MAX_WORDS];

/* Montgomery n' = -n^{-1} mod 2^32 (per size; same numeric value) */
static u32 NPRIME_1024;
static u32 NPRIME_2048;

/* -------------------------------------------------------------------------- */
/* Big-integer helpers                                                        */
/* -------------------------------------------------------------------------- */

static void bigint_copy(u32 *dst, const u32 *src, u32 nwords)
{
    for (u32 i = 0; i < nwords; ++i)
        dst[i] = src[i];
}

static void bigint_set_u32(u32 *dst, u32 v, u32 nwords)
{
    dst[0] = v;
    for (u32 i = 1; i < nwords; ++i)
        dst[i] = 0U;
}

static int bigint_equal(const u32 *a, const u32 *b, u32 nwords)
{
    for (u32 i = 0; i < nwords; ++i)
        if (a[i] != b[i])
            return 0;
    return 1;
}

/* simple software (reference) modular multiply: R = (A * B) mod N */
static void modmul_sw(const u32 *A, const u32 *B, const u32 *N, u32 *R, u32 nwords)
{
    u64 tmp[2 * MAX_WORDS];
    u32 i, j;

    for (i = 0; i < 2U * nwords; ++i)
        tmp[i] = 0ULL;

    for (i = 0; i < nwords; ++i) {
        u64 carry = 0ULL;
        for (j = 0; j < nwords; ++j) {
            u64 t = tmp[i + j] + (u64)A[i] * (u64)B[j] + carry;
            tmp[i + j] = (u32)t;
            carry = t >> 32;
        }
        tmp[i + nwords] += carry;
    }

    for (i = 0; i < nwords; ++i)
        R[i] = (u32)tmp[i];

    for (;;) {
        int ge = 0;
        for (i = nwords; i > 0; ) {
            --i;
            if (R[i] > N[i]) { ge = 1; break; }
            if (R[i] < N[i]) { ge = 0; break; }
        }
        if (!ge) break;

        u64 borrow = 0ULL;
        for (i = 0; i < nwords; ++i) {
            u64 t = (u64)R[i] - (u64)N[i] - borrow;
            R[i] = (u32)t;
            borrow = (t >> 63) & 1ULL;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* HW Montgomery wrapper (with timeout)                                      */
/* -------------------------------------------------------------------------- */

static int montgomery_mul_hw(u32 base_addr,
                             u32 nwords,
                             const u32 *A,
                             const u32 *B,
                             const u32 *N,
                             u32 nprime,
                             u32 *R,
                             const char *label)
{
    u32 i;

    for (i = 0; i < nwords; ++i) {
        Xil_Out32(REG_A(base_addr, i), A[i]);
        Xil_Out32(REG_B(base_addr, i), B[i]);
        Xil_Out32(REG_N(base_addr, i), N[i]);
    }

    Xil_Out32(REG_NPRIME(base_addr), nprime);
    Xil_Out32(REG_CONTROL(base_addr), 1U);      /* start */

    u32 polls = 0;
    while ((Xil_In32(REG_STATUS(base_addr)) & 0x1U) == 0U) {
        if (++polls > HW_DONE_TIMEOUT) {
            xil_printf("[ERROR] HW timeout in montgomery_mul_hw for %s (base 0x%08lx)\r\n",
                       label, (unsigned long)base_addr);
            return 0;
        }
    }

    for (i = 0; i < nwords; ++i)
        R[i] = Xil_In32(REG_RES(base_addr, i));

    return 1;
}

/* -------------------------------------------------------------------------- */
/* Montgomery / RSA setup                                                     */
/* -------------------------------------------------------------------------- */

/* inverse of n modulo 2^32 (n must be odd) */
static u32 modinv32(u32 n)
{
    int64_t t = 0, new_t = 1;
    int64_t r = (int64_t)(1ULL << 32), new_r = n;

    while (new_r != 0) {
        int64_t q = r / new_r;
        int64_t tmp;

        tmp = new_t;
        new_t = t - q * new_t;
        t = tmp;

        tmp = new_r;
        new_r = r - q * new_r;
        r = tmp;
    }

    if (t < 0)
        t += ((int64_t)1 << 32);

    return (u32)t;
}

/* compute 32-bit R^2 mod n, where R = 2^(32*nwords) (toy 32-bit modulus) */
static u32 compute_R2_modN_32(u32 n0, u32 nwords)
{
    u64 r = 1ULL;
    u32 total_bits = 32U * nwords;

    for (u32 i = 0; i < total_bits; ++i)
        r = (r * 2ULL) % (u64)n0;

    r = (r * r) % (u64)n0;
    return (u32)r;
}

/* Fill R2_out[] and nprime_out for a given word size */
static void init_mont_params_for_size(u32 nwords, u32 *R2_out, u32 *nprime_out)
{
    u32 n0 = RSA_N[0];
    u32 inv = modinv32(n0);
    u32 nprime = (u32)(0U - inv);          /* n' = -n^{-1} mod 2^32 */
    u32 r2 = compute_R2_modN_32(n0, nwords);

    for (u32 i = 0; i < nwords; ++i)
        R2_out[i] = 0U;
    R2_out[0] = r2;

    *nprime_out = nprime;
}

/* HW modular exponentiation (square-and-multiply, scalar exponent) */
static int modexp_hw_scalar(u32 base_addr,
                            const u32 *base,
                            u32 exp,
                            int exp_bits,
                            const u32 *N,
                            u32 nprime,
                            const u32 *R2,
                            u32 *result,
                            u32 nwords,
                            const char *label)
{
    u32 one[MAX_WORDS];
    u32 x[MAX_WORDS];
    u32 a[MAX_WORDS];
    int bit;
    int ok;

    bigint_set_u32(one, 1U, nwords);

    ok = montgomery_mul_hw(base_addr, nwords, one,  R2,  N, nprime, x, label);
    if (!ok) return 0;
    ok = montgomery_mul_hw(base_addr, nwords, base, R2,  N, nprime, a, label);
    if (!ok) return 0;

    for (bit = 0; bit < exp_bits; ++bit) {
        if ((exp >> bit) & 1U) {
            ok = montgomery_mul_hw(base_addr, nwords, x, a, N, nprime, x, label);
            if (!ok) return 0;
        }
        ok = montgomery_mul_hw(base_addr, nwords, a, a, N, nprime, a, label);
        if (!ok) return 0;
    }

    ok = montgomery_mul_hw(base_addr, nwords, x, one, N, nprime, result, label);
    if (!ok) return 0;

    return 1;
}

/* SW modular exponentiation (scalar exponent) */
static void modexp_sw_scalar(const u32 *base,
                             u32 exp,
                             int exp_bits,
                             const u32 *N,
                             u32 *result,
                             u32 nwords)
{
    u32 one[MAX_WORDS];
    u32 x[MAX_WORDS];
    u32 a[MAX_WORDS];
    int bit;

    bigint_set_u32(one, 1U, nwords);
    bigint_set_u32(x, 1U, nwords);
    bigint_copy(a, base, nwords);

    for (bit = 0; bit < exp_bits; ++bit) {
        if ((exp >> bit) & 1U)
            modmul_sw(x, a, N, x, nwords);
        modmul_sw(a, a, N, a, nwords);
    }

    bigint_copy(result, x, nwords);
}

/* -------------------------------------------------------------------------- */
/* Benchmark for a single key size                                            */
/* -------------------------------------------------------------------------- */

static void benchmark_rsa_size(const char *label,
                               u32 key_bits,
                               u32 nwords,
                               u32 base_addr,
                               const u32 *N,
                               const u32 *R2,
                               u32 nprime,
                               u32 e, int e_bits,
                               u32 d, int d_bits)
{
    u32 msg[MAX_WORDS];
    u32 c_hw[MAX_WORDS], m_hw[MAX_WORDS];
    u32 c_sw[MAX_WORDS], m_sw[MAX_WORDS];

    u64 enc_cycles_hw = 0, dec_cycles_hw = 0;
    u64 enc_cycles_sw = 0, dec_cycles_sw = 0;

    xil_printf("\r\n==============================\r\n");
    xil_printf(" %s (key size: %u bits)\r\n", label, (unsigned)key_bits);
    xil_printf("==============================\r\n");

    bigint_copy(msg, RSA_MSG, nwords);

    xil_printf("[DEBUG] Plaintext first 4 words (LE): %08x %08x %08x %08x\r\n",
               (unsigned)msg[0], (unsigned)msg[1],
               (unsigned)msg[2], (unsigned)msg[3]);

    /* HW encrypt runs */
    for (u32 run = 0; run < NUM_RUNS; ++run) {
        u64 start = Timer_GetCount();
        if (!modexp_hw_scalar(base_addr, msg, e, e_bits, N, nprime, R2,
                              c_hw, nwords, label)) {
            xil_printf("[ERROR] Aborting %s HW encrypt benchmark due to HW error.\r\n", label);
            return;
        }
        u64 end = Timer_GetCount();
        enc_cycles_hw += Timer_Delta(start, end);
    }

    /* HW decrypt runs */
    for (u32 run = 0; run < NUM_RUNS; ++run) {
        u64 start = Timer_GetCount();
        if (!modexp_hw_scalar(base_addr, c_hw, d, d_bits, N, nprime, R2,
                              m_hw, nwords, label)) {
            xil_printf("[ERROR] Aborting %s HW decrypt benchmark due to HW error.\r\n", label);
            return;
        }
        u64 end = Timer_GetCount();
        dec_cycles_hw += Timer_Delta(start, end);
    }

    /* SW encrypt runs */
    for (u32 run = 0; run < NUM_RUNS; ++run) {
        u64 start = Timer_GetCount();
        modexp_sw_scalar(msg, e, e_bits, N, c_sw, nwords);
        u64 end = Timer_GetCount();
        enc_cycles_sw += Timer_Delta(start, end);
    }

    /* SW decrypt runs */
    for (u32 run = 0; run < NUM_RUNS; ++run) {
        u64 start = Timer_GetCount();
        modexp_sw_scalar(c_sw, d, d_bits, N, m_sw, nwords);
        u64 end = Timer_GetCount();
        dec_cycles_sw += Timer_Delta(start, end);
    }

    u64 enc_hw_avg = enc_cycles_hw / NUM_RUNS;
    u64 dec_hw_avg = dec_cycles_hw / NUM_RUNS;
    u64 enc_sw_avg = enc_cycles_sw / NUM_RUNS;
    u64 dec_sw_avg = dec_cycles_sw / NUM_RUNS;

    /* time elapsed (ns) */
    u64 enc_hw_ns = (enc_hw_avg * 1000000000ULL) / (u64)GTIMER_FREQ_HZ;
    u64 dec_hw_ns = (dec_hw_avg * 1000000000ULL) / (u64)GTIMER_FREQ_HZ;
    u64 enc_sw_ns = (enc_sw_avg * 1000000000ULL) / (u64)GTIMER_FREQ_HZ;
    u64 dec_sw_ns = (dec_sw_avg * 1000000000ULL) / (u64)GTIMER_FREQ_HZ;

    /* throughput in bits/s and Mbit/s */
    u64 bits_per_op = (u64)key_bits;

    u64 enc_hw_bits_s = (enc_hw_avg > 0) ? (bits_per_op * (u64)GTIMER_FREQ_HZ) / enc_hw_avg : 0;
    u64 dec_hw_bits_s = (dec_hw_avg > 0) ? (bits_per_op * (u64)GTIMER_FREQ_HZ) / dec_hw_avg : 0;
    u64 enc_sw_bits_s = (enc_sw_avg > 0) ? (bits_per_op * (u64)GTIMER_FREQ_HZ) / enc_sw_avg : 0;
    u64 dec_sw_bits_s = (dec_sw_avg > 0) ? (bits_per_op * (u64)GTIMER_FREQ_HZ) / dec_sw_avg : 0;

    u32 enc_hw_mbps = (u32)(enc_hw_bits_s / 1000000ULL);
    u32 dec_hw_mbps = (u32)(dec_hw_bits_s / 1000000ULL);
    u32 enc_sw_mbps = (u32)(enc_sw_bits_s / 1000000ULL);
    u32 dec_sw_mbps = (u32)(dec_sw_bits_s / 1000000ULL);

    /* speedup (SW/HW), x1000 to print 3 decimal places */
    u64 enc_spd_x1000 = (enc_hw_avg > 0) ? (enc_sw_avg * 1000ULL) / enc_hw_avg : 0;
    u64 dec_spd_x1000 = (dec_hw_avg > 0) ? (dec_sw_avg * 1000ULL) / dec_hw_avg : 0;
    u32 enc_spd_int   = (u32)(enc_spd_x1000 / 1000ULL);
    u32 enc_spd_frac  = (u32)(enc_spd_x1000 % 1000ULL);
    u32 dec_spd_int   = (u32)(dec_spd_x1000 / 1000ULL);
    u32 dec_spd_frac  = (u32)(dec_spd_x1000 % 1000ULL);

    /* Debug: ciphertext and decrypted msg (first 4 words) */
    xil_printf("[DEBUG] HW ciphertext first 4 words: %08x %08x %08x %08x\r\n",
               (unsigned)c_hw[0], (unsigned)c_hw[1],
               (unsigned)c_hw[2], (unsigned)c_hw[3]);

    xil_printf("[DEBUG] SW ciphertext first 4 words: %08x %08x %08x %08x\r\n",
               (unsigned)c_sw[0], (unsigned)c_sw[1],
               (unsigned)c_sw[2], (unsigned)c_sw[3]);

    xil_printf("[DEBUG] HW decrypted first 4 words: %08x %08x %08x %08x\r\n",
               (unsigned)m_hw[0], (unsigned)m_hw[1],
               (unsigned)m_hw[2], (unsigned)m_hw[3]);

    xil_printf("[DEBUG] SW decrypted first 4 words: %08x %08x %08x %08x\r\n",
               (unsigned)m_sw[0], (unsigned)m_sw[1],
               (unsigned)m_sw[2], (unsigned)m_sw[3]);

    xil_printf("\r\n[Performance] %s\r\n", label);

    xil_printf(" HW enc: avg %lu cycles, %lu ns, %u Mbit/s\r\n",
               (unsigned long)enc_hw_avg, (unsigned long)enc_hw_ns,
               (unsigned)enc_hw_mbps);
    xil_printf(" HW dec: avg %lu cycles, %lu ns, %u Mbit/s\r\n",
               (unsigned long)dec_hw_avg, (unsigned long)dec_hw_ns,
               (unsigned)dec_hw_mbps);

    xil_printf(" SW enc: avg %lu cycles, %lu ns, %u Mbit/s\r\n",
               (unsigned long)enc_sw_avg, (unsigned long)enc_sw_ns,
               (unsigned)enc_sw_mbps);
    xil_printf(" SW dec: avg %lu cycles, %lu ns, %u Mbit/s\r\n",
               (unsigned long)dec_sw_avg, (unsigned long)dec_sw_ns,
               (unsigned)dec_sw_mbps);

    xil_printf(" Enc speedup (SW/HW): %u.%03ux\r\n",
               (unsigned)enc_spd_int, (unsigned)enc_spd_frac);
    xil_printf(" Dec speedup (SW/HW): %u.%03ux\r\n",
               (unsigned)dec_spd_int, (unsigned)dec_spd_frac);

    xil_printf("\r\n[Correctness]\r\n");
    xil_printf(" HW dec == msg: %s\r\n",
               bigint_equal(m_hw, msg, nwords) ? "OK" : "FAIL");
    xil_printf(" SW dec == msg: %s\r\n",
               bigint_equal(m_sw, msg, nwords) ? "OK" : "FAIL");
}

/* -------------------------------------------------------------------------- */
/* main                                                                       */
/* -------------------------------------------------------------------------- */

int main(void)
{
    xil_printf("RSA HW/SW benchmarks with Montgomery accelerators\r\n");

    Timer_Init();

    /* Precompute Montgomery parameters for each key size */
    init_mont_params_for_size(NWORDS_1024, RSA_R2_1024, &NPRIME_1024);
    init_mont_params_for_size(NWORDS_2048, RSA_R2_2048, &NPRIME_2048);

    /* 2048-bit benchmark (HW: montgomery_axi_0) */
    benchmark_rsa_size("RSA-2048 (HW: montgomery_axi_0)",
                       2048U,
                       NWORDS_2048,
                       MONT2048_BASE,
                       RSA_N,
                       RSA_R2_2048,
                       NPRIME_2048,
                       RSA_E, RSA_E_BITS,
                       RSA_D, RSA_D_BITS);

    /* 1024-bit benchmark (HW: montgomery_axi_1024) */
    benchmark_rsa_size("RSA-1024 (HW: montgomery_axi_1024)",
                       1024U,
                       NWORDS_1024,
                       MONT1024_BASE,
                       RSA_N,
                       RSA_R2_1024,
                       NPRIME_1024,
                       RSA_E, RSA_E_BITS,
                       RSA_D, RSA_D_BITS);

    xil_printf("\r\nAll benchmarks finished.\r\n");

    while (1) {
        /* idle */
    }

    return 0;
}
