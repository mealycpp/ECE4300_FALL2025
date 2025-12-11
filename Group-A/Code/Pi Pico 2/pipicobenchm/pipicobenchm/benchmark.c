#include <stdio.h>
#include <string.h>
// Include sys/types.h before inttypes.h to work around issue with
// certain versions of GCC and newlib which causes omission of PRIu64
#include <sys/types.h>
#include <inttypes.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "pico/sha256.h"
#include "stdint.h"
#include "benchmark.h"

void benchmark() {

    // Constants for SHA256 transformation
    static const uint32_t K[64] = {
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
    };

    // printf("Text: %d bytes\n", sizeof(K) - 1);
    // for(int i = 0; i < sizeof(K) - 1; i++) {
    //     if (i > 0 && i % 128 == 0) printf("\n");
    //     putchar(K[i]);
    // }
    // printf("\n");

    // Allocate a state object and start the calculation
    pico_sha256_state_t state;
    int rc = pico_sha256_start_blocking(&state, SHA256_BIG_ENDIAN, true); // using some DMA system resources
    hard_assert(rc == PICO_OK);
    pico_sha256_update_blocking(&state, (const uint8_t*)K, sizeof(K) - 1);

    // Get the result of the sha256 calculation
    sha256_result_t result;
    pico_sha256_finish(&state, &result);

    // print resulting sha256 result
    // printf("Result:\n");
    // for(int i = 0; i < SHA256_RESULT_BYTES; i++) {
    //     printf("%02x ", result.bytes[i]);
    //     if ((i+1) % 16 == 0) printf("\n");
    // }
}