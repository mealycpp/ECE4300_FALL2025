#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
 * Divide-and-conquer matrix multiplication: C += A * B
 * Matrices are n x n, stored in row-major 1D arrays.
 * Assumes n is a power of 2.
 */

void matmul_dc(double *A, double *B, double *C, int n) {
    if (n == 1) {
        C[0] += A[0] * B[0];
        return;
    }

    int newN = n / 2;

    // Sub-blocks of A
    double *A11 = A;
    double *A12 = A + newN;
    double *A21 = A + newN * n;
    double *A22 = A + newN * n + newN;

    // Sub-blocks of B
    double *B11 = B;
    double *B12 = B + newN;
    double *B21 = B + newN * n;
    double *B22 = B + newN * n + newN;

    // Sub-blocks of C
    double *C11 = C;
    double *C12 = C + newN;
    double *C21 = C + newN * n;
    double *C22 = C + newN * n + newN;

    // Perform recursive operations
    matmul_dc(A11, B11, C11, newN);
    matmul_dc(A12, B21, C11, newN);

    matmul_dc(A11, B12, C12, newN);
    matmul_dc(A12, B22, C12, newN);

    matmul_dc(A21, B11, C21, newN);
    matmul_dc(A22, B21, C21, newN);

    matmul_dc(A21, B12, C22, newN);
    matmul_dc(A22, B22, C22, newN);
}

// Print matrix (for debugging)
void print_matrix(double *M, int n, const char *name) {
    printf("%s =\n", name);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            printf("%8.2f ", M[i * n + j]);
        }
        printf("\n");
    }
    printf("\n");
}

// High-resolution time difference in seconds
double time_diff(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec)
         + (end.tv_nsec - start.tv_nsec) / 1e9;
}

int main(void) {
    int n = 512;   // Must be a power of 2
    int size = n * n;

    double *A = malloc(size * sizeof(double));
    double *B = malloc(size * sizeof(double));
    double *C = malloc(size * sizeof(double));

    if (!A || !B || !C) {
        fprintf(stderr, "Memory allocation failed\n");
        return 1;
    }

    // Initialize matrices
    for (int i = 0; i < size; ++i) {
        A[i] = 1.0;  
        B[i] = 1.0;
        C[i] = 0.0;
    }

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    // Perform matrix multiplication
    matmul_dc(A, B, C, n);

    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed = time_diff(start, end);
    printf("Elapsed time: %.6f seconds\n", elapsed);

    // print_matrix(C, n, "Result"); // Uncomment for debugging

    free(A);
    free(B);
    free(C);
    return 0;
}
