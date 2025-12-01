#include <iostream>
#include <vector>
#include <chrono>
#include <windows.h>

// -----------------------------------------------------------------------------
// Matrix multiplication (Divide & Conquer)
// -----------------------------------------------------------------------------
void add(std::vector<std::vector<float>>& A,
    std::vector<std::vector<float>>& B,
    std::vector<std::vector<float>>& C,
    int size)
{
    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
            C[i][j] = A[i][j] + B[i][j];
}

void sub(std::vector<std::vector<float>>& A,
    std::vector<std::vector<float>>& B,
    std::vector<std::vector<float>>& C,
    int size)
{
    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
            C[i][j] = A[i][j] - B[i][j];
}

void mulDC(std::vector<std::vector<float>>& A,
    std::vector<std::vector<float>>& B,
    std::vector<std::vector<float>>& C,
    int size)
{
    if (size <= 64)        // base case threshold
    {
        for (int i = 0; i < size; i++)
            for (int j = 0; j < size; j++)
                for (int k = 0; k < size; k++)
                    C[i][j] += A[i][k] * B[k][j];
        return;
    }

    int newSize = size / 2;

    std::vector<std::vector<float>>
        A11(newSize, std::vector<float>(newSize)),
        A12(newSize, std::vector<float>(newSize)),
        A21(newSize, std::vector<float>(newSize)),
        A22(newSize, std::vector<float>(newSize)),
        B11(newSize, std::vector<float>(newSize)),
        B12(newSize, std::vector<float>(newSize)),
        B21(newSize, std::vector<float>(newSize)),
        B22(newSize, std::vector<float>(newSize)),
        C11(newSize, std::vector<float>(newSize)),
        C12(newSize, std::vector<float>(newSize)),
        C21(newSize, std::vector<float>(newSize)),
        C22(newSize, std::vector<float>(newSize)),
        M1(newSize, std::vector<float>(newSize)),
        M2(newSize, std::vector<float>(newSize)),
        M3(newSize, std::vector<float>(newSize)),
        M4(newSize, std::vector<float>(newSize)),
        M5(newSize, std::vector<float>(newSize)),
        M6(newSize, std::vector<float>(newSize)),
        M7(newSize, std::vector<float>(newSize)),
        AResult(newSize, std::vector<float>(newSize)),
        BResult(newSize, std::vector<float>(newSize));

    for (int i = 0; i < newSize; i++)
        for (int j = 0; j < newSize; j++)
        {
            A11[i][j] = A[i][j];
            A12[i][j] = A[i][j + newSize];
            A21[i][j] = A[i + newSize][j];
            A22[i][j] = A[i + newSize][j + newSize];
            B11[i][j] = B[i][j];
            B12[i][j] = B[i][j + newSize];
            B21[i][j] = B[i + newSize][j];
            B22[i][j] = B[i + newSize][j + newSize];
        }

    add(A11, A22, AResult, newSize);
    add(B11, B22, BResult, newSize);
    mulDC(AResult, BResult, M1, newSize);

    add(A21, A22, AResult, newSize);
    mulDC(AResult, B11, M2, newSize);

    sub(B12, B22, BResult, newSize);
    mulDC(A11, BResult, M3, newSize);

    sub(B21, B11, BResult, newSize);
    mulDC(A22, BResult, M4, newSize);

    add(A11, A12, AResult, newSize);
    mulDC(AResult, B22, M5, newSize);

    sub(A21, A11, AResult, newSize);
    add(B11, B12, BResult, newSize);
    mulDC(AResult, BResult, M6, newSize);

    sub(A12, A22, AResult, newSize);
    add(B21, B22, BResult, newSize);
    mulDC(AResult, BResult, M7, newSize);

    for (int i = 0; i < newSize; i++)
        for (int j = 0; j < newSize; j++)
        {
            C11[i][j] = M1[i][j] + M4[i][j] - M5[i][j] + M7[i][j];
            C12[i][j] = M3[i][j] + M5[i][j];
            C21[i][j] = M2[i][j] + M4[i][j];
            C22[i][j] = M1[i][j] - M2[i][j] + M3[i][j] + M6[i][j];
        }

    for (int i = 0; i < newSize; i++)
        for (int j = 0; j < newSize; j++)
        {
            C[i][j] = C11[i][j];
            C[i][j + newSize] = C12[i][j];
            C[i + newSize][j] = C21[i][j];
            C[i + newSize][j + newSize] = C22[i][j];
        }
}


// -----------------------------------------------------------------------------
// Performance & "Energy Estimate"
// -----------------------------------------------------------------------------

double fileTimeToSeconds(FILETIME ft)
{
    ULARGE_INTEGER t;
    t.LowPart = ft.dwLowDateTime;
    t.HighPart = ft.dwHighDateTime;
    return (double)t.QuadPart / 10000000.0;
}

int main()
{
    int size = 512;

    std::vector<std::vector<float>> A(size, std::vector<float>(size));
    std::vector<std::vector<float>> B(size, std::vector<float>(size));
    std::vector<std::vector<float>> C(size, std::vector<float>(size));

    for (int i = 0; i < size; i++)
        for (int j = 0; j < size; j++)
        {
            A[i][j] = (float)(rand() % 10);
            B[i][j] = (float)(rand() % 10);
        }

    FILETIME startKernel, startUser, endKernel, endUser, dummy;
    GetProcessTimes(GetCurrentProcess(), &dummy, &dummy, &startKernel, &startUser);

    auto start = std::chrono::high_resolution_clock::now();

    mulDC(A, B, C, size);

    auto end = std::chrono::high_resolution_clock::now();
    GetProcessTimes(GetCurrentProcess(), &dummy, &dummy, &endKernel, &endUser);

    double wallSeconds =
        std::chrono::duration<double>(end - start).count();

    double userSec = fileTimeToSeconds(endUser) - fileTimeToSeconds(startUser);
    double kernSec = fileTimeToSeconds(endKernel) - fileTimeToSeconds(startKernel);
    double cpuSec = userSec + kernSec;

    // crude energy estimate: CPU_time * 15W (typical laptop CPU package power)
    double estimatedEnergy_Wh = (cpuSec / 3600.0) * 15.0;

    std::cout << "Matrix size: " << size << " x " << size << "\n";
    std::cout << "Wall clock time: " << wallSeconds << " sec\n";
    std::cout << "CPU time: " << cpuSec << " sec\n";
    std::cout << "Estimated energy: " << estimatedEnergy_Wh << " Wh\n";
}
