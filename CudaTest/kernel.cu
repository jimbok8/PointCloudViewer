#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <cassert>
#include <cstdio>
#include <iostream>



int main()
{
    int size = 10000;
    int* a;
    int* aGPU;
    int* locks;

    a = new int[size];

    cudaMalloc(&aGPU, sizeof(int) * size);
    cudaMemset(aGPU, 0, sizeof(int) * size);

    cudaMalloc(&locks, sizeof(int) * size);
    cudaMemset(locks, 0, sizeof(int) * size);

    maxKernel<<<64, 64>>>(size, aGPU, locks);
    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess)
        fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));

    cudaMemcpy(a, aGPU, sizeof(int) * size, cudaMemcpyDeviceToHost);
    for (int i = 0; i < size - 1; i++)
        std::cout << a[i] << ' ';
    std::cout << a[size - 1] << std::endl;


    delete[] a;
    cudaFree(aGPU);
    cudaFree(locks);

    return 0;
}
