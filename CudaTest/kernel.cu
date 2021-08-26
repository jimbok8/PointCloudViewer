#include "cuda_runtime.h"
#include "device_launch_parameters.h"

#include <cassert>
#include <cstdio>
#include <iostream>

__global__ void addKernel(int size, const int* a, int* sum, int* lock) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    while (i < size) {
        __syncthreads();
        bool success = false;
        while (!success) {
            if (atomicExch(lock, 1) == 0) {
                *sum += a[i];
                __threadfence_system();
                atomicExch(lock, 0);
                success = true;
            }
        }
        //__syncthreads();
        //bool loopFlag = false;
        //do {
        //    if ((loopFlag = atomicCAS(lock, 0, 1) == 0)) {
        //        *sum += a[i];
        //    }
        //    __threadfence_system(); //Or __threadfence_block(), __threadfence_system() according to your Memory Fence demand
        //    if (loopFlag)
        //        atomicExch(lock, 0);
        //} while (!loopFlag);

        i += gridDim.x * blockDim.x;
    }
}

__global__ void maxKernel(int size, int* a, int* locks) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    while (i < size) {
        if (i > 0) {
            //__syncthreads();
            bool loopFlag = false;
            do {
                if ((loopFlag = atomicCAS(&locks[i - 1], 0, 1) == 0)) {
                    a[i - 1] = i > a[i - 1] ? i : a[i - 1];
                }
                __threadfence_system(); //Or __threadfence_block(), __threadfence_system() according to your Memory Fence demand
                if (loopFlag)
                    atomicExch(&locks[i - 1], 0);
            } while (!loopFlag);
            //__syncthreads();
            //bool success = false;
            //while (!success) {
            //    if (atomicExch(&lock[i - 1], 1) == 0) {
            //        a[i - 1] = i > a[i - 1] ? i : a[i - 1];
            //        __threadfence_system();
            //        atomicExch(&lock[i - 1], 0);
            //        success = true;
            //    }
            //}
        }

        if (true) {
            //__syncthreads();
            bool loopFlag = false;
            do {
                if ((loopFlag = atomicCAS(&locks[i], 0, 1) == 0)) {
                    a[i] = i > a[i] ? i : a[i];
                }
                __threadfence_system(); //Or __threadfence_block(), __threadfence_system() according to your Memory Fence demand
                if (loopFlag)
                    atomicExch(&locks[i], 0);
            } while (!loopFlag);
            //__syncthreads();
            //bool success = false;
            //while (!success) {
            //    if (atomicExch(&lock[i], 1) == 0) {
            //        a[i] = i > a[i] ? i : a[i];
            //        __threadfence_system();
            //        atomicExch(&lock[i], 0);
            //        success = true;
            //    }
            //}
        }

        if (i < size - 1) {
            //__syncthreads();
            bool loopFlag = false;
            do {
                if ((loopFlag = atomicCAS(&locks[i + 1], 0, 1) == 0)) {
                    a[i + 1] = i > a[i + 1] ? i : a[i + 1];
                }
                __threadfence_system(); //Or __threadfence_block(), __threadfence_system() according to your Memory Fence demand
                if (loopFlag)
                    atomicExch(&locks[i + 1], 0);
            } while (!loopFlag);
            //__syncthreads();
            //bool success = false;
            //while (!success) {
            //    if (atomicExch(&lock[i + 1], 1) == 0) {
            //        a[i + 1] = i > a[i + 1] ? i : a[i + 1];
            //        __threadfence_system();
            //        atomicExch(&lock[i + 1], 0);
            //        success = true;
            //    }
            //}
        }

        i += gridDim.x * blockDim.x;
    }
}

int main()
{
    int size = 10000;
    int* a;
    int* aGPU;
    //int* sum;
    //int* sumGPU;
    int* locks;

    a = new int[size];
    //for (int i = 0; i < size; i++)
    //    a[i] = i + 1;
    //cudaMalloc(&aGPU, sizeof(int) * size);
    //cudaMemcpy(aGPU, a, sizeof(int) * size, cudaMemcpyHostToDevice);

    cudaMalloc(&aGPU, sizeof(int) * size);
    cudaMemset(aGPU, 0, sizeof(int) * size);

    //sum = new int;
    //cudaMalloc(&sumGPU, sizeof(int));
    //cudaMemset(sumGPU, 0, sizeof(int));

    //cudaMalloc(&lock, sizeof(int));
    //cudaMemset(lock, 0, sizeof(int));

    cudaMalloc(&locks, sizeof(int) * size);
    cudaMemset(locks, 0, sizeof(int) * size);

    maxKernel<<<64, 64>>>(size, aGPU, locks);
    cudaError_t cudaStatus = cudaGetLastError();
    if (cudaStatus != cudaSuccess)
        fprintf(stderr, "Kernel launch failed: %s\n", cudaGetErrorString(cudaStatus));

    //cudaMemcpy(sum, sumGPU, sizeof(int), cudaMemcpyDeviceToHost);
    cudaMemcpy(a, aGPU, sizeof(int) * size, cudaMemcpyDeviceToHost);
    for (int i = 0; i < size - 1; i++)
        std::cout << a[i] << ' ';
    std::cout << a[size - 1] << std::endl;

    //std::cout << *sum << std::endl;

    delete[] a;
    //delete sum;
    cudaFree(aGPU);
    //cudaFree(sumGPU);
    cudaFree(locks);

    return 0;
}
