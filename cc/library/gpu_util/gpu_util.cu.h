#pragma once
#ifndef _GPU_UTIL_H_
#define _GPU_UTIL_H_

#include <stdio.h>

#ifdef __CUDACC__
#define cudaSafe(ans) { gpuAssert((ans), __FILE__, __LINE__); }
void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true);

struct is_nonnegative {

    __host__ __device__
    bool operator()(const int64_t x) {
        return x>=0L;
    }
};

struct is_negative {

    __host__ __device__
    bool operator()(const int64_t x) {
        return x<0L;
    }
};

struct is_lteps {

    __host__ __device__
    bool operator()(const float x) {
        return fabs(x)<1e-2;
    }
};
#endif

void get_gpu_mem(size_t *free_bytes, size_t *total_bytes);

bool set_device(int device);

#endif
