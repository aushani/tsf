#include "gpu_util.cu.h"
#include "gpu_util.h"
#include "library/util/util.h"

#include <sys/time.h>

#include <boost/assert.hpp>

int64_t utime_now() {
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}

void gpuAssert(cudaError_t code, const char *file, int line, bool abort) {
    if (code != cudaSuccess) {
          fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code),
              file, line);
        if (abort) exit(code);
    }
}

void get_gpu_mem(size_t *free_bytes, size_t *total_bytes) {

    cudaError_t cuda_status = cudaMemGetInfo( free_bytes, total_bytes ) ;

    if ( cudaSuccess != cuda_status ){

        printf("Error: cudaMemGetInfo fails, %s \n", cudaGetErrorString(cuda_status) );

        exit(1);

    }
}

bool set_device(int device) {
  cudaError_t err = cudaSetDevice(device);
  BOOST_ASSERT(err == cudaSuccess);

  return true;
}
