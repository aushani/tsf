#include "depth_buffer_builder.h"

#include <stdio.h>

#include "library/util/util.h"

#include "library/gpu_util/gpu_util.cu.h"

depth_buffer_builder_t*
depth_buffer_builder_create(int height, int width) {

    depth_buffer_builder_t* dbb = (depth_buffer_builder_t*) malloc(sizeof(depth_buffer_builder_t));

    dbb->height = height;
    dbb->width = width;

    cudaMalloc(&dbb->sparse_depth, sizeof(float)*height*width);
    cudaMalloc(&dbb->tmp_1, sizeof(float)*height*width);
    cudaMalloc(&dbb->tmp_2, sizeof(float)*height*width);

    return dbb;
}

void
depth_buffer_builder_destroy(depth_buffer_builder_t *dbb) {

    cudaFree(dbb->sparse_depth);
    cudaFree(dbb->tmp_1);
    cudaFree(dbb->tmp_2);

    free(dbb);

}

__global__ void
depth_buffer_builder_build_kernel(depth_buffer_builder_t dbb) {

    // TODO: Shared memory optimization?

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;

    const int i0 = bidx * blockDim.x;
    const int j0 = bidy * blockDim.y;

    const int i = i0 + tidx;
    const int j = j0 + tidy;

    const int idx = i*dbb.width + j;

    if (dbb.sparse_depth[idx] > 0) {
        dbb.tmp_2[idx] = dbb.sparse_depth[idx];
    } else {

        float old_depth = dbb.tmp_2[idx];

        dbb.tmp_2[idx] = 0;
        int count = 0;

        for (int di=-1; di<=1; di++) {

            int i_n = i + di;
            if (i_n < 0 || i_n >= dbb.height)
                continue;

            for (int dj=-1; dj<=1; dj++) {

                if (di==0 && dj==0)
                    continue;

                int j_n = j + dj;
                if (j_n < 0 || j_n >= dbb.width)
                    continue;

                int idx_n = i_n * dbb.width + j_n;

                if (dbb.tmp_1[idx_n] > 0) {

                    float n_depth = dbb.tmp_1[idx_n];

                    // Check to make sure there isn't an occlusion or something here
                    if ( (fabs(old_depth) < 1e-3) || (old_depth + 1 > n_depth) ) {
                        dbb.tmp_2[idx] += n_depth;
                        count++;
                    }
                }
            }
        }

        if (count > 0)
            dbb.tmp_2[idx] /= count;
        else
            dbb.tmp_2[idx] = 0;
    }

}

depth_buffer_t*
depth_buffer_builder_build(depth_buffer_builder_t *dbb, float *sparse_depth,
        depth_buffer_t *db) {

    int64_t tic_memcpy = utime_now();
    cudaMemcpy(dbb->sparse_depth, sparse_depth, sizeof(float)*dbb->height*dbb->width,
            cudaMemcpyHostToDevice);
    int64_t toc_memcpy = utime_now();
    double t_memcpy = (toc_memcpy - tic_memcpy)/1e3;
    printf("\tTook %5.3f ms to memcpy sparse depth map\n", t_memcpy);

    dim3 threads_dim;
    threads_dim.x = 32;
    threads_dim.y = 32;
    threads_dim.z = 1;

    dim3 blocks_dim;
    blocks_dim.x = dbb->height/32 + 1;
    blocks_dim.y = dbb->width/32 + 1;
    blocks_dim.z = 1;

    // Clear out buffers
    int64_t tic_memset = utime_now();
    cudaMemset(dbb->tmp_1, 0, sizeof(float)*dbb->height*dbb->width);
    cudaMemset(dbb->tmp_2, 0, sizeof(float)*dbb->height*dbb->width);
    int64_t toc_memset = utime_now();
    double t_memset = (toc_memset - tic_memset)/1e3;
    printf("\tTook %5.3f ms to clear out buffers\n", t_memset);

    const int iterations = 10;
    int64_t tic_iter = utime_now();
    for (int i=0; i<iterations; i++) {

        depth_buffer_builder_build_kernel<<<blocks_dim, threads_dim>>>(*dbb);

        // swap buffers
        float *tmp = dbb->tmp_1;
        dbb->tmp_1 = dbb->tmp_2;
        dbb->tmp_2 = tmp;
    }
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_iter = utime_now();
    double t_iter = (toc_iter - tic_iter)/1e3;
    printf("\tTook %5.3f ms to build depth map buffer with %dx%d threads and %dx%d blocks\n",
            t_iter, threads_dim.x, threads_dim.y, blocks_dim.x, blocks_dim.y);

    // Now make depth buffer
    if (db == NULL) {
        db = depth_buffer_create(dbb->height, dbb->width, dbb->tmp_1);
    } else {
        int64_t tic_memcpy = utime_now();
        cudaMemcpy(db->d_depth, dbb->tmp_1, sizeof(float)*dbb->height*dbb->width, cudaMemcpyDeviceToDevice);
        cudaMemcpy(db->depth, db->d_depth, sizeof(float)*db->height*db->width, cudaMemcpyDeviceToHost);
        cudaSafe(cudaDeviceSynchronize());
        int64_t toc_memcpy = utime_now();
        double t_memcpy = (toc_memcpy - tic_memcpy)/1e3;
        printf("\tTook %5.3f ms to memcpy result\n", t_memcpy);
    }

    return db;
}
