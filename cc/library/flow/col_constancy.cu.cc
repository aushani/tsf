#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "library/util/util.h"
#include "library/gpu_util/gpu_util.cu.h"

#include "library/ray_tracing/sparse_occ_grid.h"

#include "col_constancy.h"

void
conv_create(conv_t *conv, int k, int input_channels, int output_channels, int strides) {

    conv->k = k;

    conv->input_channels = input_channels;
    conv->output_channels = output_channels;

    conv->strides = strides;

    cudaMalloc(&conv->W, sizeof(float)*k*input_channels*output_channels);
    cudaMalloc(&conv->b, sizeof(float)*output_channels);
    cudaSafe(cudaDeviceSynchronize());
}

void
conv_destroy(conv_t conv) {
    cudaFree(conv.W);
    cudaFree(conv.b);
}

__device__ inline float
conv_get_w(conv_t conv, int di, int input_channel, int output_channel) {
    int idx_w = (di*conv.k + input_channel)*conv.output_channels + output_channel;
    return conv.W[idx_w];
}

__device__ inline void
conv_set_w(conv_t conv, int di, int input_channel, int output_channel, float val) {
    int idx_w = (di*conv.k + input_channel)*conv.output_channels + output_channel;
    conv.W[idx_w] = val;
}

__device__ inline float
conv_get_b(conv_t conv, int output_channel) {
    return conv.b[output_channel];
}

__device__ inline void
conv_set_b(conv_t conv, int output_channel, float val) {
    conv.b[output_channel] = val;
}

void
fully_connected_create(fully_connected_t *fc, int input_channels, int output_channels) {

    fc->input_channels = input_channels;
    fc->output_channels = output_channels;

    cudaMalloc(&fc->W, sizeof(float)*input_channels*output_channels);
    cudaMalloc(&fc->b, sizeof(float)*output_channels);
    cudaSafe(cudaDeviceSynchronize());
}

void
fully_connected_destroy(fully_connected_t fc) {
    cudaFree(fc.W);
    cudaFree(fc.b);
}

__device__ inline float
fully_connected_get_w(fully_connected_t fc, int i, int o) {
    int idx = i*fc.output_channels + o;
    return fc.W[idx];
}

__device__ inline void
fully_connected_set_w(fully_connected_t fc, int i, int o, float val) {
    int idx = i*fc.output_channels + o;
    fc.W[idx] = val;
}

__device__ inline float
fully_connected_get_b(fully_connected_t fc, int i) {
    return fc.b[i];
}

__device__ inline void
fully_connected_set_b(fully_connected_t fc, int i, float val) {
    fc.b[i] = val;
}

col_constancy_t*
col_constancy_create(int n_dim[3], int search_size, int threads_per_block) {

    col_constancy_t *cc = (col_constancy_t*) malloc(sizeof(col_constancy_t));

    cc->n_dim[0] = n_dim[0];
    cc->n_dim[1] = n_dim[1];

    cc->search_size = search_size;
    cc->threads_per_block = threads_per_block;

    size_t sz = sizeof(float)*n_dim[0]*n_dim[1]*cc->search_size*cc->search_size;

    int64_t tic_malloc = utime_now();
    if (cudaSuccess != cudaMalloc(&cc->d_cc_scores, sz))
        printf("Error allocating memory for col constancy\n");
    if (cudaSuccess != cudaMalloc(&cc->d_cc_scores_windowed, sz))
        printf("Error allocating memory for col constancy\n");
    cudaSafe(cudaDeviceSynchronize());
    //int64_t toc_malloc = utime_now();
    //double t_malloc = (toc_malloc - tic_malloc)/1e3;
    //printf("Took %5.3f ms to malloc %ld MB for col constancy\n", t_malloc, sz/(1024*1024));

    cc->def_cc_score = n_dim[2]*logf(0.5f);

    float p_min = 1 - 1/(1 + expf(l_min));
    float p_max = 1 - 1/(1 + expf(l_max));
    cc->min_cc_score = n_dim[2]*logf( p_min*p_max  + (1-p_min)*(1-p_max));
    //printf("Default score for column height of %d is %5.3f\n", n_dim[2], cc->def_cc_score);
    //printf("Min.    score for column height of %d is %5.3f\n", n_dim[2], cc->min_cc_score);

    cudaMalloc(&cc->d_wf, sizeof(float)*n_dim[2]);
    cudaMalloc(&cc->d_wo, sizeof(float)*n_dim[2]);
    cudaMalloc(&cc->d_wd, sizeof(float)*n_dim[2]);

    return cc;
}

void
col_constancy_destroy(col_constancy_t *cc) {

    cudaFree(cc->d_cc_scores);
    cudaFree(cc->d_cc_scores_windowed);

    cudaFree(cc->d_wf);
    cudaFree(cc->d_wo);
    cudaFree(cc->d_wd);

    free(cc);
}

__device__ inline float
get_occ_grid_prob(float *d_grid, int i, int j, int k, int n_dim[3], int col_coalesce) {

    // Check bounds
    if (i<0 || i>=n_dim[0] ||
        j<0 || j>=n_dim[1] ||
        k<0 || k>=n_dim[2])
        return 0.5;    // uninformative prior

    int idx=0;
    if (col_coalesce==1)
        idx = (k*n_dim[0] + i)*n_dim[1] + j;
    else
        idx = (i*n_dim[1] + j)*n_dim[2] + k;


    return d_grid[idx];
}

__device__ inline float
score_columns_learned(col_constancy_t cc, col_constancy_params_t p, int i1, int j1,
        int i2, int j2, int d_i, int d_j, float *px_col) {

    float score = cc.b;

    // Walk down column
    for (int k=0; k<p.n_dim[2]; k++) {

        // Load up more p_x's and p_y's
        int s_x = S_UNKN;
        if (px_col[k] < 0.5) s_x = S_FREE;
        if (px_col[k] > 0.5) s_x = S_OCCU;

        float p_y = get_occ_grid_prob(p.d_occ_grid_2, i2, j2, k, p.n_dim, p.use_col_coalesce);
        int s_y = S_UNKN;
        if (p_y < 0.5) s_y = S_FREE;
        if (p_y > 0.5) s_y = S_OCCU;

        float score_p = 0.0;

        if (s_x == S_FREE && s_y == S_FREE) score_p = cc.d_wf[k];
        if (s_x == S_OCCU && s_y == S_OCCU) score_p = cc.d_wo[k];

        if (s_x == S_FREE && s_y == S_OCCU) score_p = cc.d_wd[k];
        if (s_x == S_OCCU && s_y == S_FREE) score_p = cc.d_wd[k];

        score += score_p;
    }

    // Compute distance
    /*
    float x1 = (i1 - p.n_dim[0]/2)*p.res;
    float y1 = (j1 - p.n_dim[1]/2)*p.res;

    float x2 = (i2 - p.n_dim[0]/2)*p.res;
    float y2 = (j2 - p.n_dim[1]/2)*p.res;

    float sh = sinf(cc.x_12[5]);
    float ch = cosf(cc.x_12[5]);

    float x2_p = ch*x1 - sh*y1 - cc.x_12[0];
    float y2_p = sh*x1 + ch*y1 - cc.x_12[1];

    float dx = x2_p - x2;
    float dy = y2_p - y2;
    float d2 = dx*dx + dy*dy;

    score += cc.dist_loss * expf(-d2);
    */

    //float dx = (d_i) * p.res;
    //float dy = (d_j) * p.res;
    //float d2 = dx*dx + dy*dy;
    //if (d2 > cc.max_d2) d2 = cc.max_d2;

    //score += cc.dist_loss * d2;

    float prob = 1.0 / (1.0 + expf(-score));

    return prob;
}

__device__ float
cc_lookup_score(col_constancy_t cc, int i, int j, int d_i, int d_j) {

    const int s2 = cc.search_size/2;

    if (i<0 || i>=cc.n_dim[0] || j<0 || j>=cc.n_dim[0] ||
            d_i<-s2 || d_i>s2 || d_j<-s2 || d_j>s2) {
        //return cc.def_cc_score;
        return cc.min_cc_score;
    }

    //const int n2 = cc.n_dim[0] * cc.n_dim[1];
    const int d2 = cc.search_size * cc.search_size;

    const int ij = i*cc.n_dim[1] + j;

    d_i += cc.search_size/2;
    d_j += cc.search_size/2;

    const int d_ij = d_i * cc.search_size + d_j;

    //const int idx = d_ij*n2 + ij;
    const int idx = ij*d2 + d_ij;

    return cc.d_cc_scores[idx];
}

__global__ void
col_constancy_device_build(col_constancy_t cc, col_constancy_params_t p) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;
    //const int bidz = blockIdx.z;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;
    //const int tidz = threadIdx.z;

    extern __shared__ float px_col[];

    // We can cache the columns and put them in shared memory for better performance!
    // Load shared memory
    int k_load = tidx*cc.search_size + tidy;
    if (k_load < p.n_dim[2]) {
        px_col[k_load] = get_occ_grid_prob(p.d_occ_grid_1, bidx, bidy, k_load, p.n_dim, p.use_col_coalesce);
    }

    __syncthreads();

    // Check is empty
    int is_empty = 1;
    for (int k=0; k<p.n_dim[2] && is_empty; k++)
        if (px_col[k] > 0.6)
            is_empty = 0;

    // Get the (di, dj) we need to evaluate
    const int d_i = tidx - (cc.search_size/2);
    const int d_j = tidy - (cc.search_size/2);

    float prob = 0.5;
    if (!is_empty) {
        prob = score_columns_learned(cc, p, bidx, bidy, bidx + d_i, bidy + d_j, d_i, d_j, px_col);
    }

    // Make into log prob
    float log_prob = logf(prob);

    int idx_ij = bidx*cc.n_dim[1] + bidy;
    int idx_d = tidx*cc.search_size + tidy;

    int s2 = cc.search_size * cc.search_size;
    //int n2 = cc.n_dim[0] * cc.n_dim[1];
    //int idx = idx_d * n2 + idx_ij;
    int idx = idx_ij*s2 + idx_d;

    cc.d_cc_scores[idx] = log_prob;
}

__global__ void
col_constancy_apply_window(col_constancy_t cc, col_constancy_params_t p) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;
    //const int bidz = blockIdx.z;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;
    //const int tidz = threadIdx.z;

    const int d_i = tidx - cc.search_size/2;
    const int d_j = tidy - cc.search_size/2;

    float sum = 0.0;

    for (int iw=bidx-p.patch_window; iw<=bidx+p.patch_window; iw++) {
        for (int jw=bidy-p.patch_window; jw<=bidy+p.patch_window; jw++) {
            float this_col = cc_lookup_score(cc, iw, jw, d_i, d_j);
            sum += this_col;
        }
    }

    const int d_ij = tidx * cc.search_size + tidy;
    const int ij = bidx * cc.n_dim[1] + bidy;

    const int n2 = cc.n_dim[0] * cc.n_dim[1];

    const int idx_w = d_ij*n2 + ij;
    cc.d_cc_scores_windowed[idx_w] = sum;
}

void
col_constancy_build(col_constancy_t *cc, col_constancy_params_t p) {

    dim3 threads_dim;
    threads_dim.x = cc->search_size;
    threads_dim.y = cc->search_size;
    threads_dim.z = 1;

    dim3 blocks_dim;
    blocks_dim.x = cc->n_dim[0];
    blocks_dim.y = cc->n_dim[1];
    blocks_dim.z = 1;

    int shared_memory_dim = p.n_dim[2];
    size_t shared_memory_sz = shared_memory_dim * sizeof(float);

    int64_t tic_build = utime_now();
    col_constancy_device_build<<<blocks_dim, threads_dim, shared_memory_sz>>>(*cc, p);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_build = utime_now();
    double t_build = (toc_build - tic_build)/1e3;
    printf("\tTook %5.3f ms to build %dx%d %dx%d column constancy using:\n",
            t_build, cc->n_dim[0], cc->n_dim[1], cc->search_size, cc->search_size);

    printf("\t\t%dx%dx%d threads and %dx%dx%d blocks and %ld bytes of shared memory\n",
            threads_dim.x, threads_dim.y, threads_dim.z,
            blocks_dim.x, blocks_dim.y, blocks_dim.z,
            shared_memory_sz);

    int64_t tic_window = utime_now();
    col_constancy_apply_window<<<blocks_dim, threads_dim>>>(*cc, p);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_window = utime_now();
    double t_window = (toc_window - tic_window)/1e3;
    printf("\tTook %5.3f ms to apply %d window with:\n", t_window, p.patch_window);
    printf("\t\t%dx%dx%d threads and %dx%dx%d blocks\n",
            threads_dim.x, threads_dim.y, threads_dim.z,
            blocks_dim.x, blocks_dim.y, blocks_dim.z);
}
