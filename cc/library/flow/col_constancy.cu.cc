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

// forward declaration
__global__ void
tf_matcher_init(conv_t conv, fully_connected_t fc);

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

void
col_constancy_init_tf(col_constancy_t *cc) {

    // Make the tf matcher

    // hard coded
    int k = 3;
    int strides = 1;
    int dim = 14;

    int conv_input_channels = 2; // 2 occ grids
    int conv_output_channels = 8;
    int fc_input_channels = conv_output_channels;
    int fc_output_channels = 2;

    conv_create(&cc->conv_layer, k, conv_input_channels, conv_output_channels, strides);
    fully_connected_create(&cc->fc_layer, dim*fc_input_channels, fc_output_channels);

    // init (this is ugly)
    tf_matcher_init<<<1, 1>>>(cc->conv_layer, cc->fc_layer);
    cudaSafe(cudaDeviceSynchronize());
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

    col_constancy_init_tf(cc);

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
score_columns_tf(col_constancy_t cc, col_constancy_params_t p, int i1, int j1, int
        i2, int j2, int d_i, int d_j, float *px_col) {

    // Do convolutional layer
    int n_conv_output = cc.conv_layer.output_channels*p.n_dim[2];
    //float *conv_output = (float*) malloc(sizeof(float)*n_conv_output);
    float conv_output[112]; // hard coded works, not sure why

    const int k1_2 = cc.conv_layer.k/2;

    for (int k=0; k<p.n_dim[2]; ++k) {

        for (int output_channel=0; output_channel<cc.conv_layer.output_channels; ++output_channel) {

            float sum = conv_get_b(cc.conv_layer, output_channel);

            for (int dk = 0; dk<cc.conv_layer.k; ++dk) {

                for (int input_channel=0; input_channel<cc.conv_layer.input_channels; ++input_channel) {

                    float w = conv_get_w(cc.conv_layer, dk, input_channel, output_channel);

                    int kk = cc.conv_layer.strides*k + dk - k1_2;

                    float v = 0;

                    if (kk >= 0 && kk<p.n_dim[2]) {
                        if (input_channel == 0) {
                            v = px_col[kk];
                        } else if (input_channel == 1) {
                            v = get_occ_grid_prob(p.d_occ_grid_2, i2, j2, kk, p.n_dim, p.use_col_coalesce);
                        }
                    }


                    sum += v * w;
                }
            }

            // Do the RELU here
            float res = sum;
            if (res < 0.0) res = 0.0;

            int idx = k*cc.conv_layer.output_channels + output_channel;
            conv_output[idx] = res;
        }
    }

    // Now do fully connected layer
    int n_fc_output = cc.fc_layer.output_channels;
    //float *fc_output = (float*) malloc(sizeof(float)*n_fc_output);
    float fc_output[2]; // again, hard coded, not sure why

    for (int output_channel=0; output_channel<cc.fc_layer.output_channels; output_channel++) {

        float sum = fully_connected_get_b(cc.fc_layer, output_channel);

        int idx_fc = 0;

        for (int k=0; k<p.n_dim[2]; k++) {
            for (int ic=0; ic<cc.conv_layer.output_channels; ic++) {

                int idx = k*cc.conv_layer.output_channels + ic;
                float v = conv_output[idx];

                float w = fully_connected_get_w(cc.fc_layer, idx_fc++, output_channel);

                sum += v*w;
            }
        }

        fc_output[output_channel] = sum;
    }
    //free(conv_output);

    // Convert to prob using softmax
    // (hack, we know we have two output channels)
    float exp0 = expf(fc_output[0]);
    float exp1 = expf(fc_output[1]);

    float prob = exp1 / (exp0 + exp1);

    //free(fc_output);

    return prob;
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
        prob = score_columns_tf(cc, p, bidx, bidy, bidx + d_i, bidy + d_j, d_i, d_j, px_col);
        //prob = score_columns_learned(cc, p, bidx, bidy, bidx + d_i, bidy + d_j, d_i, d_j, px_col);
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

// THIS IS SO UGLY BUT I'M LAZY
__global__ void
tf_matcher_init(conv_t conv, fully_connected_t fc) {

    /*
    occ_input_w
    [[[[-0.05238699  0.39667225 -0.04053304 -0.60537761 -0.26478422 -0.08277871
         0.          0.        ]
       [ 0.1974607   0.46566218  0.0091867   0.60418302 -1.08609354  0.26559097
         0.          0.        ]]]


     [[[ 0.66306502  0.30313012 -1.04062402 -0.731435   -0.04492672  0.73547232
         0.          0.        ]
       [-0.6809513   0.25567245  1.12375426  0.78725463  1.08691525 -0.74871194
         0.          0.        ]]]


     [[[ 0.09695023  0.21317133 -0.0172855   0.1216329  -0.03809703 -0.00944418
         0.          0.        ]
       [-0.1119959  -0.00757185 -0.05846857 -0.10538616  0.48850417  0.03932838
         0.          0.        ]]]]
    */

    conv_set_w(conv, 0, 0, 0,  0.05238699);
    conv_set_w(conv, 0, 0, 1,  0.39667225);
    conv_set_w(conv, 0, 0, 2, -0.04053304);
    conv_set_w(conv, 0, 0, 3, -0.60537761);
    conv_set_w(conv, 0, 0, 4, -0.26478422);
    conv_set_w(conv, 0, 0, 5, -0.08277871);
    conv_set_w(conv, 0, 0, 6,  0.        );
    conv_set_w(conv, 0, 0, 7,  0.        );

    conv_set_w(conv, 0, 1, 0,  0.1974607 );
    conv_set_w(conv, 0, 1, 1,  0.46566218);
    conv_set_w(conv, 0, 1, 2,  0.0091867 );
    conv_set_w(conv, 0, 1, 3,  0.60418302);
    conv_set_w(conv, 0, 1, 4, -1.08609354);
    conv_set_w(conv, 0, 1, 5,  0.26559097);
    conv_set_w(conv, 0, 1, 6,  0.        );
    conv_set_w(conv, 0, 1, 7,  0.        );


    conv_set_w(conv, 1, 0, 0,  0.66306502);
    conv_set_w(conv, 1, 0, 1,  0.30313012);
    conv_set_w(conv, 1, 0, 2, -1.04062402);
    conv_set_w(conv, 1, 0, 3, -0.731435  );
    conv_set_w(conv, 1, 0, 4, -0.04492672);
    conv_set_w(conv, 1, 0, 5,  0.73547232);
    conv_set_w(conv, 1, 0, 6,  0.        );
    conv_set_w(conv, 1, 0, 7,  0.        );

    conv_set_w(conv, 1, 1, 0, -0.6809513 );
    conv_set_w(conv, 1, 1, 1,  0.25567245);
    conv_set_w(conv, 1, 1, 2,  1.12375426);
    conv_set_w(conv, 1, 1, 3,  0.78725463);
    conv_set_w(conv, 1, 1, 4,  1.08691525);
    conv_set_w(conv, 1, 1, 5, -0.74871194);
    conv_set_w(conv, 1, 1, 6,  0.        );
    conv_set_w(conv, 1, 1, 7,  0.        );


    conv_set_w(conv, 2, 0, 0,  0.09695023);
    conv_set_w(conv, 2, 0, 1,  0.21317133);
    conv_set_w(conv, 2, 0, 2, -0.0172855 );
    conv_set_w(conv, 2, 0, 3,  0.1216329 );
    conv_set_w(conv, 2, 0, 4, -0.03809703);
    conv_set_w(conv, 2, 0, 5, -0.00944418);
    conv_set_w(conv, 2, 0, 6,  0.        );
    conv_set_w(conv, 2, 0, 7,  0.        );

    conv_set_w(conv, 2, 1, 0, -0.1119959 );
    conv_set_w(conv, 2, 1, 1, -0.00757185);
    conv_set_w(conv, 2, 1, 2, -0.05846857);
    conv_set_w(conv, 2, 1, 3, -0.10538616);
    conv_set_w(conv, 2, 1, 4,  0.48850417);
    conv_set_w(conv, 2, 1, 5,  0.03932838);
    conv_set_w(conv, 2, 1, 6,  0.        );
    conv_set_w(conv, 2, 1, 7,  0.        );

    /*
    occ_input_b [ -3.50655848e-03   1.88007876e-01  -1.33862792e-04  -7.61228148e-03
      -5.19128852e-02  -1.60278194e-02   0.00000000e+00   0.00000000e+00]
    */

    conv_set_b(conv, 0, -3.50655848e-03);
    conv_set_b(conv, 1,  1.88007876e-01);
    conv_set_b(conv, 2, -1.33862792e-04);
    conv_set_b(conv, 3, -7.61228148e-03);
    conv_set_b(conv, 4, -5.19128852e-02);
    conv_set_b(conv, 5, -1.60278194e-02);
    conv_set_b(conv, 6,  0.00000000e+00);
    conv_set_b(conv, 7,  0.00000000e+00);

    /*
    occ_decision_layer_w
    [[ 0.23055148 -0.23055549]
     [-0.17655812  0.1765593 ]
     [ 0.097316   -0.08147407]
     [ 0.02658524 -0.10436169]
     [-0.18051893  0.18051656]
     [ 0.15523562 -0.15523504]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [-0.06835665  0.06836928]
     [ 0.02442657 -0.02477191]
     [ 0.76893502 -0.66464442]
     [ 0.07437789 -0.07214063]
     [-1.51986754  1.519853  ]
     [ 0.2089514  -0.20895067]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.17635991 -0.17636234]
     [ 0.12657994 -0.12157987]
     [ 0.78308678 -0.96233511]
     [-0.1345709   0.13387072]
     [-1.30331719  1.30339038]
     [ 0.23243228 -0.23243393]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.49753276 -0.49641407]
     [ 0.09169663 -0.03542714]
     [ 0.80073446 -0.86725426]
     [ 0.20953523 -0.20456129]
     [-1.14051867  1.15788651]
     [ 0.46869844 -0.46870568]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.51855922 -0.5204255 ]
     [-0.43345392  0.44993371]
     [ 0.56904346 -0.62977618]
     [ 0.259146   -0.26155755]
     [-1.3046608   1.31153691]
     [ 0.48707306 -0.48689514]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.47603279 -0.47722766]
     [-0.29000625  0.28921884]
     [ 0.59100264 -0.49500045]
     [ 0.21292694 -0.19828264]
     [-1.43880498  1.41369593]
     [ 0.47358868 -0.47350606]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.4945679  -0.49380305]
     [-0.13637798  0.12026031]
     [ 0.64324647 -0.57366365]
     [ 0.34865791 -0.36276293]
     [-1.45976043  1.46049857]
     [ 0.60218173 -0.60206121]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.4694854  -0.46926066]
     [-0.11278735  0.11212205]
     [ 0.45006114 -0.4356305 ]
     [ 0.52503502 -0.46287963]
     [-1.32922089  1.31629515]
     [ 0.60757232 -0.60802031]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.51732713 -0.52467865]
     [-0.2371085   0.25541306]
     [ 0.67027158 -0.66316539]
     [ 0.57913798 -0.56751275]
     [-1.27866173  1.23537886]
     [ 0.78210109 -0.78214949]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.57486641 -0.57740504]
     [-0.28352582  0.29809323]
     [ 0.63211554 -0.61152864]
     [ 0.56790078 -0.52136528]
     [-1.30135047  1.30351961]
     [ 0.82686806 -0.82896519]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.64024889 -0.6429553 ]
     [-0.25769201  0.27263176]
     [ 0.70768476 -0.58808178]
     [ 0.72712737 -0.64096338]
     [-1.25509429  1.22173953]
     [ 0.92576754 -0.92590392]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.59126812 -0.58872342]
     [-0.12911107  0.12997237]
     [ 0.46636218 -0.56768084]
     [ 0.8323614  -0.85039616]
     [-1.12847579  1.12717271]
     [ 0.93855011 -0.93854719]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.53505456 -0.53722399]
     [ 0.16535719 -0.16534896]
     [ 0.43428287 -0.40366876]
     [ 0.92427903 -0.98100859]
     [-0.7389425   0.73871505]
     [ 1.02538097 -1.02539122]
     [ 0.          0.        ]
     [ 0.          0.        ]
     [ 0.92651099 -0.91819316]
     [ 0.30241197 -0.34039894]
     [ 1.04700828 -1.10025704]
     [ 1.39115918 -1.2985127 ]
     [-0.22002417  0.2199893 ]
     [ 1.22034764 -1.22020578]
     [ 0.          0.        ]
     [ 0.          0.        ]]
    */

    int ic = 0;

    fully_connected_set_w(fc, ic,   0,  0.23055148);
    fully_connected_set_w(fc, ic++, 1, -0.23055549);

    fully_connected_set_w(fc, ic,   0, -0.17655812);
    fully_connected_set_w(fc, ic++, 1,  0.1765593);

    fully_connected_set_w(fc, ic,   0,  0.097316  );
    fully_connected_set_w(fc, ic++, 1, -0.08147407);

    fully_connected_set_w(fc, ic,   0,  0.02658524);
    fully_connected_set_w(fc, ic++, 1, -0.10436169);

    fully_connected_set_w(fc, ic,   0, -0.18051893);
    fully_connected_set_w(fc, ic++, 1,  0.18051656);

    fully_connected_set_w(fc, ic,   0,  0.15523562);
    fully_connected_set_w(fc, ic++, 1, -0.15523504);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0, -0.06835665);
    fully_connected_set_w(fc, ic++, 1,  0.06836928);

    fully_connected_set_w(fc, ic,   0,  0.02442657);
    fully_connected_set_w(fc, ic++, 1, -0.02477191);

    fully_connected_set_w(fc, ic,   0,  0.76893502);
    fully_connected_set_w(fc, ic++, 1, -0.66464442);

    fully_connected_set_w(fc, ic,   0,  0.07437789);
    fully_connected_set_w(fc, ic++, 1, -0.07214063);

    fully_connected_set_w(fc, ic,   0, -1.51986754);
    fully_connected_set_w(fc, ic++, 1,  1.519853);

    fully_connected_set_w(fc, ic,   0,  0.2089514 );
    fully_connected_set_w(fc, ic++, 1, -0.20895067);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.17635991);
    fully_connected_set_w(fc, ic++, 1, -0.17636234);

    fully_connected_set_w(fc, ic,   0,  0.12657994);
    fully_connected_set_w(fc, ic++, 1, -0.12157987);

    fully_connected_set_w(fc, ic,   0,  0.78308678);
    fully_connected_set_w(fc, ic++, 1, -0.96233511);

    fully_connected_set_w(fc, ic,   0, -0.1345709 );
    fully_connected_set_w(fc, ic++, 1,  0.13387072);

    fully_connected_set_w(fc, ic,   0, -1.30331719);
    fully_connected_set_w(fc, ic++, 1,  1.30339038);

    fully_connected_set_w(fc, ic,   0,  0.23243228);
    fully_connected_set_w(fc, ic++, 1, -0.23243393);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.49753276);
    fully_connected_set_w(fc, ic++, 1, -0.49641407);

    fully_connected_set_w(fc, ic,   0,  0.09169663);
    fully_connected_set_w(fc, ic++, 1, -0.03542714);

    fully_connected_set_w(fc, ic,   0,  0.80073446);
    fully_connected_set_w(fc, ic++, 1, -0.86725426);

    fully_connected_set_w(fc, ic,   0,  0.20953523);
    fully_connected_set_w(fc, ic++, 1, -0.20456129);

    fully_connected_set_w(fc, ic,   0, -1.14051867);
    fully_connected_set_w(fc, ic++, 1,  1.15788651);

    fully_connected_set_w(fc, ic,   0,  0.46869844);
    fully_connected_set_w(fc, ic++, 1, -0.46870568);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.51855922);
    fully_connected_set_w(fc, ic++, 1, -0.5204255);

    fully_connected_set_w(fc, ic,   0, -0.43345392);
    fully_connected_set_w(fc, ic++, 1,  0.44993371);

    fully_connected_set_w(fc, ic,   0,  0.56904346);
    fully_connected_set_w(fc, ic++, 1, -0.62977618);

    fully_connected_set_w(fc, ic,   0,  0.259146  );
    fully_connected_set_w(fc, ic++, 1, -0.26155755);

    fully_connected_set_w(fc, ic,   0, -1.3046608 );
    fully_connected_set_w(fc, ic++, 1,  1.31153691);

    fully_connected_set_w(fc, ic,   0,  0.48707306);
    fully_connected_set_w(fc, ic++, 1, -0.48689514);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.47603279);
    fully_connected_set_w(fc, ic++, 1, -0.47722766);

    fully_connected_set_w(fc, ic,   0, -0.29000625);
    fully_connected_set_w(fc, ic++, 1,  0.28921884);

    fully_connected_set_w(fc, ic,   0,  0.59100264);
    fully_connected_set_w(fc, ic++, 1, -0.49500045);

    fully_connected_set_w(fc, ic,   0,  0.21292694);
    fully_connected_set_w(fc, ic++, 1, -0.19828264);

    fully_connected_set_w(fc, ic,   0, -1.43880498);
    fully_connected_set_w(fc, ic++, 1,  1.41369593);

    fully_connected_set_w(fc, ic,   0,  0.47358868);
    fully_connected_set_w(fc, ic++, 1, -0.47350606);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.4945679 );
    fully_connected_set_w(fc, ic++, 1, -0.49380305);

    fully_connected_set_w(fc, ic,   0, -0.13637798);
    fully_connected_set_w(fc, ic++, 1,  0.12026031);

    fully_connected_set_w(fc, ic,   0,  0.64324647);
    fully_connected_set_w(fc, ic++, 1, -0.57366365);

    fully_connected_set_w(fc, ic,   0,  0.34865791);
    fully_connected_set_w(fc, ic++, 1, -0.36276293);

    fully_connected_set_w(fc, ic,   0, -1.45976043);
    fully_connected_set_w(fc, ic++, 1,  1.46049857);

    fully_connected_set_w(fc, ic,   0,  0.60218173);
    fully_connected_set_w(fc, ic++, 1, -0.60206121);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.4694854 );
    fully_connected_set_w(fc, ic++, 1, -0.46926066);

    fully_connected_set_w(fc, ic,   0, -0.11278735);
    fully_connected_set_w(fc, ic++, 1,  0.11212205);

    fully_connected_set_w(fc, ic,   0,  0.45006114);
    fully_connected_set_w(fc, ic++, 1, -0.4356305);

    fully_connected_set_w(fc, ic,   0,  0.52503502);
    fully_connected_set_w(fc, ic++, 1, -0.46287963);

    fully_connected_set_w(fc, ic,   0, -1.32922089);
    fully_connected_set_w(fc, ic++, 1,  1.31629515);

    fully_connected_set_w(fc, ic,   0,  0.60757232);
    fully_connected_set_w(fc, ic++, 1, -0.60802031);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.51732713);
    fully_connected_set_w(fc, ic++, 1, -0.52467865);

    fully_connected_set_w(fc, ic,   0, -0.2371085 );
    fully_connected_set_w(fc, ic++, 1,  0.25541306);

    fully_connected_set_w(fc, ic,   0,  0.67027158);
    fully_connected_set_w(fc, ic++, 1, -0.66316539);

    fully_connected_set_w(fc, ic,   0,  0.57913798);
    fully_connected_set_w(fc, ic++, 1, -0.56751275);

    fully_connected_set_w(fc, ic,   0, -1.27866173);
    fully_connected_set_w(fc, ic++, 1,  1.23537886);

    fully_connected_set_w(fc, ic,   0,  0.78210109);
    fully_connected_set_w(fc, ic++, 1, -0.78214949);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.57486641);
    fully_connected_set_w(fc, ic++, 1, -0.57740504);

    fully_connected_set_w(fc, ic,   0, -0.28352582);
    fully_connected_set_w(fc, ic++, 1,  0.29809323);

    fully_connected_set_w(fc, ic,   0,  0.63211554);
    fully_connected_set_w(fc, ic++, 1, -0.61152864);

    fully_connected_set_w(fc, ic,   0,  0.56790078);
    fully_connected_set_w(fc, ic++, 1, -0.52136528);

    fully_connected_set_w(fc, ic,   0, -1.30135047);
    fully_connected_set_w(fc, ic++, 1,  1.30351961);

    fully_connected_set_w(fc, ic,   0,  0.82686806);
    fully_connected_set_w(fc, ic++, 1, -0.82896519);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.64024889);
    fully_connected_set_w(fc, ic++, 1, -0.6429553);

    fully_connected_set_w(fc, ic,   0, -0.25769201);
    fully_connected_set_w(fc, ic++, 1,  0.27263176);

    fully_connected_set_w(fc, ic,   0,  0.70768476);
    fully_connected_set_w(fc, ic++, 1, -0.58808178);

    fully_connected_set_w(fc, ic,   0,  0.72712737);
    fully_connected_set_w(fc, ic++, 1, -0.64096338);

    fully_connected_set_w(fc, ic,   0, -1.25509429);
    fully_connected_set_w(fc, ic++, 1,  1.22173953);

    fully_connected_set_w(fc, ic,   0,  0.92576754);
    fully_connected_set_w(fc, ic++, 1, -0.92590392);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.59126812);
    fully_connected_set_w(fc, ic++, 1, -0.58872342);

    fully_connected_set_w(fc, ic,   0, -0.12911107);
    fully_connected_set_w(fc, ic++, 1,  0.12997237);

    fully_connected_set_w(fc, ic,   0,  0.46636218);
    fully_connected_set_w(fc, ic++, 1, -0.56768084);

    fully_connected_set_w(fc, ic,   0,  0.8323614 );
    fully_connected_set_w(fc, ic++, 1, -0.85039616);

    fully_connected_set_w(fc, ic,   0, -1.12847579);
    fully_connected_set_w(fc, ic++, 1,  1.12717271);

    fully_connected_set_w(fc, ic,   0,  0.93855011);
    fully_connected_set_w(fc, ic++, 1, -0.93854719);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.53505456);
    fully_connected_set_w(fc, ic++, 1, -0.53722399);

    fully_connected_set_w(fc, ic,   0,  0.16535719);
    fully_connected_set_w(fc, ic++, 1, -0.16534896);

    fully_connected_set_w(fc, ic,   0,  0.43428287);
    fully_connected_set_w(fc, ic++, 1, -0.40366876);

    fully_connected_set_w(fc, ic,   0,  0.92427903);
    fully_connected_set_w(fc, ic++, 1, -0.98100859);

    fully_connected_set_w(fc, ic,   0, -0.7389425 );
    fully_connected_set_w(fc, ic++, 1,  0.73871505);

    fully_connected_set_w(fc, ic,   0,  1.02538097);
    fully_connected_set_w(fc, ic++, 1, -1.02539122);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.92651099);
    fully_connected_set_w(fc, ic++, 1, -0.91819316);

    fully_connected_set_w(fc, ic,   0,  0.30241197);
    fully_connected_set_w(fc, ic++, 1, -0.34039894);

    fully_connected_set_w(fc, ic,   0,  1.04700828);
    fully_connected_set_w(fc, ic++, 1, -1.10025704);

    fully_connected_set_w(fc, ic,   0,  1.39115918);
    fully_connected_set_w(fc, ic++, 1, -1.2985127);

    fully_connected_set_w(fc, ic,   0, -0.22002417);
    fully_connected_set_w(fc, ic++, 1,  0.2199893);

    fully_connected_set_w(fc, ic,   0,  1.22034764);
    fully_connected_set_w(fc, ic++, 1, -1.22020578);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    fully_connected_set_w(fc, ic,   0,  0.        );
    fully_connected_set_w(fc, ic++, 1,  0.);

    //occ_decision_layer_b [ 0.03614965  0.07003767]
    fully_connected_set_b(fc, 0, 0.03614965);
    fully_connected_set_b(fc, 1, 0.07003767);

}
