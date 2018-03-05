#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "library/util/util.h"
#include "library/gpu_util/gpu_util.cu.h"

#include "flow_solver.h"
#include "col_constancy.h"

#define USE_COLUMN_COALESCE 1

#define THREADS_DIM 16

flow_solver_t*
flow_solver_create(int max_n_dim[3], int iterations, float smoothing_weight, int threads) {

    flow_solver_t *solver = (flow_solver_t*) malloc(sizeof(flow_solver_t));

    int max_n2 = max_n_dim[0] * max_n_dim[1];
    int max_n3 = max_n2 * max_n_dim[2];

    int64_t tic_alloc = utime_now();
    if (cudaSuccess != cudaMalloc(&solver->d_occ_grid_1, sizeof(float)*max_n3))
            printf("Error allocating memory for obs flow 3d solver\n");
    if (cudaSuccess != cudaMalloc(&solver->d_occ_grid_2, sizeof(float)*max_n3))
            printf("Error allocating memory for obs flow 3d solver\n");

    if (cudaSuccess != cudaMalloc(&solver->d_flow_u, sizeof(int32_t)*max_n2))
            printf("Error allocating memory for obs flow 3d solver\n");
    if (cudaSuccess != cudaMalloc(&solver->d_flow_v, sizeof(int32_t)*max_n2))
            printf("Error allocating memory for obs flow 3d solver\n");
    if (cudaSuccess != cudaMalloc(&solver->d_flow_valid, sizeof(int32_t)*max_n2))
            printf("Error allocating memory for obs flow 3d solver\n");

    if (cudaSuccess != cudaMalloc(&solver->d_flow_score, sizeof(float)*max_n2))
            printf("Error allocating memory for obs flow 3d solver\n");

    if (cudaSuccess != cudaMalloc(&solver->d_flow_score_to, sizeof(float)*max_n2))
            printf("Error allocating memory for obs flow 3d solver\n");
    if (cudaSuccess != cudaMalloc(&solver->d_flow_score_to_valid, sizeof(float)*max_n2))
            printf("Error allocating memory for obs flow 3d solver\n");

    int64_t toc_alloc = utime_now();
    double t_alloc = (toc_alloc - tic_alloc)/1e3;
    int64_t bytes = sizeof(float) * max_n3 * 2;
    bytes += sizeof(int32_t) * max_n2 * 6;
    printf("Took %5.3f ms to alloc %ld MBytes of memory for obs flow solver\n", t_alloc, bytes/(1024*1024));

    solver->threads_per_block = threads;

    solver->em_iterations = iterations;
    solver->smoothing_weight = smoothing_weight;

    return solver;
}

void
flow_solver_destroy(flow_solver_t *solver) {

    // Free everything
    cudaFree(solver->d_occ_grid_1);
    cudaFree(solver->d_occ_grid_2);

    cudaFree(solver->d_flow_u);
    cudaFree(solver->d_flow_v);
    cudaFree(solver->d_flow_valid);

    cudaFree(solver->d_flow_score);
    cudaFree(solver->d_flow_score_to);
    cudaFree(solver->d_flow_score_to_valid);

    free(solver);
}

__device__ inline float
get_occ_grid_prob(float *d_grid, int i, int j, int k, int n_dim[3], int col_coalesce) {

    // Check bounds
    if (i<0 || i>=n_dim[0] ||
        j<0 || j>=n_dim[1] ||
        k<0 || k>=n_dim[2])
        return 0.5;

    int idx=0;
    if (col_coalesce==1)
        idx = (k*n_dim[0] + i)*n_dim[1] + j;
    else
        idx = (i*n_dim[1] + j)*n_dim[2] + k;


    return d_grid[idx];
}

__device__ inline void
get_flow_col(flow_solver_t solver, int i, int j, int *u, int *v, int *valid) {

    // Check bounds
    if ( i<0 || i>=solver.n_dim[0] || j<0 || j>=solver.n_dim[1]) {
        *u = 0;
        *v = 0;
        *valid = 0;
        return;
    }

    int idx = i*solver.n_dim[1] + j;

    *u = solver.d_flow_u[idx];
    *v = solver.d_flow_v[idx];

    *valid = solver.d_flow_valid[idx];
}

__device__ inline float
lookup_score(col_constancy_t cc, int i, int j, int d_i, int d_j) {

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

__device__ inline float
lookup_score_windowed(col_constancy_t cc, int i, int j, int d_i, int d_j) {

    const int s2 = cc.search_size/2;

    if (i<0 || i>=cc.n_dim[0] || j<0 || j>=cc.n_dim[0] ||
            d_i<-s2 || d_i>s2 || d_j<-s2 || d_j>s2) {
        //return cc.def_cc_score;
        return cc.min_cc_score;
    }

    const int n2 = cc.n_dim[0] * cc.n_dim[1];
    //const int d2 = cc.search_size * cc.search_size;

    const int ij = i*cc.n_dim[1] + j;

    d_i += cc.search_size/2;
    d_j += cc.search_size/2;

    const int d_ij = d_i * cc.search_size + d_j;

    const int idx = d_ij*n2 + ij;

    return cc.d_cc_scores_windowed[idx];
}

__device__ inline void
score_smoothness_init_terms(flow_solver_opt_t opt, int i, int j,
        int *sum_u2, int *sum_v2, int *sum_u, int *sum_v, int *count,
        int i0_n, int j0_n, int32_t *flow_s_u, int32_t *flow_s_v, int32_t *flow_s_valid, int flow_s_size_y) {

    *sum_u2 = 0;
    *sum_v2 = 0;

    *sum_u = 0;
    *sum_v = 0;

    *count = 0;

    int i_n, j_n, idx_n;
    int32_t u_n, v_n, valid_n;

    for (int di=-opt.smoothing_window; di<=opt.smoothing_window; di++) {

        i_n = i + di;

        for (int dj=-opt.smoothing_window; dj<=opt.smoothing_window; dj++) {

            if (di==0 && dj==0)
                continue;

            j_n = j + dj;

            idx_n = (i_n - i0_n)*flow_s_size_y + (j_n - j0_n);
            u_n = flow_s_u[idx_n];
            v_n = flow_s_v[idx_n];
            valid_n = flow_s_valid[idx_n];

            if (valid_n) {
                *sum_u2 += u_n*u_n;
                *sum_v2 += v_n*v_n;

                *sum_u += u_n;
                *sum_v += v_n;

                (*count)++;
            }
        }
    }
}

__device__ inline float
score_smoothness_shared(flow_solver_opt_t opt, int i, int j, int u, int v,
        int i0_n, int j0_n, int32_t *flow_s_u, int32_t *flow_s_v, int32_t *flow_s_valid, int flow_s_size_y) {

    float smoothness_score=0.0;

    int i_n, j_n, idx_n;
    int32_t u_n, v_n, valid_n;

    for (int di=-opt.smoothing_window; di<=opt.smoothing_window; di++) {

        i_n = i + di;

        for (int dj=-opt.smoothing_window; dj<=opt.smoothing_window; dj++) {

            if (di==0 && dj==0)
                continue;

            j_n = j + dj;

            // flow_s_valid should take care of this
            //if (i_n>=0 && i_n<solver.n_dim[0] && j_n>=0 && j_n<solver.n_dim[1]) {

                idx_n = (i_n - i0_n)*flow_s_size_y + (j_n - j0_n);
                u_n = flow_s_u[idx_n];
                v_n = flow_s_v[idx_n];
                valid_n = flow_s_valid[idx_n];

                if (valid_n) {
                    int du = u - u_n;
                    int dv = v - v_n;
                    int d2 = du*du + dv*dv;

                    // robust cost functoin
                    if (d2>4)
                        d2 = 4;

                    smoothness_score += -d2;
                }
            //}
        }
    }

    return smoothness_score;
}

__device__ void
flow_solver_load_res_sm4(flow_solver_t solver, flow_solver_opt_t opt, int i, int j,
        int i0_n, int j0_n, int s_sz_x, int s_sz_y, int jump_size,
        int32_t *flow_s_u, int32_t *flow_s_v, int32_t *flow_s_valid, float
        *flow_s_score, float *flow_s_score_to, int32_t *flow_s_score_to_valid) {

    for (int di=-1; di<=1; di+=2) {

        // Where are we loading this time for shared memory?
        int i_n = i + di*jump_size;

        // Make sure this is in range before we do anything
        if (i_n<0 || i_n>=solver.n_dim[0])
            continue;

        for (int dj=-1; dj<=1; dj+=2) {

            // Where are we loading this time for shared memory?
            int j_n = j + dj*jump_size;

            // Make sure this is in range before we do anything
            if (j_n<0 || j_n>=solver.n_dim[1])
                continue;

            // Get index in our shared memory
            int idx_n = (i_n - i0_n)*s_sz_y + (j_n - j0_n);

            // Get index in global memory
            int idx = i_n*solver.n_dim[1] + j_n;

            // Now load from global memory into shared memory
            if (flow_s_u)              flow_s_u[idx_n]              = solver.d_flow_u[idx];
            if (flow_s_v)              flow_s_v[idx_n]              = solver.d_flow_v[idx];
            if (flow_s_valid)          flow_s_valid[idx_n]          = solver.d_flow_valid[idx];
            if (flow_s_score)          flow_s_score[idx_n]          = solver.d_flow_score[idx];
            if (flow_s_score_to)       flow_s_score_to[idx_n]       = solver.d_flow_score_to[idx];
            if (flow_s_score_to_valid) flow_s_score_to_valid[idx_n] = solver.d_flow_score_to_valid[idx];
        }
    }
}

__device__ void
flow_solver_load_res_sm9(flow_solver_t solver, flow_solver_opt_t opt, int i, int j,
        int i0_n, int j0_n, int s_sz_x, int s_sz_y, int jump_size,
        int32_t *flow_s_u, int32_t *flow_s_v, int32_t *flow_s_valid, float
        *flow_s_score, float *flow_s_score_to, int32_t *flow_s_score_to_valid) {

    for (int di=-1; di<=1; di++) {

        // Where are we loading this time for shared memory?
        int i_n = i + di*jump_size;

        // Make sure this is in range before we do anything
        if (i_n<0 || i_n>=solver.n_dim[0])
            continue;

        for (int dj=-1; dj<=1; dj++) {

            // Where are we loading this time for shared memory?
            int j_n = j + dj*jump_size;

            // Make sure this is in range before we do anything
            if (j_n<0 || j_n>=solver.n_dim[1])
                continue;

            // Get index in our shared memory
            int idx_n = (i_n - i0_n)*s_sz_y + (j_n - j0_n);

            // Get index in global memory
            int idx = i_n*solver.n_dim[1] + j_n;

            // Now load from global memory into shared memory
            if (flow_s_u)              flow_s_u[idx_n]              = solver.d_flow_u[idx];
            if (flow_s_v)              flow_s_v[idx_n]              = solver.d_flow_v[idx];
            if (flow_s_valid)          flow_s_valid[idx_n]          = solver.d_flow_valid[idx];
            if (flow_s_score)          flow_s_score[idx_n]          = solver.d_flow_score[idx];
            if (flow_s_score_to)       flow_s_score_to[idx_n]       = solver.d_flow_score_to[idx];
            if (flow_s_score_to_valid) flow_s_score_to_valid[idx_n] = solver.d_flow_score_to_valid[idx];
        }
    }
}

__global__ void
flow_solver_cookie1(flow_solver_t solver, flow_solver_opt_t opt, col_constancy_t cc) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;

    const int i0 = bidx * blockDim.x;
    const int j0 = bidy * blockDim.y;

    const int i = i0 + tidx;
    const int j = j0 + tidy;

    const int idx = i*solver.n_dim[1] + j;

    // Load everything into shared memory
    extern __shared__ int32_t shared_mem[];

    int s_sz_x = opt.uv_window*2 + blockDim.x;
    int s_sz_y = opt.uv_window*2 + blockDim.y;
    int s_sz = s_sz_x * s_sz_y;

    int s_sz_x_sm = opt.smoothing_window*2 + blockDim.x;
    int s_sz_y_sm = opt.smoothing_window*2 + blockDim.y;
    int s_sz_sm = s_sz_x_sm * s_sz_y_sm;

    int32_t *flow_s_u_sm =                           &shared_mem[0*s_sz_sm];
    int32_t *flow_s_v_sm =                           &shared_mem[1*s_sz_sm];
    int32_t *flow_s_valid_sm =                       &shared_mem[2*s_sz_sm];

    int start = 3*s_sz_sm;
    //float *flow_s_score =                (float*) &shared_mem[3*s_sz];
    float *flow_s_score_to =             (float*) &shared_mem[start + 0*s_sz];
    int32_t *flow_s_score_to_valid =              &shared_mem[start + 1*s_sz];

    // origin of block's shared memory
    int i0_n = i0 - opt.uv_window;
    int j0_n = j0 - opt.uv_window;

    flow_solver_load_res_sm9(solver, opt, i, j, i0_n, j0_n, s_sz_x, s_sz_y, opt.uv_window,
            NULL, NULL, NULL, NULL, flow_s_score_to, flow_s_score_to_valid);

    //  Loading smoothing with different dimensions for efficiency
    int i0_n_sm = i0 - opt.smoothing_window;
    int j0_n_sm = j0 - opt.smoothing_window;
    flow_solver_load_res_sm4(solver, opt, i, j, i0_n_sm, j0_n_sm, s_sz_x_sm, s_sz_y_sm, opt.smoothing_window,
            flow_s_u_sm, flow_s_v_sm, flow_s_valid_sm, NULL, NULL, NULL);

    __syncthreads();
    // Now we're doing loading into shared memory!

    // if we're out of range, stop now
    // (but we needed these threads to help us load into shared memory)
    if (i>=solver.n_dim[0] || j>=solver.n_dim[1])
        return;

    // Check to see if we really need to process this column
    // (ie, is there something in this column)
    // (but we needed these threads to help us load into shared memory)
    int col_empty = 1;
    for (int k=0; k<solver.n_dim[2] && col_empty; k++)
       if (get_occ_grid_prob(solver.d_occ_grid_1, i, j, k, solver.n_dim, USE_COLUMN_COALESCE) > 0.6)
           col_empty = 0;

    if (col_empty) {
        solver.d_flow_u[idx] = 0;
        solver.d_flow_v[idx] = 0;
        solver.d_flow_valid[idx] = 0;
        return;
    }

    // Find best place for this (i, j) to go according to cc
    float best_score = 0;
    int32_t best_u = 0;
    int32_t best_v = 0;

    int valid = 0;

    // Our index in shared memory for smoothing
    int idx_s_sm = (i - i0_n_sm) * s_sz_y_sm + (j - j0_n_sm);

    // Our current result
    int our_flow_valid = flow_s_valid_sm[idx_s_sm];
    int our_flow_u = flow_s_u_sm[idx_s_sm];
    int our_flow_v = flow_s_v_sm[idx_s_sm];

    // Indexing params
    const int s2 = cc.search_size/2;
    const int n2 = cc.n_dim[0] * cc.n_dim[1];

    const int ij = i*cc.n_dim[1] + j;

    // Smoothing terms
    int sum_u2 = 0;
    int sum_v2 = 0;

    int sum_u = 0;
    int sum_v = 0;

    int count = 0;

    if (opt.smoothing_valid) {
        score_smoothness_init_terms(opt, i, j, &sum_u2, &sum_v2, &sum_u, &sum_v,
                &count, i0_n_sm, j0_n_sm, flow_s_u_sm, flow_s_v_sm, flow_s_valid_sm,
                s_sz_y_sm);
    }

    const int uv_window = opt.uv_window;
    for (int du=-uv_window; du<=uv_window; du++) {

        const int d_i = du + s2;
        const int di_s = d_i * cc.search_size;

        for (int dv=-uv_window; dv<=uv_window; dv++) {

            //float score = lookup_score_windowed(cc, i, j, du, dv);

            /////////////////////

            const int d_j = dv + s2;
            const int d_ij = di_s + d_j;
            const int idx_d_ij = d_ij*n2 + ij;

            float score = cc.d_cc_scores_windowed[idx_d_ij];
            /////////////////////

            // Add small distance regularization
            score -= (du*du + dv*dv)*0.001;

            // Add smoothing
            if (opt.smoothing_valid) {
                float smoothness_score = 0.0;

                smoothness_score += sum_u2 + sum_v2;
                smoothness_score += count * (du*du + dv*dv);
                smoothness_score += -2 * (du*sum_u + dv*sum_v);

                //float smoothness_score_2 = score_smoothness_shared(opt, i, j, du, dv,
                //        i0_n_sm, j0_n_sm, flow_s_u_sm, flow_s_v_sm, flow_s_valid_sm,
                //        s_sz_y_sm);

                //if (i % 10 == 0 && j % 10 == 0 && du == 0 && dv == 0) {
                //    printf("%5.3f vs %5.3f\n", smoothness_score, smoothness_score_2);
                //}

                smoothness_score *= solver.smoothing_weight;
                score += -smoothness_score;
            }

            if (score > best_score || !valid) {

                // Is it better than the best score that's already there?
                int i_to = i + du;
                int j_to = j + dv;

                // Check bounds
                if (i_to >= 0 && i_to < solver.n_dim[0] &&
                        j_to >= 0 && j_to < solver.n_dim[1]) {

                    //int idx_to = i_to * solver.n_dim[1] + j_to;
                    int idx_to_s = (i_to - i0_n) * s_sz_y + (j_to - j0_n);

                    int currently_pointing_here = our_flow_valid && our_flow_u==du && our_flow_v==dv;

                    // Only assign if this is better than what's already pointing
                    // there (at least right now), if anything, or if we're updating
                    // our own score
                    if (!flow_s_score_to_valid[idx_to_s] ||
                            score > flow_s_score_to[idx_to_s] ||
                            currently_pointing_here) {

                        // Now we can assign flow
                        best_score = score;

                        best_u = du;
                        best_v = dv;

                        valid = 1;
                    }
                }
            }
        }
    }

    solver.d_flow_u[idx] = best_u;
    solver.d_flow_v[idx] = best_v;

    // We're valid if we were able to assign something
    solver.d_flow_valid[idx] = valid;

    solver.d_flow_score[idx] = best_score;
}

__global__ void
flow_solver_cookie2(flow_solver_t solver, flow_solver_opt_t opt, col_constancy_t cc) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;

    const int i0 = bidx * blockDim.x;
    const int j0 = bidy * blockDim.y;

    const int i = i0 + tidx;
    const int j = j0 + tidy;

    const int idx = i*solver.n_dim[1] + j;

    // Load everything into shared memory
    extern __shared__ int32_t shared_mem[];

    int s_sz_x = opt.uv_window*2 + blockDim.x;
    int s_sz_y = opt.uv_window*2 + blockDim.y;
    int s_sz = s_sz_x * s_sz_y;

    int32_t *flow_s_u =              &shared_mem[0*s_sz];
    int32_t *flow_s_v =              &shared_mem[1*s_sz];
    int32_t *flow_s_valid =          &shared_mem[2*s_sz];
    float *flow_s_score =   (float*) &shared_mem[3*s_sz];

    // origin of block's shared memory
    int i0_n = i0 - opt.uv_window;
    int j0_n = j0 - opt.uv_window;

    flow_solver_load_res_sm9(solver, opt, i, j, i0_n, j0_n, s_sz_x, s_sz_y, opt.uv_window,
            flow_s_u, flow_s_v, flow_s_valid, flow_s_score, NULL, NULL);

    __syncthreads();
    // Now we're doing loading into shared memory!

    // Find best place for this (i, j) to come from
    float best_score = 0;
    int32_t best_i_from = 0;
    int32_t best_j_from = 0;

    int valid = 0;

    const int uv_window = opt.uv_window;
    for (int du=-uv_window; du<=uv_window; du++) {

        int i_from = i + du;

        // Check bounds
        if (i_from < 0 || i_from >= solver.n_dim[0])
            continue;

        for (int dv=-uv_window; dv<=uv_window; dv++) {

            int j_from = j + dv;

            // Check bounds
            if (j_from < 0 || j_from >= solver.n_dim[1])
                continue;

            //int idx_from = i_from * solver.n_dim[1] + j_from;
            int idx_from_s = (i_from - i0_n)*s_sz_y + (j_from - j0_n);

            // Check if this potential from location has a valid score
            if (!flow_s_valid[idx_from_s])
                continue;

            // Does the flow this location point here?
            int i_to = i_from + flow_s_u[idx_from_s];
            int j_to = j_from + flow_s_v[idx_from_s];

            if (i_to != i || j_to != j)
                continue;

            float score = flow_s_score[idx_from_s];

            if (score > best_score || !valid) {
                best_score = score;

                best_i_from = i_from;
                best_j_from = j_from;

                valid = 1;
            }
        }
    }

    // Now clear out other answers that point to here (if we found something)
    if (valid) {
        for (int du=-uv_window; du<=uv_window; du++) {
            for (int dv=-uv_window; dv<=uv_window; dv++) {

                int i_from = i + du;
                int j_from = j + dv;

                if (i_from < 0 || i_from >= solver.n_dim[0])
                    continue;

                if (j_from < 0 || j_from >= solver.n_dim[1])
                    continue;

                if (i_from == best_i_from && j_from == best_j_from)
                    continue;

                // Does the flow this location point here?
                int idx_from_s = (i_from - i0_n)*s_sz_y + (j_from - j0_n);

                // Does the flow this location point here?
                int i_to = i_from + flow_s_u[idx_from_s];
                int j_to = j_from + flow_s_v[idx_from_s];

                if (i_to != i || j_to != j)
                    continue;

                int idx_from = i_from * solver.n_dim[1] + j_from;
                solver.d_flow_valid[idx_from] = 0;
            }
        }

        // Update score
        solver.d_flow_score_to[idx] = best_score;
    }

    solver.d_flow_score_to_valid[idx] = valid;
}

flow_image_t*
flow_solver_compute_cookie(flow_solver_t *solver, sparse_occ_grid_t *g1, sparse_occ_grid_t *g2) {

    flow_solver_opt_t opt;
    opt.smoothing_valid = 0;
    opt.smoothing_window = 2;

    // Get dense versions of the occ grids
    int64_t tic_dense = utime_now();
    sparse_occ_grid_get_dense_device(g1, solver->d_occ_grid_1, USE_COLUMN_COALESCE);
    sparse_occ_grid_get_dense_device(g2, solver->d_occ_grid_2, USE_COLUMN_COALESCE);
    int64_t toc_dense = utime_now();
    double t_dense = (toc_dense - tic_dense)/1e3;
    printf("\tTook %5.3f ms to create dense device occ grids\n", t_dense);

    solver->n_dim[0] = g1->n_dim[0];
    solver->n_dim[1] = g1->n_dim[1];
    solver->n_dim[2] = g1->n_dim[2];
    solver->res = g1->res;

    // Set up search area
    opt.uv_window = solver->uv_window;
    int search_size = 2*opt.uv_window + 1;

    // Compute col constancy using appearance matcher
    col_constancy_params_t p = {0};
    p.d_occ_grid_1 = solver->d_occ_grid_1;
    p.d_occ_grid_2 = solver->d_occ_grid_2;
    p.n_dim[0] = g1->n_dim[0];
    p.n_dim[1] = g1->n_dim[1];
    p.n_dim[2] = g1->n_dim[2];
    p.patch_window = 1;
    p.use_col_coalesce = 1;

    p.res = g1->res;

    int n_dim[3];
    n_dim[0] = g1->n_dim[0];
    n_dim[1] = g1->n_dim[1];
    n_dim[2] = g1->n_dim[2];
    col_constancy_t *cc = col_constancy_create(n_dim, search_size, solver->threads_per_block);

    // Set up weights
    const float wf[14] = {0.40, 0.64, 0.48, 0.23, 0.08, 0.01, 0.10, 0.06, 0.14, 0.06, 0.08, 0.10, 0.19, 0.34};
    const float wo[14] = {0.56, 0.69, 1.05, 1.48, 1.49, 1.49, 1.65, 1.37, 1.58, 1.49, 2.36, 3.18, 3.20, 0.00};
    const float wd[14] = {0.32, 0.39, 0.01, -0.44, -0.60, -0.52, -0.53, -0.45, -0.58, -0.69, -0.15, 0.37, 0.40, 0.00};

    // not being used
    cc->dist_loss = 6.97;

    cudaMemcpy(cc->d_wf, wf, sizeof(float)*14, cudaMemcpyHostToDevice);
    cudaMemcpy(cc->d_wo, wo, sizeof(float)*14, cudaMemcpyHostToDevice);
    cudaMemcpy(cc->d_wd, wd, sizeof(float)*14, cudaMemcpyHostToDevice);

    cc->b = -1.41;

    //cc->x_12[0] = x_12[0];
    //cc->x_12[1] = x_12[1];
    //cc->x_12[2] = x_12[2];
    //cc->x_12[3] = x_12[3];
    //cc->x_12[4] = x_12[4];
    //cc->x_12[5] = x_12[5];

    col_constancy_build(cc, p);

    // Setup thread and block dims
    dim3 threads_dim;
    threads_dim.x = THREADS_DIM;
    threads_dim.y = THREADS_DIM;
    threads_dim.z = 1;

    dim3 blocks_dim;
    blocks_dim.x = solver->n_dim[0] / threads_dim.x + 1;
    blocks_dim.y = solver->n_dim[1] / threads_dim.y + 1;
    blocks_dim.z = 1;

    int sm_dim_sm = THREADS_DIM + 2*opt.smoothing_window;
    int sm_dim_uv = THREADS_DIM + 2*opt.uv_window;

    int sm_dim_sm2 = sm_dim_sm * sm_dim_sm;
    int sm_dim_uv2 = sm_dim_uv * sm_dim_uv;

    size_t shared_memory_sz_1 = sizeof(float) * (sm_dim_sm2 * 3 + sm_dim_uv2 * 2);
    size_t shared_memory_sz_2 = sizeof(float) * (sm_dim_uv2 * 4);

    printf("\tRunning iterations with %dx%d threads and %dx%d blocks and %ld, %ld bytes of shared memory\n",
            threads_dim.x, threads_dim.y, blocks_dim.x, blocks_dim.y,
            shared_memory_sz_1, shared_memory_sz_2);

    // Set initial flow scores to invalid
    int n2 = solver->n_dim[0] * solver->n_dim[1];
    cudaMemset(solver->d_flow_valid, 0, sizeof(int32_t)*n2);
    cudaMemset(solver->d_flow_score_to_valid, 0, sizeof(int32_t)*n2);

    // Now we're ready to solve
    int64_t tic_iters = utime_now();
    for (int iter=0; iter<solver->em_iterations; iter++) {

        // Find where each col expects to go
        //int64_t tic_exp = utime_now();
        flow_solver_cookie1<<<blocks_dim, threads_dim, shared_memory_sz_1>>>(*solver, opt, *cc);
        cudaSafe(cudaDeviceSynchronize());
        //int64_t toc_exp = utime_now();
        //double t_exp = (toc_exp - tic_exp)/1e3;
        //printf("\tExp took %5.3f ms\n", t_exp);

        // Find best each for each col receiving flow
        //int64_t tic_max = utime_now();
        flow_solver_cookie2<<<blocks_dim, threads_dim, shared_memory_sz_2>>>(*solver, opt, *cc);
        cudaSafe(cudaDeviceSynchronize());
        //int64_t toc_max = utime_now();
        //double t_max = (toc_max - tic_max)/1e3;
        //printf("\tMax took %5.3f ms\n", t_max);

        // We can now smooth
        opt.smoothing_valid = 1;
    }
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_iters = utime_now();
    double t_iters = (toc_iters - tic_iters)/1e3;
    printf("\tTook %5.3f ms for %d iterations of EM\n", t_iters, solver->em_iterations);

    // Cleanup
    col_constancy_destroy(cc);

    // Copy flow back
    int n_dim_im[2] = {g1->n_dim[0], g1->n_dim[1]};
    flow_image_t *flow_image = flow_image_create(n_dim_im, g1->res);

    int64_t tic_copy = utime_now();
    flow_image_copy_from_gpu(flow_image, solver->d_flow_u, solver->d_flow_v, solver->d_flow_valid);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_copy = utime_now();
    double t_copy = (toc_copy - tic_copy)/1e3;
    printf("\tTook %5.3f ms to copy flow\n", t_copy);

    // Set flow time (seconds)
    flow_image->dt = (g1->utime - g2->utime)/1e6;

    return flow_image;
}
