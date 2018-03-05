#pragma once
#ifndef _FLOW_SOLVER_H_
#define _FLOW_SOLVER_H_

#include "library/ray_tracing/sparse_occ_grid.h"
#include "library/ray_tracing/sparse_occ_grid_multires.h"

#include "flow_image.h"

typedef struct {

    // Computing smoothing?
    int smoothing_valid;

    // Window parameters
    int patch_window;
    int smoothing_window;
    int uv_window;

} flow_solver_opt_t;

typedef struct {

    // Solver data
    int threads_per_block;

    int em_iterations;
    float smoothing_weight;

    int uv_window;

    // Size of grid being processed
    int n_dim[3];
    float res;

    // Dense copies of probabilities (in device memory)
    float *d_occ_grid_1;
    float *d_occ_grid_2;

    // Flow Results
    int32_t *d_flow_u;
    int32_t *d_flow_v;
    int32_t *d_flow_valid;

    // Best score coming from this position
    float   *d_flow_score;

    // Best score to this position and whether or not it's valid
    float   *d_flow_score_to;
    int32_t *d_flow_score_to_valid;

} flow_solver_t;

flow_solver_t*
flow_solver_create(int max_n_dim[3], int iterations, float smoothing_weight, int threads);

void
flow_solver_destroy(flow_solver_t *solver);

flow_image_t*
flow_solver_compute_cookie(flow_solver_t *solver, sparse_occ_grid_t *g1, sparse_occ_grid_t *g2);

#endif
