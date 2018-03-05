#pragma once
#ifndef _GPU_OCC_GRID_BUILDER_H_
#define _GPU_OCC_GRID_BUILDER_H_

#include "sparse_occ_grid.h"
#include "sparse_occ_grid_multires.h"

#include "library/sensors/velodyne.h"

typedef struct {

    int num_returns;

    float pose_xyzrph_scan[6];

    float *x;
    float *y;
    float *z;
    int32_t *sensor_num;

    float* origin_x;
    float* origin_y;
    float* origin_z;

    int32_t *skip_hits;
    uint16_t use_skip_hits;

} gpu_velodyne_returns_t;

typedef struct {

    float resolution;
    int64_t n_dim[3];
    int max_hits_per_ray;

} gpu_occ_grid_builder_opt_t;

typedef struct {

    int threads_per_block;

    float resolution_base;
    int64_t n_dim_base[3];

    int max_hits;
    int max_hits_per_ray_base;

    int64_t *d_keys;
    float *d_vals;

    int verbose;

    float *d_x_vs;

    gpu_velodyne_returns_t vr;

} gpu_occ_grid_builder_t;

gpu_occ_grid_builder_t*
gpu_occ_grid_builder_create(int max_hits, int threads_per_block, int max_hits_per_ray,
                            float resolution, int n_dim[3]);

void
gpu_occ_grid_builder_destroy(gpu_occ_grid_builder_t *builder);

void
gpu_occ_grid_builder_set_x_vs(gpu_occ_grid_builder_t *builder, float *x_vs, int num_el);

sparse_occ_grid_t*
gpu_occ_grid_builder_build(gpu_occ_grid_builder_t *builder, const velodyne_returns_t *vr, double pose[6], int32_t *skip_hits, int64_t utime);

sparse_occ_grid_multires_t*
gpu_occ_grid_builder_build_multires(gpu_occ_grid_builder_t *builder, const velodyne_returns_t *vr, double pose[6], int32_t *skip_hits, int n_levels, int64_t utime);

sparse_occ_grid_t*
gpu_occ_grid_builder_add(gpu_occ_grid_builder_t *builder,
        const float *hits_x, const float *hits_y, const float *hits_z,
        const float *origin_x, const float *origin_y, const float *origin_z,
        int n_hits, sparse_occ_grid_t *sog);

#endif
