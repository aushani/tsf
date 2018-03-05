#pragma once
#ifndef _SPARSE_OCC_GRID_H_
#define _SPARSE_OCC_GRID_H_

#include <stdint.h>

const float l_occ = 1.0;
const float l_free = -0.1;

const float l_thresh = 1.0;

const float l_max = 3.0;
const float l_min = -1.0;

const int S_UNKN = 0;
const int S_FREE = 1;
const int S_OCCU = 2;

typedef struct {

    int64_t n_dim[3];
    float res;

    double pose[6]; // Origin pose of the occupancy grid

    int64_t *keys;
    float *vals;

    int64_t *d_keys;
    float  *d_vals;

    int n_kv;

    // Does the i,j column look like it's background?
    int *filter;
    int *d_filter;

    int64_t utime;

} sparse_occ_grid_t;

typedef struct {

    sparse_occ_grid_t *grid;

    int want_dense;     // Whether or not we should only look at nonzero elements
    int use_filter;     // Whether or not we should use the filter

    int64_t idx_at;     // The next index we want to look at
    int64_t max_idx;    // The maximum index in the occ grid

    int64_t map_index_at;   // The index in our list of key/values that we're at
    int64_t key_at;         // The key that we're currently waiting on

    uint8_t done; // Are we done?

} sparse_occ_grid_it_t;

sparse_occ_grid_t*
sparse_occ_grid_create(float resolution, int64_t n_dim[3]);

sparse_occ_grid_t*
sparse_occ_grid_copy(sparse_occ_grid_t *grid, int want_host, int want_device);

void
sparse_occ_grid_destroy(sparse_occ_grid_t *grid);

float
sparse_occ_grid_lookup(sparse_occ_grid_t *grid, int i, int j, int k);

int
sparse_occ_grid_count_nonzero(sparse_occ_grid_t *grid);

void
sparse_occ_grid_save(sparse_occ_grid_t *grid, char* filename);

void
sparse_occ_grid_get_dense_device(sparse_occ_grid_t *grid, float *d_grid, int col_coalesce);

void
sparse_occ_grid_get_dense_device_mle(sparse_occ_grid_t *grid, int *d_grid, int col_coalesce);

int32_t*
sparse_occ_grid_get_dense_slice(sparse_occ_grid_t *grid, int row);

void
sparse_occ_grid_get_xyz(sparse_occ_grid_t *grid, int64_t idx, double *x, double *y, double *z);

void
sparse_occ_grid_get_xyz(sparse_occ_grid_t *grid, int i, int j, int k, double *x, double *y, double *z);

void
sparse_occ_grid_center_xyz(sparse_occ_grid_t *grid, double x, double y, double z,
        double *x_c, double *y_c, double *z_c);

void
sparse_occ_grid_get_ijk(sparse_occ_grid_t *grid, double x, double y, double z, int *i, int *j, int *k);

void
sparse_occ_grid_it_init(sparse_occ_grid_t *grid, sparse_occ_grid_it_t *git, int want_dense);

void
sparse_occ_grid_it_use_filter(sparse_occ_grid_it_t *git, int use_filter);

uint8_t
sparse_occ_grid_it_next(sparse_occ_grid_it_t *git, int64_t *key, float *val);

void
sparse_occ_grid_idx_to_ijk(sparse_occ_grid_t *grid, int64_t idx, int *i, int *j, int *k);

void
sparse_occ_grid_ijk_to_idx(sparse_occ_grid_t *grid, int i, int j, int k, int64_t *idx);

int
sparse_occ_grid_count_occ_in_col(sparse_occ_grid_t *grid, int i, int j);

void
sparse_occ_grid_build_filter(sparse_occ_grid_t *grid, const float *wf, const float *wo, float b, int window_size);

int
sparse_occ_grid_check_filter(sparse_occ_grid_t *grid, int i, int j);

int
sparse_occ_grid_check_filter(sparse_occ_grid_t *grid, int64_t idx);

int
sparse_occ_grid_is_occuluded(sparse_occ_grid_t *grid, double x1, double y1, double z1, double x2, double y2, double z2);

sparse_occ_grid_t*
sparse_occ_grid_merge(sparse_occ_grid_t *g1, sparse_occ_grid_t *g2);

void
sparse_occ_grid_clean(sparse_occ_grid_t *grid);

void
sparse_occ_grid_save(sparse_occ_grid_t *grid, const char* fn);

sparse_occ_grid_t*
sparse_occ_grid_load(const char* fn);

sparse_occ_grid_t*
sparse_occ_grid_get_subset(sparse_occ_grid_t *grid, double x[6], int64_t n_dim[3]);

#endif
