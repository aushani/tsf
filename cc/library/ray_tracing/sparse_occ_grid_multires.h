#pragma once
#ifndef _SPARSE_OCC_GRID_MULTIRES_H_
#define _SPARSE_OCC_GRID_MULTIRES_H_

#include "sparse_occ_grid.h"

typedef struct {

    int n_levels;
    sparse_occ_grid_t **grids;

} sparse_occ_grid_multires_t;

sparse_occ_grid_multires_t*
sparse_occ_grid_multires_create(int n_levels);

void
sparse_occ_grid_multires_destroy(sparse_occ_grid_multires_t *g_mr);

void
sparse_occ_grid_multires_destroy_shallow(sparse_occ_grid_multires_t *g_mr);

sparse_occ_grid_t*
sparse_occ_grid_multires_get_level(sparse_occ_grid_multires_t* g_mr, int level);

#endif
