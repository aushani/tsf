#include "sparse_occ_grid_multires.h"

sparse_occ_grid_multires_t*
sparse_occ_grid_multires_create(int n_levels) {

    sparse_occ_grid_multires_t *g_mr = (sparse_occ_grid_multires_t*) malloc(sizeof(sparse_occ_grid_multires_t));

    g_mr->n_levels = n_levels;
    g_mr->grids = (sparse_occ_grid_t**) calloc(sizeof(sparse_occ_grid_t*), n_levels);

    return g_mr;

}

void
sparse_occ_grid_multires_destroy_helper(sparse_occ_grid_multires_t *g_mr, int deep) {

    if (deep) {
        for (int i=0; i<g_mr->n_levels; i++) {
            if (g_mr->grids[i])
                sparse_occ_grid_destroy(g_mr->grids[i]);
        }
    }

    free(g_mr->grids);
    free(g_mr);
}

void
sparse_occ_grid_multires_destroy(sparse_occ_grid_multires_t *g_mr) {
    sparse_occ_grid_multires_destroy_helper(g_mr, 1);
}

void
sparse_occ_grid_multires_destroy_shallow(sparse_occ_grid_multires_t *g_mr) {
    sparse_occ_grid_multires_destroy_helper(g_mr, 0);
}

sparse_occ_grid_t*
sparse_occ_grid_multires_get_level(sparse_occ_grid_multires_t* g_mr, int level) {

    return g_mr->grids[level];

}
