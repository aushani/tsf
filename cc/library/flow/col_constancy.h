#pragma once
#ifndef _COL_CONSTANCY_H_
#define _COL_CONSTANCY_H_

#include "flow.h"

typedef struct {

    int k;
    int input_channels;
    int output_channels;

    int strides;

    float *W;
    float *b;

} conv_t;

typedef struct {

    int input_channels;
    int output_channels;

    float *W;
    float *b;

} fully_connected_t;

typedef struct {

    int n_dim[2];

    int search_size;
    int threads_per_block;

    float def_cc_score;
    float min_cc_score;

    float *d_cc_scores;
    float *d_cc_scores_windowed;

    float *d_wf;
    float *d_wo;
    float *d_wd;
    float dist_loss;
    float b;

    float x_12[6];

    // the tensorflow way
    conv_t              conv_layer;
    fully_connected_t   fc_layer;


} col_constancy_t;

typedef struct {

    //int *d_p_g1;
    //int *d_p_g2;

    float *d_occ_grid_1;
    float *d_occ_grid_2;

    int n_dim[3];

    int patch_window;
    int use_col_coalesce;

    int32_t *d_flow0_u;
    int32_t *d_flow0_v;
    int32_t *d_flow0_valid;

    int initial_guess_scale;
    int initial_guess_valid;

    float res;

} col_constancy_params_t;

col_constancy_t*
col_constancy_create(int n_dim[3], int search_size, int threads_per_block);

void
col_constancy_destroy(col_constancy_t *cc);

void
col_constancy_build(col_constancy_t *cc, col_constancy_params_t p);

#endif
