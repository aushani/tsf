#pragma once
#ifndef _FLOW_IMAGE_H
#define _FLOW_IMAGE_H_

#include "flow.h"

typedef struct {

    int n_dim[2];
    float res;

    float dt;

    int32_t *flow_u;
    int32_t *flow_v;
    int32_t *flow_valid;

} flow_image_t;

flow_image_t*
flow_image_create(int n_dim[2], float res);

flow_image_t*
flow_image_copy(flow_image_t *f);

void
flow_image_destroy(flow_image_t* f);

flow_t
flow_image_get_flow(flow_image_t *f, int i, int j);

void
flow_image_copy_from_gpu(flow_image_t* f, int32_t *d_flow_u, int32_t *d_flow_v, int32_t *d_flow_valid);

void
flow_image_save_bin(flow_image_t *f, const char* filename);

flow_image_t*
flow_load_bin(const char* filename);

void
flow_image_save_csv(flow_image_t *f, const char* filename, int velocity);

#endif
