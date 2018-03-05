#pragma once
#ifndef _DEPTH_BUFFER_BUILDER_H_
#define _DEPTH_BUFFER_BUILDER_H_

#include "depth_buffer.h"

typedef struct {

    int height;
    int width;

    float *sparse_depth;

    float *tmp_1;
    float *tmp_2;

} depth_buffer_builder_t;

depth_buffer_builder_t*
depth_buffer_builder_create(int height, int width);

void
depth_buffer_builder_destroy(depth_buffer_builder_t *dbb);

depth_buffer_t*
depth_buffer_builder_build(depth_buffer_builder_t *dbb, float *sparse_depth, depth_buffer_t *db);

#endif

