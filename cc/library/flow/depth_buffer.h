#pragma once
#ifndef _DEPTH_BUFFER_H_
#define _DEPTH_BUFFER_H_

typedef struct {

    int64_t utime;

    int height;
    int width;

    float *depth;
    float *d_depth;

} depth_buffer_t;

depth_buffer_t*
depth_buffer_create(int height, int width, const float *d_depth);

depth_buffer_t*
depth_buffer_copy(const depth_buffer_t *db);

void
depth_buffer_destroy(depth_buffer_t *db);

float
depth_buffer_get(depth_buffer_t *db, int h, int w);

#endif
