#include "depth_buffer.h"

depth_buffer_t*
depth_buffer_create(int height, int width, const float *d_depth) {

    depth_buffer_t* db = (depth_buffer_t*) malloc(sizeof(depth_buffer_t));

    db->height = height;
    db->width = width;

    db->depth = NULL;
    db->d_depth = NULL;

    cudaMalloc(&db->d_depth, sizeof(float)*height*width);
    cudaMemcpy(db->d_depth, d_depth, sizeof(float)*height*width, cudaMemcpyDeviceToDevice);

    db->depth = (float*) malloc(sizeof(float)*height*width);
    cudaMemcpy(db->depth, d_depth, sizeof(float)*height*width, cudaMemcpyDeviceToHost);

    return db;
}

depth_buffer_t*
depth_buffer_copy(const depth_buffer_t *db) {

    depth_buffer_t *db2 = depth_buffer_create(db->height, db->width, db->d_depth);

    return db2;
}

void
depth_buffer_destroy(depth_buffer_t *db) {

    if (db->depth)
        free(db->depth);

    if (db->d_depth)
        cudaFree(db->d_depth);

    free(db);
}

float
depth_buffer_get(depth_buffer_t *db, int h, int w) {

    if (h<0 || h >= db->height)
        return -1;

    if (w<0 || w >= db->width)
        return -1;

    if (db->depth == NULL)
        return -1;

    return db->depth[h*db->width + w];
}
