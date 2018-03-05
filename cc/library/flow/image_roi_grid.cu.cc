#include "image_roi_grid.h"

#include "library/ray_tracing/util.h"
//#include "library/ray_tracing/gpu_util.h"

image_roi_grid_t*
image_roi_grid_create(const int n_dim[2], int width, int height, int depth) {

    int64_t tic = utime_now();

    image_roi_grid_t *roi_grid = (image_roi_grid_t*) malloc(sizeof(image_roi_grid_t));

    roi_grid->utime = -1;

    roi_grid->n_dim[0] = n_dim[0];
    roi_grid->n_dim[1] = n_dim[1];

    roi_grid->width = width;
    roi_grid->height = height;

    roi_grid->depth = depth;

    size_t sz_data = sizeof(uint8_t)*n_dim[0]*n_dim[1]*width*height*(depth+1); // remember depth err (meters)
    cudaMalloc(&roi_grid->d_data, sz_data);

    roi_grid->data = NULL;

    size_t mem = sz_data;
    int64_t toc = utime_now();
    double t = (toc-tic)/1e3;
    printf("\tTook %5.3f ms to create image roi grid with %ld MBytes\n", t, mem/(1024*1024));

    return roi_grid;
}

void
image_roi_grid_destroy(image_roi_grid_t *roi_grid) {

    int64_t tic = utime_now();

    if (roi_grid->data) free(roi_grid->data);
    if (roi_grid->d_data) cudaFree(roi_grid->d_data);

    free(roi_grid);

    int64_t toc = utime_now();
    double t = (toc-tic)/1e3;
    printf("\tTook %5.3f ms to destroy image roi grid\n", t);
}

image_roi_grid_t*
image_roi_grid_make_host_copy(const image_roi_grid_t* roi_grid) {

    image_roi_grid_t *res = (image_roi_grid_t*) malloc(sizeof(image_roi_grid_t));

    res->utime = roi_grid->utime;

    res->n_dim[0] = roi_grid->n_dim[0];
    res->n_dim[1] = roi_grid->n_dim[1];

    res->width = roi_grid->width;
    res->height = roi_grid->height;

    res->depth = roi_grid->depth;

    size_t sz_data = sizeof(uint8_t)*res->n_dim[0]*res->n_dim[1]*res->width*res->height*(res->depth+1); // remember depth err (meters)

    res->data = (uint8_t*) malloc(sz_data);
    res->d_data = NULL;

    cudaMemcpy(res->data, roi_grid->d_data, sz_data, cudaMemcpyDeviceToHost);

    return res;
}

image_roi_t*
image_roi_grid_get_at(image_roi_grid_t *roi_grid, int grid_i, int grid_j) {

    // Check in range
    if (grid_i < 0 || grid_i >= roi_grid->n_dim[0]) return NULL;
    if (grid_j < 0 || grid_j >= roi_grid->n_dim[1]) return NULL;

    // Make sure that we have the data on the host
    if (roi_grid->data == NULL) {
        size_t sz_data = sizeof(uint8_t)*roi_grid->n_dim[0]*roi_grid->n_dim[1]
                        *roi_grid->width*roi_grid->height*(roi_grid->depth+1); // remember depth err (meters)

        roi_grid->data = (uint8_t*) malloc(sz_data);

        cudaMemcpy(roi_grid->data, roi_grid->d_data, sz_data, cudaMemcpyDeviceToHost);
    }

    int idx_roi = 0;
    int idx_grid = grid_i*roi_grid->n_dim[1] + grid_j;

    int idx = idx_grid*roi_grid->width*roi_grid->height + idx_roi;

    uint8_t *data = &roi_grid->data[idx*(roi_grid->depth+1)];

    image_roi_t* roi = image_roi_create(roi_grid->width, roi_grid->height, roi_grid->depth);

    image_roi_set_data(roi, data);

    return roi;

}
