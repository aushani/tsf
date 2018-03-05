#include "image_roi.h"

#include "library/ray_tracing/util.h"

image_roi_t*
image_roi_create(int w, int h, int d) {

    image_roi_t *roi = (image_roi_t*) malloc(sizeof(image_roi_t));

    roi->width = w;
    roi->height = h;
    roi->depth = d;

    roi->data = (uint8_t*) malloc(sizeof(uint8_t)*h*w*(d+1));

    return roi;
}

void
image_roi_destroy(image_roi_t *roi) {

    free(roi->data);

    free(roi);
}

float
image_roi_get(image_roi_t *roi, int i, int j, int d) {

    if (i<0 || i>=roi->width || j<0 || j>=roi->height)
        return -1;

    if (d<0 || d>=roi->depth)
        return -1;

    int idx = ((i*roi->height) + j) * (roi->depth+1) + d;
    return roi->data[idx]*255.0;
}

int
image_roi_get_depth_err_byte(image_roi_t *roi, int i, int j) {

    if (i<0 || i>=roi->width || j<0 || j>=roi->height)
        return -1;

    int idx = (i*roi->height) + j;
    idx = idx*(roi->depth+1) + (roi->depth);

    int val = roi->data[idx];
    return val;
}

void
image_roi_set_data(image_roi_t *roi, const uint8_t *data) {

    size_t sz = sizeof(uint8_t)*roi->width*roi->height*(roi->depth+1);
    memcpy(roi->data, data, sz);
}
