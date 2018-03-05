#include <stdio.h>

#include "flow_image.h"

flow_image_t*
flow_image_create(int n_dim[2], float res) {

    flow_image_t* f = (flow_image_t*) malloc(sizeof(flow_image_t));

    f->n_dim[0] = n_dim[0];
    f->n_dim[1] = n_dim[1];

    f->res = res;

    f->flow_u = NULL;
    f->flow_v = NULL;
    f->flow_valid = NULL;

    return f;
}

flow_image_t*
flow_image_copy(flow_image_t *f) {

    flow_image_t *res = flow_image_create(f->n_dim, f->res);

    res->dt = f->dt;

    // Allocate host memory
    size_t sz = sizeof(int32_t) * res->n_dim[0] * res->n_dim[1];
    res->flow_u = (int32_t*) malloc(sz);
    res->flow_v = (int32_t*) malloc(sz);
    res->flow_valid = (int32_t*) malloc(sz);

    memcpy(res->flow_u, f->flow_u, sz);
    memcpy(res->flow_v, f->flow_v, sz);
    memcpy(res->flow_valid, f->flow_valid, sz);

    return res;
}

void
flow_image_destroy(flow_image_t* f) {

    if (f->flow_u)
        free(f->flow_u);

    if (f->flow_v)
        free(f->flow_v);

    if (f->flow_valid)
        free(f->flow_valid);

    free(f);
}

flow_t
flow_image_get_flow(flow_image_t *f, int i, int j) {

    int idx = i*f->n_dim[1] + j;

    flow_t flow;

    flow.u = f->flow_u[idx];
    flow.v = f->flow_v[idx];
    flow.valid = f->flow_valid[idx];

    return flow;
}

void
flow_image_copy_from_gpu(flow_image_t* f, int32_t *d_flow_u, int32_t *d_flow_v, int32_t *d_flow_valid) {

    // Allocate host memory
    f->flow_u = (int32_t*) malloc(sizeof(int32_t)*f->n_dim[0]*f->n_dim[1]);
    f->flow_v = (int32_t*) malloc(sizeof(int32_t)*f->n_dim[0]*f->n_dim[1]);
    f->flow_valid = (int32_t*) malloc(sizeof(int32_t)*f->n_dim[0]*f->n_dim[1]);

    // Copy from device
    cudaMemcpy(f->flow_u, d_flow_u, sizeof(int32_t)*f->n_dim[0]*f->n_dim[1], cudaMemcpyDeviceToHost);
    cudaMemcpy(f->flow_v, d_flow_v, sizeof(int32_t)*f->n_dim[0]*f->n_dim[1], cudaMemcpyDeviceToHost);
    cudaMemcpy(f->flow_valid, d_flow_valid, sizeof(int32_t)*f->n_dim[0]*f->n_dim[1], cudaMemcpyDeviceToHost);
}

void
flow_image_save_bin(flow_image_t *f, const char* filename) {

    FILE *fp = fopen(filename, "w");

    fwrite(f->n_dim, sizeof(int), 2, fp);
    fwrite(&f->res, sizeof(float), 1, fp);
    fwrite(&f->dt, sizeof(float), 1, fp);
    fwrite(f->flow_u, sizeof(int32_t), f->n_dim[0]*f->n_dim[1], fp);
    fwrite(f->flow_v, sizeof(int32_t), f->n_dim[0]*f->n_dim[1], fp);
    fwrite(f->flow_valid, sizeof(int32_t), f->n_dim[0]*f->n_dim[1], fp);

    fclose(fp);
}

flow_image_t*
flow_load_bin(const char* filename) {

    FILE *f_in = fopen(filename, "r");

    fseek(f_in, 0, SEEK_END); // seek to end of file
    int size = ftell(f_in); // get current file pointer
    fseek(f_in, 0, SEEK_SET); // seek back to beginning of file

    char *in = (char*) malloc(size);
    fread(in, sizeof(char), size, f_in);

    fclose(f_in);

    char *at = in;

    // Find n_dim and resolution
    int n_dim[2];
    memcpy(&n_dim[0], at, sizeof(int));
    at += sizeof(int);
    memcpy(&n_dim[1], at, sizeof(int));
    at += sizeof(int);

    float res = 0.0;
    memcpy(&res, at, sizeof(float));
    at += sizeof(float);

    float dt = 0.0;
    memcpy(&dt, at, sizeof(float));
    at += sizeof(float);

    flow_image_t *flow = flow_image_create(n_dim, res);
    flow->dt = dt;

    flow->flow_u = (int32_t*) malloc(sizeof(int32_t)*n_dim[0]*n_dim[1]);
    flow->flow_v = (int32_t*) malloc(sizeof(int32_t)*n_dim[0]*n_dim[1]);
    flow->flow_valid = (int32_t*) malloc(sizeof(int32_t)*n_dim[0]*n_dim[1]);

    memcpy(flow->flow_u, at, sizeof(int32_t)*n_dim[0]*n_dim[1]);
    at += sizeof(int32_t)*n_dim[0]*n_dim[1];

    memcpy(flow->flow_v, at, sizeof(int32_t)*n_dim[0]*n_dim[1]);
    at += sizeof(int32_t)*n_dim[0]*n_dim[1];

    memcpy(flow->flow_valid, at, sizeof(int32_t)*n_dim[0]*n_dim[1]);

    free(in);

    return flow;
}

void
flow_image_save_csv(flow_image_t *f, const char* filename, int velocity) {

    FILE *f_out = fopen(filename, "w");

    // Do we convert to speed?
    float k = velocity ? (1/f->dt):1.0;

    // Save u first, and then v, and then idx (for validity check)
    for (int pass=0; pass<3; pass++) {
        for (int i=0; i<f->n_dim[0]; i++) {
            for (int j=0; j<f->n_dim[1]; j++) {

                flow_t f_ij = flow_image_get_flow(f, i, j);

                const char* append = (j==f->n_dim[1]-1) ? "\n":",";

                switch(pass) {
                    case 0:
                        fprintf(f_out, "%5.3f%s", f_ij.u*f->res*k, append);
                        break;

                    case 1:
                        fprintf(f_out, "%5.3f%s", f_ij.v*f->res*k, append);
                        break;

                    case 2:
                        fprintf(f_out, "%d%s", f_ij.valid, append);
                        break;
                }
            }
        }
    }

    fclose(f_out);
}
