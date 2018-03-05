#include "image_roi_grid_builder.h"

#include "library/util/util.h"

#include "library/gpu_util/gpu_util.cu.h"

image_roi_grid_builder_t*
image_roi_grid_builder_create(int n_dim[3], float res, int im_w, int im_h, int roi_w, int roi_h, int depth) {

    int64_t tic = utime_now();

    image_roi_grid_builder_t *rg_builder = (image_roi_grid_builder_t*) malloc(sizeof(image_roi_grid_builder_t));

    size_t sz_image = sizeof(uint8_t)*im_w*im_h*depth;

    rg_builder->n_dim[0] = n_dim[0];
    rg_builder->n_dim[1] = n_dim[1];
    rg_builder->n_dim[2] = n_dim[2];
    rg_builder->res = res;

    rg_builder->im_w = im_w;
    rg_builder->im_h = im_h;

    rg_builder->roi_w = roi_w;
    rg_builder->roi_h = roi_h;
    rg_builder->depth = depth;

    cudaSafe(cudaMalloc(&rg_builder->d_M, sizeof(float)*12));
    cudaSafe(cudaMalloc(&rg_builder->d_image, sz_image));

    size_t sz_coords = sizeof(int)*n_dim[0]*n_dim[1]*roi_w*roi_h;
    size_t sz_coords_z = sizeof(float)*n_dim[0]*n_dim[1]*roi_w*roi_h;
    cudaSafe(cudaMalloc(&rg_builder->coords_x, sz_coords));
    cudaSafe(cudaMalloc(&rg_builder->coords_y, sz_coords));
    cudaSafe(cudaMalloc(&rg_builder->coords_z, sz_coords_z));
    cudaSafe(cudaDeviceSynchronize());

    int64_t toc = utime_now();
    double t_ms = (toc - tic)/1e3;

    size_t mem = sz_image + sz_coords*2 + sz_coords_z;
    printf("\tTook %5.3f ms to allocate %ld Mbytes for GPU image roi grid builder\n",
            t_ms, mem/(1024*1024));

    return rg_builder;
}

void
image_roi_grid_builder_destroy(image_roi_grid_builder_t* rg_builder) {

    cudaFree(rg_builder->d_M);
    cudaFree(rg_builder->d_image);

    cudaFree(rg_builder->coords_x);
    cudaFree(rg_builder->coords_y);
    cudaFree(rg_builder->coords_z);

    free(rg_builder);
}

__device__ void
set_coords(image_roi_grid_builder_t rg_builder, float x, float y, float z,
        int grid_i, int grid_j, int roi_x, int roi_y) {

    int idx_roi = roi_x*rg_builder.roi_h + roi_y;
    int idx_grid = grid_i*rg_builder.n_dim[1] + grid_j;

    int idx = idx_roi*rg_builder.n_dim[0]*rg_builder.n_dim[1] + idx_grid;

    rg_builder.coords_x[idx] = round(x);
    rg_builder.coords_y[idx] = round(y);
    rg_builder.coords_z[idx] = z;
}

__device__ void
get_coords(image_roi_grid_builder_t rg_builder, int *x, int *y, float *z,
        int grid_i, int grid_j, int roi_x, int roi_y) {

    int idx_roi = roi_x*rg_builder.roi_h + roi_y;
    int idx_grid = grid_i*rg_builder.n_dim[1] + grid_j;

    int idx = idx_roi*rg_builder.n_dim[0]*rg_builder.n_dim[1] + idx_grid;

    *x = rg_builder.coords_x[idx];
    *y = rg_builder.coords_y[idx];
    *z = rg_builder.coords_z[idx];
}

__global__ void
precompute_coords(image_roi_grid_builder_t rg_builder) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;

    const int grid_i0 = bidx * blockDim.x;
    const int grid_j0 = bidy * blockDim.y;

    const int grid_i = grid_i0 + tidx;
    const int grid_j = grid_j0 + tidy;

    if (grid_i >= rg_builder.n_dim[0] || grid_j >= rg_builder.n_dim[1])
        return;

    float x_center = (grid_i-rg_builder.n_dim[0]/2) * rg_builder.res;
    float y_center = (grid_j-rg_builder.n_dim[1]/2) * rg_builder.res;

    float y0 = y_center + rg_builder.res/2;
    float y1 = y_center - rg_builder.res/2;

    float z0 = -rg_builder.n_dim[2]/2 * rg_builder.res;
    float z1 = -z0;

    float p_c0[3];
    float p_c[3];

    float x_im, y_im, z_im;

    for (int i=0; i<rg_builder.roi_w; i++) {
        float y_ij = y0 + ((y1-y0)*i)/rg_builder.roi_w;

        p_c0[0] = rg_builder.d_M[0] * x_center + rg_builder.d_M[1] * y_ij + rg_builder.d_M[3];
        p_c0[1] = rg_builder.d_M[4] * x_center + rg_builder.d_M[5] * y_ij + rg_builder.d_M[7];
        p_c0[2] = rg_builder.d_M[8] * x_center + rg_builder.d_M[9] * y_ij + rg_builder.d_M[11];

        for (int j=0; j<rg_builder.roi_h; j++) {

            float z_ij = z0 + ((z1-z0)*j)/rg_builder.roi_h;

            p_c[0] = p_c0[0] + rg_builder.d_M[2] * z_ij;
            p_c[1] = p_c0[1] + rg_builder.d_M[6] * z_ij;
            p_c[2] = p_c0[2] + rg_builder.d_M[10] * z_ij;

            x_im = p_c[0]/p_c[2];
            y_im = p_c[1]/p_c[2];
            z_im = p_c[2];

            set_coords(rg_builder, x_im, y_im, z_im, grid_i, grid_j, i, j);
        }
    }

}

void
image_roi_grid_builder_set_M(image_roi_grid_builder_t* rg_builder, Eigen::MatrixXd M) {

    int64_t tic = utime_now();

    float M_array[12];
    for (int i=0; i<M.rows(); i++) {
        for (int j=0; j<M.cols(); j++) {
            M_array[i*M.cols() + j] = M(i, j);
        }
    }

    cudaMemcpy(rg_builder->d_M, M_array, sizeof(float)*12, cudaMemcpyHostToDevice);

    // Precompute all the coordinate transformations
    dim3 threads_dim;
    threads_dim.x = 32;
    threads_dim.y = 32;
    threads_dim.z = 1;

    dim3 blocks_dim;
    blocks_dim.x = rg_builder->n_dim[0] / threads_dim.x + 1;
    blocks_dim.y = rg_builder->n_dim[1] / threads_dim.y + 1;
    blocks_dim.z = 1;

    printf("\tMake image roi grid coordinates with %dx%d threads and %dx%d blocks\n",
            threads_dim.x, threads_dim.y, blocks_dim.x, blocks_dim.y);

    precompute_coords<<<blocks_dim, threads_dim>>>(*rg_builder);
    cudaSafe(cudaDeviceSynchronize());

    int64_t toc = utime_now();
    double t_ms = (toc - tic)/1e3;
    printf("\tTook %5.3f ms to precompute projections\n", t_ms);
}

__global__ void
build(image_roi_grid_builder_t rg_builder, image_roi_grid_t grid, depth_buffer_t db) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;

    const int grid_i0 = bidx * blockDim.x;
    const int grid_j0 = bidy * blockDim.y;

    const int grid_i = grid_i0 + tidx;
    const int grid_j = grid_j0 + tidy;

    if (grid_i >= rg_builder.n_dim[0] || grid_j >= rg_builder.n_dim[1])
        return;

    int x_im, y_im;
    float z_im;

    for (int i=0; i<rg_builder.roi_w; i++) {
        for (int j=0; j<rg_builder.roi_h; j++) {

            get_coords(rg_builder, &x_im, &y_im, &z_im, grid_i, grid_j, i, j);

            uint8_t r = 0;
            uint8_t g = 0;
            uint8_t b = 0;
            uint8_t depth_err_byte = 255;

            if (x_im >= 0 && x_im < rg_builder.im_w &&
                y_im >= 0 && y_im < rg_builder.im_h &&
                z_im > 0) {

                // get image pixel value
                int idx0 = ( (rg_builder.im_h - y_im)*rg_builder.im_w + x_im)*rg_builder.depth;
                r = rg_builder.d_image[idx0 + 0];
                g = rg_builder.d_image[idx0 + 1];
                b = rg_builder.d_image[idx0 + 2];

                // Check depth map
                int idx_db = x_im*db.width + y_im;
                float depth = db.d_depth[idx_db];
                float depth_err = depth - z_im;
                if (depth<1e-3)
                    depth_err = 999.9;

                // Map to depth err byte
                int val = round(depth_err*100) + 127;
                if (val<0) val = 0;
                if (val>255) val = 255;
                depth_err_byte = val;
            }

            int idx_roi = i*rg_builder.roi_h + j;
            int idx_grid = grid_i*rg_builder.n_dim[1] + grid_j;
            //int idx = idx_roi*rg_builder.n_dim[0]*rg_builder.n_dim[1] + idx_grid;
            int idx = idx_grid*rg_builder.roi_w*rg_builder.roi_h + idx_roi;

            grid.d_data[idx*(rg_builder.depth+1) + 0] = r;
            grid.d_data[idx*(rg_builder.depth+1) + 1] = g;
            grid.d_data[idx*(rg_builder.depth+1) + 2] = b;
            grid.d_data[idx*(rg_builder.depth+1) + 3] = depth_err_byte;
        }
    }
}

image_roi_grid_t*
image_roi_grid_builder_build(image_roi_grid_builder_t *rg_builder, osg::Image *image,
        depth_buffer_t *db, image_roi_grid_t *grid) {

    if (grid==NULL) {
        grid = image_roi_grid_create(rg_builder->n_dim, rg_builder->roi_w,
            rg_builder->roi_h, rg_builder->depth);
    }

    // Send image to device memory
    const unsigned char* image_data = image->data();
    cudaMemcpy(rg_builder->d_image, image_data,
            sizeof(uint8_t) * rg_builder->im_w * rg_builder->im_h * rg_builder->depth,
            cudaMemcpyHostToDevice);

    dim3 threads_dim;
    threads_dim.x = 1;
    threads_dim.y = 8;
    threads_dim.z = 1;

    dim3 blocks_dim;
    blocks_dim.x = (rg_builder->n_dim[0]-1) / threads_dim.x + 1;
    blocks_dim.y = (rg_builder->n_dim[1]-1) / threads_dim.y + 1;
    blocks_dim.z = 1;

    printf("\tBuild image roi grid with %dx%d threads and %dx%d blocks\n",
            threads_dim.x, threads_dim.y, blocks_dim.x, blocks_dim.y);

    int64_t tic = utime_now();
    build<<<blocks_dim, threads_dim>>>(*rg_builder, *grid, *db);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc = utime_now();
    double t_ms = (toc - tic)/1e3;
    printf("\tTook %5.3f ms to build image roi grid\n", t_ms);

    return grid;
}

