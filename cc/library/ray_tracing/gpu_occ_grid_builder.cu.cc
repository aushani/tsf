#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <thrust/device_ptr.h>
#include <thrust/device_free.h>
#include <thrust/unique.h>
#include <thrust/remove.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/merge.h>
#include <thrust/functional.h>
#include <thrust/execution_policy.h>
#include <thrust/system_error.h>

#include "gpu_occ_grid_builder.h"

#include "util.h"
#include "library/gpu_util/gpu_util.cu.h"

gpu_occ_grid_builder_t*
gpu_occ_grid_builder_create(int max_hits, int threads_per_block,
                            int max_hits_per_ray, float resolution,
                            int n_dim[3]) {

    gpu_occ_grid_builder_t* builder = (gpu_occ_grid_builder_t*) malloc(sizeof(gpu_occ_grid_builder_t));

    builder->threads_per_block = threads_per_block;

    builder->resolution_base = resolution;
    builder->n_dim_base[0] = n_dim[0];
    builder->n_dim_base[1] = n_dim[1];
    builder->n_dim_base[2] = n_dim[2];

    builder->max_hits = max_hits;

    // Compute max hits per ray based on dimensions of occupancy grid
    if (max_hits_per_ray<0) {
        for (int i=0; i<3; i++)
            if (max_hits_per_ray < n_dim[i])
                max_hits_per_ray = n_dim[i];
    }

    builder->max_hits_per_ray_base = max_hits_per_ray;

    int num_el = max_hits * max_hits_per_ray;

    builder->verbose = 0;

    int64_t tic_malloc = utime_now();

    // Allocate space for free/occu list
    cudaSafe(cudaMalloc(&builder->d_keys, sizeof(int64_t)*num_el));
    cudaSafe(cudaMalloc(&builder->d_vals, sizeof(float)*num_el));
    cudaSafe(cudaDeviceSynchronize());

    // Allocate space for the velodyne returns
    cudaMalloc(&builder->vr.x, sizeof(float)*max_hits);
    cudaMalloc(&builder->vr.y, sizeof(float)*max_hits);
    cudaMalloc(&builder->vr.z, sizeof(float)*max_hits);

    cudaMalloc(&builder->vr.origin_x, sizeof(float)*max_hits);
    cudaMalloc(&builder->vr.origin_y, sizeof(float)*max_hits);
    cudaMalloc(&builder->vr.origin_z, sizeof(float)*max_hits);

    cudaMalloc(&builder->vr.sensor_num, sizeof(int32_t)*max_hits);
    cudaMalloc(&builder->vr.skip_hits, sizeof(int32_t)*max_hits);

    int64_t toc_malloc = utime_now();
    double t_malloc = (toc_malloc - tic_malloc)/1e3;

    size_t bytes = (sizeof(int64_t) + sizeof(float))*num_el;
    bytes += sizeof(float)*3*max_hits;
    bytes += sizeof(float)*3*max_hits;
    bytes += sizeof(int32_t)*2*max_hits;

    printf("Took %5.3f ms to malloc %ld Mbytes for GPU occ grid builder (max hits = %d)\n",
            t_malloc, bytes/(1024*1024), max_hits_per_ray);

    printf("Ray trace to +- %5.3fx%5.3fx%5.3f m at %5.3f m res with %d max hits\n",
            n_dim[0]/2*resolution, n_dim[1]/2*resolution, n_dim[2]/2*resolution, resolution, max_hits);

    return builder;
}

void
gpu_occ_grid_builder_destroy(gpu_occ_grid_builder_t *builder) {

    cudaFree(builder->d_keys);
    cudaFree(builder->d_vals);

    cudaFree(builder->vr.x);
    cudaFree(builder->vr.y);
    cudaFree(builder->vr.z);

    cudaFree(builder->vr.origin_x);
    cudaFree(builder->vr.origin_y);
    cudaFree(builder->vr.origin_z);

    cudaFree(builder->vr.sensor_num);
    cudaFree(builder->vr.skip_hits);

    if (builder->d_x_vs)
        cudaFree(builder->d_x_vs);

    cudaSafe(cudaDeviceSynchronize());

    free(builder);
}

void
gpu_occ_grid_builder_set_x_vs(gpu_occ_grid_builder_t *builder, float *x_vs, int num_el) {

    if (builder->d_x_vs)
        cudaFree(builder->d_x_vs);

    // Ship x_vs to GPU
    if (cudaMalloc(&builder->d_x_vs, sizeof(float)*num_el) != cudaSuccess) printf("error");
    if (cudaMemcpy(builder->d_x_vs, x_vs, sizeof(float)*num_el, cudaMemcpyHostToDevice) != cudaSuccess) printf("error");

}

__device__ inline void
so3_rotxyz (float R[9], float rph[3]) {
    float sr, sp, sh, cr, cp, ch;
    sincosf (rph[0], &sr, &cr);
    sincosf (rph[1], &sp, &cp);
    sincosf (rph[2], &sh, &ch);

    R[0] = ch*cp;
    R[1] = -sh*cr + ch*sp*sr;
    R[2] = sh*sr + ch*sp*cr;
    R[3] =  sh*cp;
    R[4] =  ch*cr + sh*sp*sr;
    R[5] =  -ch*sr + sh*sp*cr;
    R[6] = -sp;
    R[7] = cp*sr;
    R[8] = cp*cr;
}

__global__ void
ray_trace_rw_gpu(gpu_occ_grid_builder_t builder, gpu_occ_grid_builder_opt_t opt) {

    const int bidx = blockIdx.x;
    const int tidx = threadIdx.x;

    // load ray params
    int hit_i = tidx + bidx*builder.threads_per_block;
    if (hit_i >= builder.vr.num_returns)
        return;

    float hit_world[3] = {builder.vr.x[hit_i],
                          builder.vr.y[hit_i],
                          builder.vr.z[hit_i]};

    float origin_world[3] = {builder.vr.origin_x[hit_i],
                             builder.vr.origin_y[hit_i],
                             builder.vr.origin_z[hit_i]};

    // Compute position of hit in the world relative to the given origin point of the scan
    float R[9];
    so3_rotxyz(R, &builder.vr.pose_xyzrph_scan[3]);

    // Rotate into local frame
    float hit[3] = {hit_world[0], hit_world[1], hit_world[2]};
    hit[0] = R[0]*hit_world[0] + R[1]*hit_world[1] + R[2]*hit_world[2];
    hit[1] = R[3]*hit_world[0] + R[4]*hit_world[1] + R[5]*hit_world[2];
    hit[2] = R[6]*hit_world[0] + R[7]*hit_world[1] + R[8]*hit_world[2];

    // Add translation
    hit[0] += builder.vr.pose_xyzrph_scan[0];
    hit[1] += builder.vr.pose_xyzrph_scan[1];
    hit[2] += builder.vr.pose_xyzrph_scan[2];

    // Compute where laser shot from relative to origin point of scan
    float origin[3] = {origin_world[0], origin_world[1], origin_world[2]};

    // Rotate into local frame
    origin[0] = R[0]*origin_world[0] + R[1]*origin_world[1] + R[2]*origin_world[2];
    origin[1] = R[3]*origin_world[0] + R[4]*origin_world[1] + R[5]*origin_world[2];
    origin[2] = R[6]*origin_world[0] + R[7]*origin_world[1] + R[8]*origin_world[2];

    // Add translation
    origin[0] += builder.vr.pose_xyzrph_scan[0];
    origin[1] += builder.vr.pose_xyzrph_scan[1];
    origin[2] += builder.vr.pose_xyzrph_scan[2];

    float p0[3]; // origin
    float p1[3]; // end of ray
    float ad[3]; // abs of diff * 2
    int8_t sgn[3]; // which way am i going
    int64_t p[3]; // point i'm at (starts at origin)
    uint16_t dominant_dim = 0; // which dim am i stepping through

    for (int8_t i=0; i<3; ++i) {
        p0[i] = llrintf(origin[i] / opt.resolution) + opt.n_dim[i]/2;
        p1[i] = llrintf(hit[i] / opt.resolution) + opt.n_dim[i]/2;

        ad[i] = abs(p1[i] - p0[i]) * 2;

        sgn[i] = (p1[i]>p0[i]) - (p0[i]>p1[i]);

        p[i] = p0[i];

        if (ad[i] > ad[dominant_dim])
            dominant_dim = i;
    }

    float err[3];
    for (int i=0; i<3; i++)
        err[i] =  ad[i] - ad[dominant_dim]/2;

    // walk down ray
    int valid = 1;
    int mem_step_size = builder.vr.num_returns;
    int mem_idx = hit_i;
    int64_t key = -1;
    float val = 0.0;
    for (int step = 0; step<(opt.max_hits_per_ray-1); ++step) {

        // What index of memory are we going to touch?
        //int idx = ((bidx*opt.max_hits_per_ray) + step) * builder.threads_per_block + tidx;

        // are we done?
        if ( (sgn[dominant_dim]>0) ?
                (p[dominant_dim]+0.0f>=p1[dominant_dim]) :
                (p[dominant_dim]+0.0f<=p1[dominant_dim])) {
            valid = 0;
        }

        if (p[0]<0 || p[0] >= opt.n_dim[0]
                || p[1] < 0 || p[1] >= opt.n_dim[1]
                || p[2] < 0 || p[2] >= opt.n_dim[2]) {
            valid = 0;
        }

        if (valid) {
            key = ((p[0]*opt.n_dim[1]) + p[1]) * opt.n_dim[2] + p[2];
            val = l_free;

            // step forwards
            for (int dim=0; dim<3; ++dim) {
                if (dim!=dominant_dim) {
                    if (err[dim] >= 0) {
                        p[dim] += sgn[dim];
                        err[dim] -= ad[dominant_dim];
                    }
                }
            }

            for (int dim=0; dim<3; ++dim) {
                if (dim==dominant_dim) {
                    p[dim] += sgn[dim];
                } else {
                    err[dim] += ad[dim];
                }
            }
        } else { // !valid
            key = -1;
            val = 0.0;
        }

        // Now write out key value pair
        builder.d_keys[mem_idx] = key;
        builder.d_vals[mem_idx] = val;

        mem_idx += mem_step_size;

    }

    p[0] = p1[0];
    p[1] = p1[1];
    p[2] = p1[2];

    if ( (builder.vr.use_skip_hits==0 || builder.vr.skip_hits[hit_i]==0)
            && p[0]>=0 && p[0]<opt.n_dim[0]
            && p[1]>=0 && p[1]<opt.n_dim[1]
            && p[2]>=0 && p[2]<opt.n_dim[2]) {
        key = ((p[0]*opt.n_dim[1]) + p[1]) * opt.n_dim[2] + p[2];
        val = l_occ;
    } else {
        key = -1;
        val = 0.0;
    }

    // Now write out key value pair
    builder.d_keys[mem_idx] = key;
    builder.d_vals[mem_idx] = val;
}

sparse_occ_grid_t*
gpu_occ_grid_builder_accumulate(gpu_occ_grid_builder_t *builder, int num_hits,
        gpu_occ_grid_builder_opt_t opt) {

    int num_el = num_hits * opt.max_hits_per_ray;
    if (builder->verbose) printf("num el: %d\n", num_el);

    thrust::device_ptr<int64_t> dd_keys(builder->d_keys);
    thrust::device_ptr<float> dd_vals(builder->d_vals);

    // Remove unnecesary items
    int64_t tic_presort = utime_now();
    thrust::device_ptr<int64_t> new_keys_end = thrust::remove_if(dd_keys, dd_keys + num_el, is_negative());
    thrust::device_ptr<float> new_vals_end = thrust::remove_if(dd_vals, dd_vals + num_el, is_lteps());
    //int num_keys_left = new_keys_end - dd_keys;
    //int num_vals_left = new_vals_end - dd_vals;
    //printf("%d vs %d\n", num_keys_left, num_vals_left);
    num_el = new_keys_end - dd_keys;
    thrust::sort_by_key(dd_keys, dd_keys + num_el, dd_vals);
    int64_t toc_presort = utime_now();
    double t_presort = (toc_presort - tic_presort)/1e3;
    if (builder->verbose) printf("\tTook %5.3f ms to presort key/values (have %d left)\n", t_presort, num_el);

    // Reduce by key

    // Allocate more
    int64_t *d_keys_out;
    float *d_vals_out;
    cudaMalloc(&d_keys_out, sizeof(int64_t) * num_el);
    cudaMalloc(&d_vals_out, sizeof(float) * num_el);
    cudaMemset(d_keys_out, 0, sizeof(int64_t)*num_el);
    cudaMemset(d_vals_out, 0, sizeof(float) * num_el);

    thrust::device_ptr<int64_t> dd_keys_out(d_keys_out);
    thrust::device_ptr<float> dd_vals_out(d_vals_out);

    thrust::pair<thrust::device_ptr<int64_t>, thrust::device_ptr<float> > new_end;

    int64_t tic_reduce_by_key = utime_now();
    new_end = thrust::reduce_by_key(dd_keys,
                                    dd_keys + num_el,
                                    dd_vals,
                                    dd_keys_out,
                                    dd_vals_out);

    int key_value_pairs = new_end.first - dd_keys_out;
    if (builder->verbose) printf("\tHave %d key value pairs\n", key_value_pairs);
    int64_t toc_reduce_by_key = utime_now();
    double t_reduce_by_key = (toc_reduce_by_key - tic_reduce_by_key)/1e3;
    if (builder->verbose) printf("\tTime to reduce by key is %5.3f ms\n", t_reduce_by_key);

    // Is it already sorted?
    int64_t tic_verifysorted = utime_now();
    bool sorted = thrust::is_sorted(dd_keys_out, dd_keys_out + key_value_pairs);
    int64_t toc_verifysorted = utime_now();
    double t_verifysorted = (toc_verifysorted - tic_verifysorted)/1e3;
    if (builder->verbose) printf("\tTook %5.3f ms to verify keys are %s\n", t_verifysorted, sorted ? "sorted":"not sorted");

    if (!sorted) {
        // Sort array
        int64_t tic_sort = utime_now();
        thrust::sort_by_key(dd_keys_out, dd_keys_out + key_value_pairs, dd_vals_out);
        int64_t toc_sort = utime_now();
        double t_sort = (toc_sort - tic_sort)/1e3;
        if (builder->verbose) printf("\tTook %5.3f ms to sort key/values\n", t_sort);
    }

    sparse_occ_grid_t *grid = sparse_occ_grid_create(opt.resolution, opt.n_dim);

    //grid->d_keys = d_keys_out;
    //grid->d_vals = d_vals_out;
    cudaMalloc(&grid->d_keys, sizeof(int64_t)*key_value_pairs);
    cudaMalloc(&grid->d_vals, sizeof(float)*key_value_pairs);
    cudaMemcpy(grid->d_keys, d_keys_out, sizeof(int64_t)*key_value_pairs, cudaMemcpyDeviceToDevice);
    cudaMemcpy(grid->d_vals, d_vals_out, sizeof(float)*key_value_pairs, cudaMemcpyDeviceToDevice);
    cudaFree(d_keys_out);
    cudaFree(d_vals_out);
    grid->n_kv = key_value_pairs;

    // Copy to host memory
    int64_t tic_copy = utime_now();
    grid->keys = (int64_t*) malloc(sizeof(int64_t)*key_value_pairs);
    grid->vals = (float*) malloc(sizeof(float)*key_value_pairs);
    cudaMemcpy(grid->keys, grid->d_keys, sizeof(int64_t) * key_value_pairs, cudaMemcpyDeviceToHost);
    cudaMemcpy(grid->vals, grid->d_vals, sizeof(float) * key_value_pairs, cudaMemcpyDeviceToHost);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_copy = utime_now();
    double t_copy = (toc_copy - tic_copy)/1e3;
    if (builder->verbose) printf("\tTook %5.3f ms to copy to host\n", t_copy);

    return grid;
}

void
gpu_occ_grid_builder_ship_vr(gpu_occ_grid_builder_t *builder, const velodyne_returns_t *vr, double pose[6], int32_t *skip_hits) {

    int64_t tic_ship_vr = utime_now();

    float *x = (float*) malloc(sizeof(float)*vr->num_returns);
    float *y = (float*) malloc(sizeof(float)*vr->num_returns);
    float *z = (float*) malloc(sizeof(float)*vr->num_returns);

    float *origin_x = (float*) malloc(sizeof(float)*vr->num_returns);
    float *origin_y = (float*) malloc(sizeof(float)*vr->num_returns);
    float *origin_z = (float*) malloc(sizeof(float)*vr->num_returns);

    int64_t tic_reorder = utime_now();
    for (int i=0; i<vr->num_returns; ++i) {
        x[i] = vr->xyz[3*i + 0];
        y[i] = vr->xyz[3*i + 1];
        z[i] = vr->xyz[3*i + 2];

        //origin_x[i] = vr->origin_xyz[3*i + 0];
        //origin_y[i] = vr->origin_xyz[3*i + 1];
        //origin_z[i] = vr->origin_xyz[3*i + 2];
        origin_x[i] = 0;
        origin_y[i] = 0;
        origin_z[i] = 1.73;
    }
    int64_t toc_reorder = utime_now();
    double t_reorder = (toc_reorder - tic_reorder)/1e3;
    if (builder->verbose) printf("Took %5.3f to reorder xyz's and origin's\n", t_reorder);

    if (cudaMemcpy(builder->vr.x, x, sizeof(float)*vr->num_returns, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.y, y, sizeof(float)*vr->num_returns, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.z, z, sizeof(float)*vr->num_returns, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");

    if (cudaMemcpy(builder->vr.origin_x, origin_x, sizeof(float)*vr->num_returns, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.origin_y, origin_y, sizeof(float)*vr->num_returns, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.origin_z, origin_z, sizeof(float)*vr->num_returns, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");

    //cudaMemcpy(builder->vr.sensor_num, vr->sensor_num, sizeof(int32_t)*vr->num_returns, cudaMemcpyHostToDevice);
    cudaMemset(builder->vr.sensor_num, 0, sizeof(int32_t)*vr->num_returns);
    if (skip_hits) {
        builder->vr.use_skip_hits = 1;
        cudaMemcpy(builder->vr.skip_hits, skip_hits, sizeof(int32_t)*vr->num_returns, cudaMemcpyHostToDevice);
        printf("using skip hits\n");
    } else {
        builder->vr.use_skip_hits = 0;
        cudaMemset(builder->vr.skip_hits, 0, sizeof(uint16_t)*vr->num_returns);
    }

    builder->vr.num_returns = vr->num_returns;

    builder->vr.pose_xyzrph_scan[0] = pose[0];
    builder->vr.pose_xyzrph_scan[1] = pose[1];
    builder->vr.pose_xyzrph_scan[2] = pose[2];
    builder->vr.pose_xyzrph_scan[3] = pose[3];
    builder->vr.pose_xyzrph_scan[4] = pose[4];
    builder->vr.pose_xyzrph_scan[5] = pose[5];

    // Cleanup
    free(x);
    free(y);
    free(z);
    free(origin_x);
    free(origin_y);
    free(origin_z);

    cudaSafe(cudaDeviceSynchronize());

    int64_t toc_ship_vr = utime_now();
    double t_ship_vr = (toc_ship_vr - tic_ship_vr)/1e3;

    if (builder->verbose) printf("Took %5.3f ms to send vr to GPU\n", t_ship_vr);
}

sparse_occ_grid_t*
gpu_occ_grid_builder_build(gpu_occ_grid_builder_t *builder, const velodyne_returns_t *vr,
        double pose[6], int32_t *skip_hits, int64_t utime) {

    sparse_occ_grid_multires_t* g_mr = gpu_occ_grid_builder_build_multires(builder, vr, pose, skip_hits, 1, utime);
    sparse_occ_grid_t *g = sparse_occ_grid_multires_get_level(g_mr, 0);

    sparse_occ_grid_multires_destroy_shallow(g_mr);

    return g;
}

sparse_occ_grid_multires_t*
gpu_occ_grid_builder_build_multires(gpu_occ_grid_builder_t *builder, const velodyne_returns_t *vr,
        double pose[6], int32_t *skip_hits, int n_levels, int64_t utime) {

    if (builder->verbose) printf("starting gpu ray trace...\n");
    int64_t tic_total = utime_now();

    sparse_occ_grid_multires_t *g_mr = sparse_occ_grid_multires_create(n_levels);

    int threads = builder->threads_per_block;
    int blocks = vr->num_returns / builder->threads_per_block + 1;

    int64_t toc_setup = utime_now();
    double t_setup = (toc_setup - tic_total)/1e3;
    if (builder->verbose) printf("Setup took %5.3f ms\n", t_setup);

    // Ship velodyne hits to GPU
    gpu_occ_grid_builder_ship_vr(builder, vr, pose, skip_hits);

    int64_t toc_sofar = utime_now();
    double t_sofar = (toc_sofar - tic_total)/1e3;
    if (builder->verbose) printf("So far at total time of %5.3f ms\n", t_sofar);

    // Build every level sequentially
    gpu_occ_grid_builder_opt_t opt;
    opt.resolution = builder->resolution_base;
    opt.n_dim[0] = builder->n_dim_base[0];
    opt.n_dim[1] = builder->n_dim_base[1];
    opt.n_dim[2] = builder->n_dim_base[2];
    opt.max_hits_per_ray = builder->max_hits_per_ray_base;

    for (int i=0; i<n_levels; i++) {

        // Do actual ray tracing
        int64_t tic_ray_trace = utime_now();
        ray_trace_rw_gpu<<<blocks, threads>>>(*builder, opt);
        cudaSafe(cudaDeviceSynchronize());
        int64_t toc_ray_trace = utime_now();
        double t_ray_trace = (toc_ray_trace - tic_ray_trace)/1e3;
        if (builder->verbose) printf("Ray tracing took %5.3f ms\n", t_ray_trace);

        // Now accumulate everything
        int64_t tic_build = utime_now();
        g_mr->grids[i] = gpu_occ_grid_builder_accumulate(builder, vr->num_returns, opt);
        cudaSafe(cudaDeviceSynchronize());
        int64_t toc_build = utime_now();
        double t_build = (toc_build - tic_build)/1e3;
        if (builder->verbose) printf("accumlating result took %5.3f ms\n", t_build);

        // Add in the given timestamp and pose
        g_mr->grids[i]->utime = utime;
        for (int pose_dim=0; pose_dim<6; pose_dim++) {
            g_mr->grids[i]->pose[pose_dim] = pose[pose_dim];
        }

        // setup next level
        opt.resolution *= 2;
        opt.n_dim[0] = ceil(opt.n_dim[0]/2.0);
        opt.n_dim[1] = ceil(opt.n_dim[1]/2.0);
        opt.n_dim[2] = ceil(opt.n_dim[2]/2.0);
        opt.max_hits_per_ray = ceil(opt.max_hits_per_ray/2.0);
    }

    int64_t toc_total = utime_now();
    double t_total = (toc_total - tic_total)/1e3;
    if (builder->verbose) printf("ray tracing for %d points at %d levels took with %d gpu threads and %d blocks %5.3f ms total\n",
            vr->num_returns, n_levels, threads, blocks, t_total);


    return g_mr;
}

sparse_occ_grid_t*
gpu_occ_grid_builder_add(gpu_occ_grid_builder_t *builder,
        const float *hits_x, const float *hits_y, const float *hits_z,
        const float *origin_x, const float *origin_y, const float *origin_z,
        int n_hits, sparse_occ_grid_t *sog) {

    // Ship data

    int tic_ship_vr = utime_now();

    if (cudaMemcpy(builder->vr.x, hits_x, sizeof(float)*n_hits, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.y, hits_y, sizeof(float)*n_hits, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.z, hits_z, sizeof(float)*n_hits, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");

    if (cudaMemcpy(builder->vr.origin_x, origin_x, sizeof(float)*n_hits, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.origin_y, origin_y, sizeof(float)*n_hits, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");
    if (cudaMemcpy(builder->vr.origin_z, origin_z, sizeof(float)*n_hits, cudaMemcpyHostToDevice) != cudaSuccess) printf("error\n");

    builder->vr.pose_xyzrph_scan[0] = 0;
    builder->vr.pose_xyzrph_scan[1] = 0;
    builder->vr.pose_xyzrph_scan[2] = 0;
    builder->vr.pose_xyzrph_scan[3] = 0;
    builder->vr.pose_xyzrph_scan[4] = 0;
    builder->vr.pose_xyzrph_scan[5] = 0;

    builder->vr.use_skip_hits = 0;

    builder->vr.num_returns = n_hits;

    int toc_ship_vr = utime_now();
    double t_ship_vr = (toc_ship_vr - tic_ship_vr)/1e3;
    if (builder->verbose) printf("Took %5.3f ms to send vr to GPU\n", t_ship_vr);

    // Set up options
    gpu_occ_grid_builder_opt_t opt;
    opt.resolution = builder->resolution_base;
    opt.n_dim[0] = builder->n_dim_base[0];
    opt.n_dim[1] = builder->n_dim_base[1];
    opt.n_dim[2] = builder->n_dim_base[2];
    opt.max_hits_per_ray = builder->max_hits_per_ray_base;

    int threads = builder->threads_per_block;
    int blocks = n_hits / builder->threads_per_block + 1;

    // Do actual ray tracing
    int64_t tic_ray_trace = utime_now();
    ray_trace_rw_gpu<<<blocks, threads>>>(*builder, opt);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_ray_trace = utime_now();
    double t_ray_trace = (toc_ray_trace - tic_ray_trace)/1e3;
    if (builder->verbose) printf("Ray tracing took %5.3f ms\n", t_ray_trace);

    // Now accumulate this scan
    int64_t tic_build = utime_now();
    sparse_occ_grid_t *sog_scan = gpu_occ_grid_builder_accumulate(builder, n_hits, opt);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_build = utime_now();
    double t_build = (toc_build - tic_build)/1e3;
    if (builder->verbose) printf("accumlating result took %5.3f ms\n", t_build);

    if (sog == NULL) {
        return sog_scan;
    }

    // Do merging with old scan here
    sparse_occ_grid_t *res = sparse_occ_grid_merge(sog, sog_scan);
    sparse_occ_grid_destroy(sog_scan);

    return res;

}
