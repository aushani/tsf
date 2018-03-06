#include <stdio.h>

#include <Eigen/Core>
#include "thirdparty/perls-math/ssc.h"

#include <cstdlib>

#include "sparse_occ_grid.h"

#include "util.h"

#include "library/gpu_util/gpu_util.cu.h"

#include <thrust/device_vector.h>
#include <thrust/reduce.h>
#include <thrust/merge.h>
#include <thrust/remove.h>
#include <thrust/sort.h>
#include <thrust/count.h>

sparse_occ_grid_t*
sparse_occ_grid_create(float resolution, int64_t n_dim[3]) {

    sparse_occ_grid_t *grid = (sparse_occ_grid_t*) malloc(sizeof(*grid));

    grid->n_dim[0] = n_dim[0];
    grid->n_dim[1] = n_dim[1];
    grid->n_dim[2] = n_dim[2];
    grid->res = resolution;

    // 0 out pose
    grid->pose[0] = 0.0;
    grid->pose[1] = 0.0;
    grid->pose[2] = 0.0;
    grid->pose[3] = 0.0;
    grid->pose[4] = 0.0;
    grid->pose[5] = 0.0;

    grid->n_kv = 0;

    grid->keys = NULL;
    grid->vals = NULL;

    grid->d_keys = NULL;
    grid->d_vals = NULL;

    grid->filter = NULL;
    grid->d_filter = NULL;

    return grid;
}

sparse_occ_grid_t*
sparse_occ_grid_copy(sparse_occ_grid_t *grid, int want_host, int want_device) {

    sparse_occ_grid_t *res = sparse_occ_grid_create(grid->res, grid->n_dim);

    // Copy over pose
    res->pose[0] = grid->pose[0];
    res->pose[1] = grid->pose[1];
    res->pose[2] = grid->pose[2];
    res->pose[3] = grid->pose[3];
    res->pose[4] = grid->pose[4];
    res->pose[5] = grid->pose[5];

    res->n_kv = grid->n_kv;

    if (want_host && grid->keys) {
        res->keys = (int64_t*) malloc(sizeof(int64_t)*res->n_kv);
        res->vals = (float*) malloc(sizeof(float)*res->n_kv);

        memcpy(res->keys, grid->keys, sizeof(int64_t)*res->n_kv);
        memcpy(res->vals, grid->vals, sizeof(float)*res->n_kv);

        if (grid->filter != NULL) {
            res->filter = (int*) malloc(sizeof(int)*grid->n_dim[0]*grid->n_dim[1]);
            memcpy(res->filter, grid->filter, sizeof(int)*grid->n_dim[0]*grid->n_dim[1]);
        }
    } else {
        res->keys = NULL;
        res->vals = NULL;
        res->filter = NULL;
    }

    if (want_device && grid->d_keys) {
        cudaMalloc(&res->d_keys, sizeof(int64_t)*res->n_kv);
        cudaMalloc(&res->d_vals, sizeof(float)*res->n_kv);

        cudaMemcpy(res->d_keys, grid->d_keys, sizeof(int64_t)*res->n_kv, cudaMemcpyDeviceToDevice);
        cudaMemcpy(res->d_vals, grid->d_vals, sizeof(float)*res->n_kv, cudaMemcpyDeviceToDevice);

        if (grid->d_filter != NULL) {
            cudaMalloc(&res->d_filter, sizeof(int)*grid->n_dim[0]*grid->n_dim[1]);
            cudaMemcpy(res->d_filter, grid->d_filter, sizeof(int)*grid->n_dim[0]*grid->n_dim[1], cudaMemcpyDeviceToDevice);
        }

    } else {
        res->d_keys = NULL;
        res->d_vals = NULL;
        res->d_filter = NULL;
    }

    return res;
}

void
sparse_occ_grid_destroy(sparse_occ_grid_t *grid) {

    if (grid->keys) free(grid->keys);
    if (grid->vals) free(grid->vals);

    if (grid->d_keys) cudaFree(grid->d_keys);
    if (grid->d_vals) cudaFree(grid->d_vals);

    if (grid->filter) free(grid->filter);
    if (grid->d_filter) cudaFree(grid->d_filter);

    free(grid);
}

float
sparse_occ_grid_lookup_helper(sparse_occ_grid_t *grid, int64_t idx) {

    // binary search

    int i = 0;
    int j = grid->n_kv;

    while (i!=j) {

        // Find midpoint
        int k = (i+j)>>1;

        //printf("%d, %d, %d\n", i, j, k);

        // Branch
        int64_t k_k = grid->keys[k];
        if (k_k < idx)
            i = k+1;
        else if (k_k > idx)
            j = k;
        else if (k_k == idx)
            return grid->vals[k];

    }

    if (grid->keys[i] == idx)
        return grid->vals[i];

    if (grid->keys[j] == idx)
        return grid->vals[j];

    return 0.0;
}

__device__ float
sparse_occ_grid_device_lookup_helper(sparse_occ_grid_t grid, int64_t idx) {

    // binary search

    int i = 0;
    int j = grid.n_kv;

    while (i!=j) {

        // Find midpoint
        int k = (i+j)>>1;

        //printf("%d, %d, %d\n", i, j, k);

        // Branch
        int64_t k_k = grid.d_keys[k];
        if (k_k < idx)
            i = k+1;
        else if (k_k > idx)
            j = k;
        else if (k_k == idx)
            return grid.d_vals[k];

    }

    if (grid.d_keys[i] == idx)
        return grid.d_vals[i];

    if (grid.d_keys[j] == idx)
        return grid.d_vals[j];

    return 0.0;
}

float
sparse_occ_grid_lookup(sparse_occ_grid_t *grid, int i, int j, int k) {

    if (i<0 || i>=grid->n_dim[0]) return 0.0;
    if (j<0 || j>=grid->n_dim[1]) return 0.0;
    if (k<0 || k>=grid->n_dim[2]) return 0.0;

    int64_t idx = ((i*grid->n_dim[1]) + j )*grid->n_dim[2] + k;
    //printf("lookup %d, %d, %d = %d\n", i, j, k, idx);

    return sparse_occ_grid_lookup_helper(grid, idx);
}

void
sparse_occ_grid_it_init(sparse_occ_grid_t *grid, sparse_occ_grid_it_t *git,
        int want_dense) {

    git->grid = grid;

    git->done = 0;

    git->want_dense = want_dense;

    git->idx_at = 0;

    if (want_dense) {
        git->max_idx = grid->n_dim[0]*grid->n_dim[1]*grid->n_dim[2];
        if (git->max_idx==0)
            git->done = 1;
    } else {
        if (grid->n_kv == 0)
            git->done = 1;

        git->max_idx = grid->keys[grid->n_kv-1];
    }

    git->map_index_at = 0;
    git->key_at = git->grid->keys[0];

    git->use_filter = 0;
}

void
sparse_occ_grid_it_use_filter(sparse_occ_grid_it_t *git, int use_filter) {

    git->use_filter = use_filter;
}

uint8_t
sparse_occ_grid_it_next(sparse_occ_grid_it_t *git, int64_t *key, float *val) {

    int64_t idx;
    float v = 0.0;

    // could be smarter
    do {

        if (git->done)
            return 0;

        // Figure out which index we're now looking at
        if (git->want_dense) {
            idx = git->idx_at++;
        } else {
            idx = git->key_at;
            git->idx_at = idx;
        }

        // Is this the next element in our list of keys?
        if (idx == git->key_at) {

            // Grab the value
            v = git->grid->vals[git->map_index_at];

            // Move on to the next element
            git->map_index_at++;

            // Grab the next key
            if (git->map_index_at < git->grid->n_kv)
                git->key_at = git->grid->keys[git->map_index_at];
        }

        git->done = git->idx_at >= git->max_idx;

    } while (git->use_filter && sparse_occ_grid_check_filter(git->grid, idx));

    if (key) *key = idx;
    if (val) *val = v;

    return 1;
}

int
sparse_occ_grid_count_nonzero(sparse_occ_grid_t *grid) {

    int count = 0;

    for (int i=0; i<grid->n_kv; i++) {

        if (fabs(grid->vals[i]) > 1e-3) {
            count++;
        }
    }

    return count;
}

void
sparse_occ_grid_save(sparse_occ_grid_t *grid, char* filename) {

    FILE *fp = fopen(filename, "w");

    float gpu;
    sparse_occ_grid_it_t git;
    sparse_occ_grid_it_init(grid, &git, 1); // want dense

    while (sparse_occ_grid_it_next(&git, NULL, &gpu)) {
        if (gpu>0.1)
            fprintf(fp, "%d\n", 1);
        else if (gpu<-0.1)
            fprintf(fp, "%d\n", -1);
        else
            fprintf(fp, "%d\n", 0);
    }

    fclose(fp);
}

int32_t*
sparse_occ_grid_get_dense_slice(sparse_occ_grid_t *grid, int row) {

    int32_t *res = (int32_t*) malloc(sizeof(int32_t)*grid->n_dim[0]*grid->n_dim[1]);

    for (int i=0; i<grid->n_dim[0]; i++) {
        for (int j=0; j<grid->n_dim[1]; j++) {
            float val = sparse_occ_grid_lookup(grid, i, j, row);

            if (val>0)
                res[i*grid->n_dim[1] + j] = 1;
            else if (val<0)
                res[i*grid->n_dim[1] + j] = -1;
            else
                res[i*grid->n_dim[1] + j] = 0;

        }
    }

    return res;
}

__global__ void
sparse_occ_grid_populate_dense(sparse_occ_grid_t grid, float *d_grid, int col_coalesce) {

    const int bidx = blockIdx.x;
    const int tidx = threadIdx.x;

    int threads_per_block = blockDim.x;
    int64_t idx = tidx + bidx*threads_per_block;

    int n3 = grid.n_dim[0] * grid.n_dim[1] * grid.n_dim[2];

    // Check bounds
    if (idx >= n3) {
        return;
    }

    // Where should we put the result?
    // This really only matters if we're trying to coalesce by column
    int64_t idx_put = idx;
    if (col_coalesce==1) {

        // Get the pixel we're at in the dense grid
        int i = idx / (grid.n_dim[1]*grid.n_dim[2]);
        int j = (idx - i*grid.n_dim[1]*grid.n_dim[2]) / grid.n_dim[2];
        int k = idx - (i*grid.n_dim[1] + j)*grid.n_dim[2];

        // Change order of indexing
        idx_put = (k*grid.n_dim[0] + i)*grid.n_dim[1] + j;
    }

    float ll = sparse_occ_grid_device_lookup_helper(grid, idx);

    // Clip
    if (ll > l_max) ll = l_max;
    if (ll < l_min) ll = l_min;

    float p = 1.0f - 1.0f / (1+expf(ll));

    // Check filter value
    if (grid.d_filter != NULL) {
        int i = idx / (grid.n_dim[1]*grid.n_dim[2]);
        int j = (idx - i*grid.n_dim[1]*grid.n_dim[2]) / grid.n_dim[2];

        int64_t idx_ij = i*grid.n_dim[1] + j;

        if (grid.d_filter[idx_ij])
            p = 0.5;
    }

    d_grid[idx_put] = p;
}

__global__ void
sparse_occ_grid_populate_mle_state(sparse_occ_grid_t grid, int *d_grid, int col_coalesce) {

    const int bidx = blockIdx.x;
    const int tidx = threadIdx.x;

    int threads_per_block = blockDim.x;
    int64_t idx = tidx + bidx*threads_per_block;

    int n3 = grid.n_dim[0] * grid.n_dim[1] * grid.n_dim[2];

    // Check bounds
    if (idx >= n3) {
        return;
    }

    // Where should we put the result?
    // This really only matters if we're trying to coalesce by column
    int64_t idx_put = idx;
    if (col_coalesce==1) {

        // Get the pixel we're at in the dense grid
        int i = idx / (grid.n_dim[1]*grid.n_dim[2]);
        int j = (idx - i*grid.n_dim[1]*grid.n_dim[2]) / grid.n_dim[2];
        int k = idx - (i*grid.n_dim[1] + j)*grid.n_dim[2];

        // Change order of indexing
        idx_put = (k*grid.n_dim[0] + i)*grid.n_dim[1] + j;
    }

    float ll = sparse_occ_grid_device_lookup_helper(grid, idx);

    int val = S_UNKN;

    if (ll < 0) val = S_FREE; // free
    if (ll > 0) val = S_OCCU; // occupied

    // Check filter value
    if (grid.d_filter != NULL) {
        int i = idx / (grid.n_dim[1]*grid.n_dim[2]);
        int j = (idx - i*grid.n_dim[1]*grid.n_dim[2]) / grid.n_dim[2];

        int64_t idx_ij = i*grid.n_dim[1] + j;

        if (grid.d_filter[idx_ij])
            val = S_UNKN;
    }

    d_grid[idx_put] = val;
}

void
sparse_occ_grid_get_dense_device(sparse_occ_grid_t *grid, float *d_grid, int col_coalesce) {

    if (grid->d_keys == NULL || grid->d_vals == NULL)
        printf("WARNING: Don't have memory on device\n");

    int n3 = grid->n_dim[0] * grid->n_dim[1] * grid->n_dim[2];

    int threads = 1024;
    int blocks = n3 / threads + 1;

    //printf("Populating dense with %d threads and %d blocks\n", threads, blocks);
    sparse_occ_grid_populate_dense<<<blocks, threads>>>(*grid, d_grid, col_coalesce);
    cudaSafe(cudaDeviceSynchronize());
}

void
sparse_occ_grid_get_dense_device_mle(sparse_occ_grid_t *grid, int *d_grid, int col_coalesce) {

    if (grid->d_keys == NULL || grid->d_vals == NULL)
        printf("WARNING: Don't have memory on device\n");

    int n3 = grid->n_dim[0] * grid->n_dim[1] * grid->n_dim[2];

    int threads = 1024;
    int blocks = n3 / threads + 1;

    //printf("Populating dense with %d threads and %d blocks\n", threads, blocks);
    sparse_occ_grid_populate_mle_state<<<blocks, threads>>>(*grid, d_grid, col_coalesce);
    cudaSafe(cudaDeviceSynchronize());
}

__global__ void
sparse_occ_grid_populate_dense_binary(sparse_occ_grid_t grid, int *d_grid, int col_coalesce) {

    const int bidx = blockIdx.x;
    const int tidx = threadIdx.x;

    int threads_per_block = blockDim.x;
    int64_t idx = tidx + bidx*threads_per_block;

    int n3 = grid.n_dim[0] * grid.n_dim[1] * grid.n_dim[2];

    // Check bounds
    if (idx >= n3) {
        return;
    }

    // Where should we put the result?
    // This really only matters if we're trying to coalesce by column
    int64_t idx_put = idx;
    if (col_coalesce==1) {

        // Get the pixel we're at in the dense grid
        int i = idx / (grid.n_dim[1]*grid.n_dim[2]);
        int j = (idx - i*grid.n_dim[1]*grid.n_dim[2]) / grid.n_dim[2];
        int k = idx - (i*grid.n_dim[1] + j)*grid.n_dim[2];

        // Change order of indexing
        idx_put = (k*grid.n_dim[0] + i)*grid.n_dim[1] + j;
    }

    float ll = sparse_occ_grid_device_lookup_helper(grid, idx);

    int p = 0.0;

    if (ll > 0) p = 1.0;
    if (ll < 0) p = -1.0;

    d_grid[idx_put] = p;

}

void
sparse_occ_grid_get_dense_device_binary(sparse_occ_grid_t *grid, int *d_grid, int col_coalesce) {

    if (grid->d_keys == NULL || grid->d_vals == NULL)
        printf("WARNING: Don't have memory on device\n");

    int n3 = grid->n_dim[0] * grid->n_dim[1] * grid->n_dim[2];

    int threads = 1024;
    int blocks = n3 / threads + 1;

    //printf("Populating dense with %d threads and %d blocks\n", threads, blocks);
    sparse_occ_grid_populate_dense_binary<<<blocks, threads>>>(*grid, d_grid, col_coalesce);
    cudaSafe(cudaDeviceSynchronize());
}

void
sparse_occ_grid_idx_to_ijk(sparse_occ_grid_t *grid, int64_t idx, int *i, int *j, int *k) {

    int tmp_i = (idx / (grid->n_dim[1]*grid->n_dim[2]));
    int tmp_j = ((idx - tmp_i*grid->n_dim[1]*grid->n_dim[2]) / grid->n_dim[2]);
    int tmp_k = (idx - (tmp_i*grid->n_dim[1] + tmp_j)*grid->n_dim[2]);

    if (i) *i = tmp_i;
    if (j) *j = tmp_j;
    if (k) *k = tmp_k;
}

void
sparse_occ_grid_ijk_to_idx(sparse_occ_grid_t *grid, int i, int j, int k, int64_t *idx) {

    if (i<0 || i>=grid->n_dim[0]) *idx = -1;
    else if (j<0 || j>=grid->n_dim[1]) *idx = -1;
    else if (k<0 || k>=grid->n_dim[2]) *idx = -1;
    else {
        *idx = i*grid->n_dim[1]*grid->n_dim[2] + j*grid->n_dim[2] + k;
    }
}

void
sparse_occ_grid_get_xyz(sparse_occ_grid_t *grid, int64_t idx, double *x, double *y, double *z) {

    int i, j, k;
    sparse_occ_grid_idx_to_ijk(grid, idx, &i, &j, &k);

    // In a 5x5x5 occ grid, index (2, 2, 2) should be in the center (ie, res/2)

    *x = (i-grid->n_dim[0]/2) * grid->res; //+ grid->res/2;
    *y = (j-grid->n_dim[1]/2) * grid->res; //+ grid->res/2;
    *z = (k-grid->n_dim[2]/2) * grid->res; //+ grid->res/2;
}

void
sparse_occ_grid_get_xyz(sparse_occ_grid_t *grid, int i, int j, int k, double *x, double *y, double *z) {

    // In a 5x5x5 occ grid, index (2, 2, 2) should be in the center (ie, res/2)

    *x = (i-grid->n_dim[0]/2) * grid->res; //+ grid->res/2;
    *y = (j-grid->n_dim[1]/2) * grid->res; //+ grid->res/2;
    *z = (k-grid->n_dim[2]/2) * grid->res; //+ grid->res/2;
}

void
sparse_occ_grid_center_xyz(sparse_occ_grid_t *grid, double x, double y, double z,
        double *x_c, double *y_c, double *z_c) {

    int i, j, k;
    sparse_occ_grid_get_ijk(grid, x, y, z, &i, &j, &k);

    int64_t idx = ((i*grid->n_dim[1]) + j )*grid->n_dim[2] + k;

    sparse_occ_grid_get_xyz(grid, idx, x_c, y_c, z_c);
}

void
sparse_occ_grid_get_ijk(sparse_occ_grid_t *grid, double x, double y, double z, int *i, int *j, int *k) {

    *i = round((x)/grid->res) + grid->n_dim[0]/2;
    *j = round((y)/grid->res) + grid->n_dim[1]/2;
    *k = round((z)/grid->res) + grid->n_dim[2]/2;
}

int
sparse_occ_grid_count_occ_in_col(sparse_occ_grid_t *grid, int i, int j) {

    int count = 0;
    for (int k=0; k<grid->n_dim[2]; k++) {
        if (sparse_occ_grid_lookup(grid, i, j, k)>l_max)
            count++;
    }

    return count;
}

__global__ void
sparse_occ_grid_build_filter_device(sparse_occ_grid_t grid, float *wf_d, float *wo_d,
        float b, int window_size) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;
    //const int bidz = blockIdx.z;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;
    //const int tidz = threadIdx.z;

    const int i = bidx * blockDim.x + tidx;
    const int j = bidy * blockDim.y + tidy;

    extern __shared__ int8_t og_state[];

    // First load shared memory

    // shared memory params
    const int i0 = bidx * blockDim.x;
    const int j0 = bidy * blockDim.y;

    const int i0_n = i0 - window_size;
    const int j0_n = j0 - window_size;

    //const int s_sz_x = window_size*2 + blockDim.x;
    const int s_sz_y = window_size*2 + blockDim.y;
    //const int s_sz = s_sz_x * s_sz_y;

    for (int di=-1; di<=1; di++) {

        // Where are we loading this time for shared memory?
        int i_n = i + di*window_size;

        // Make sure this is in range before we do anything
        if (i_n<0 || i_n>=grid.n_dim[0])
            continue;

        for (int dj=-1; dj<=1; dj++) {

            // Where are we loading this time for shared memory?
            int j_n = j + dj*window_size;

            // Make sure this is in range before we do anything
            if (j_n<0 || j_n>=grid.n_dim[1])
                continue;

            // Get index in our shared memory
            int idx_n_0 = (i_n - i0_n)*s_sz_y + (j_n - j0_n);
            idx_n_0 *= grid.n_dim[2];

            // Get index in global memory
            int idx_0 = ((i_n*grid.n_dim[1]) + j_n)*grid.n_dim[2];

            for (int k=0; k<grid.n_dim[2]; k++) {

                float v = sparse_occ_grid_device_lookup_helper(grid, idx_0 + k);

                if (v < 0)
                    og_state[idx_n_0 + k] = S_FREE;
                else if (v > 0)
                    og_state[idx_n_0 + k] = S_OCCU;
                else
                    og_state[idx_n_0 + k] = S_UNKN;

            }
        }
    }

    __syncthreads();
    // Now we're donig loading shared memory

    if (i >= grid.n_dim[0] || j >= grid.n_dim[1])
        return;

    // Get column of data and compute score
    float score = b;

    int idx_window = 0;

    for (int di = -window_size; di<=window_size; di++) {
        for (int dj = -window_size; dj<=window_size; dj++) {

            int i_w = i + di;
            int j_w = j + dj;

            int idx_n = (i_w - i0_n)*s_sz_y + (j_w - j0_n);
            idx_n *= grid.n_dim[2];

            for (int k = 0; k < grid.n_dim[2]; k++) {

                int8_t state = og_state[idx_n + k];

                float w = 0.0;

                if (state == S_FREE) w = wf_d[idx_window];
                if (state == S_OCCU) w = wo_d[idx_window];

                score += w;

                idx_window++;
            }
        }
    }

    // Populate filter
    grid.d_filter[i*grid.n_dim[1] + j] = score>0 ? 0:1;
}

__global__ void
sparse_occ_grid_smooth_filter_device(sparse_occ_grid_t grid, int *res) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;
    //const int bidz = blockIdx.z;

    const int tidx = threadIdx.x;
    const int tidy = threadIdx.y;
    //const int tidz = threadIdx.z;

    const int i = bidx * blockDim.x + tidx;
    const int j = bidy * blockDim.y + tidy;

    if (i >= grid.n_dim[0] || j >= grid.n_dim[1])
        return;

    int count0 = 0;
    int count1 = 0;

    for (int di=-1; di<=1; di++) {
        for (int dj=-1; dj<=1; dj++) {

            int ii = i + di;
            int jj = j + dj;

            if (ii>=0 && ii<grid.n_dim[0] && jj>=0 && jj<grid.n_dim[1]) {

                int64_t idx = ii*grid.n_dim[1] + jj;

                if (grid.d_filter[idx] == 0) count0++;
                if (grid.d_filter[idx] == 1) count1++;
            }
        }
    }

    int val = count0 > count1 ? 0:1;

    int64_t idx = i*grid.n_dim[1] + j;
    res[idx] = val;
}

void
sparse_occ_grid_build_filter(sparse_occ_grid_t *grid, const float *wf, const float *wo, float b, int window_size) {

    int patch_size = (2*window_size) + 1;
    //printf("Patch size is: %d\n", patch_size);
    //printf("Dim is: %d\n", grid->n_dim[2] * patch_size);

    float *wf_d, *wo_d;
    cudaMalloc(&wf_d, sizeof(float) * grid->n_dim[2] * patch_size * patch_size);
    cudaMalloc(&wo_d, sizeof(float) * grid->n_dim[2] * patch_size * patch_size);

    cudaMemcpy(wf_d, wf, sizeof(float)*grid->n_dim[2] * patch_size * patch_size, cudaMemcpyHostToDevice);
    cudaMemcpy(wo_d, wo, sizeof(float)*grid->n_dim[2] * patch_size * patch_size, cudaMemcpyHostToDevice);

    // Make sure filter is malloced
    int n2 = grid->n_dim[0] * grid->n_dim[1];
    if (grid->filter == NULL) {
        grid->filter = (int*) malloc(sizeof(int)*n2);
        cudaMalloc(&grid->d_filter, sizeof(int)*n2);
    }

    dim3 threads;
    const int thread_dim = 16;
    threads.x = thread_dim;
    threads.y = thread_dim;

    dim3 blocks;
    blocks.x = grid->n_dim[0]/threads.x + 1;
    blocks.y = grid->n_dim[1]/threads.y + 1;

    int shared_memory_dim = 2*window_size + thread_dim;
    size_t shared_memory_sz = (shared_memory_dim*shared_memory_dim) * grid->n_dim[2] * sizeof(int8_t);

    int64_t tic_filter = utime_now();
    sparse_occ_grid_build_filter_device<<<blocks, threads, shared_memory_sz>>>(*grid, wf_d, wo_d, b, window_size);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_filter = utime_now();
    double t_filter = (toc_filter - tic_filter)/1e3;
    printf("Filtering occupancy grid took %5.3f ms with %dx%d blocks and %dx%d threads and %ld shared memory\n",
            t_filter, blocks.x, blocks.y, threads.x, threads.y, shared_memory_sz);

    //int64_t tic_smooth = utime_now();
    //int *tmp;
    //cudaMalloc(&tmp, sizeof(int)*n2);
    //sparse_occ_grid_smooth_filter_device<<<blocks, threads>>>(*grid, tmp);
    //cudaFree(grid->d_filter);
    //grid->d_filter = tmp;
    //cudaSafe(cudaDeviceSynchronize());
    //int64_t toc_smooth = utime_now();
    //double t_smooth = (toc_smooth - tic_smooth)/1e3;
    //printf("Smoothing occupancy grid took %5.3f ms\n", t_smooth);

    // Copy filter to host memory
    int64_t tic_cp = utime_now();
    cudaMemcpy(grid->filter, grid->d_filter, sizeof(int)*n2, cudaMemcpyDeviceToHost);
    cudaSafe(cudaDeviceSynchronize());
    int64_t toc_cp = utime_now();
    double t_cp = (toc_cp - tic_cp)/1e3;
    printf("Copying filter to host took %5.3fms\n", t_cp);

    // Cleanup
    cudaFree(wf_d);
    cudaFree(wo_d);

}

int
sparse_occ_grid_check_filter(sparse_occ_grid_t *grid, int i, int j) {
    if (grid->filter == NULL)
        return 0;

    return grid->filter[i*grid->n_dim[1] + j];
}

int
sparse_occ_grid_check_filter(sparse_occ_grid_t *grid, int64_t idx) {
    if (grid->filter == NULL)
        return 0;

    int i, j, k;
    sparse_occ_grid_idx_to_ijk(grid, idx, &i, &j, &k);

    return grid->filter[i*grid->n_dim[1] + j];
}

int
sparse_occ_grid_is_occuluded(sparse_occ_grid_t *grid, double x1, double y1, double z1, double x2, double y2, double z2) {

    // Setup
    double origin[3] = {x1, y1, z1};
    double hit[3] = {x2, y2, z2};

    double p0[3]; // origin
    double p1[3]; // end of ray
    double ad[3]; // abs of diff * 2
    int8_t sgn[3]; // which way am i going
    int64_t p[3]; // point i'm at (starts at origin)
    uint16_t dominant_dim = 0; // which dim am i stepping through

    for (int8_t i=0; i<3; ++i) {
        p0[i] = llrintf(origin[i] / grid->res) + grid->n_dim[i]/2;
        p1[i] = llrintf(hit[i] / grid->res) + grid->n_dim[i]/2;

        ad[i] = abs(p1[i] - p0[i]) * 2;

        sgn[i] = (p1[i]>p0[i]) - (p0[i]>p1[i]);

        p[i] = p0[i];

        if (ad[i] > ad[dominant_dim])
            dominant_dim = i;
    }

    double err[3];
    for (int i=0; i<3; i++)
        err[i] =  ad[i] - ad[dominant_dim]/2;

    // walk down ray
    int max_steps = grid->n_dim[0];
    if (max_steps < grid->n_dim[1]) max_steps = grid->n_dim[1];
    if (max_steps < grid->n_dim[2]) max_steps = grid->n_dim[2];

    for (int step = 0; step<max_steps; ++step) {

        // are we done?
        if ( (sgn[dominant_dim]>0) ?
                (p[dominant_dim]+0.0f>=p1[dominant_dim]) :
                (p[dominant_dim]+0.0f<=p1[dominant_dim])) {
            break;
        }

        // Bounds check
        if (p[0]<0 || p[0] >= grid->n_dim[0]
                || p[1] < 0 || p[1] >= grid->n_dim[1]
                || p[2] < 0 || p[2] >= grid->n_dim[2]) {
            break;
        }

        int64_t key = ((p[0]*grid->n_dim[1]) + p[1]) * grid->n_dim[2] + p[2];

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

        // Now check for occulsion
        if (sparse_occ_grid_lookup_helper(grid, key) > 0)
            return 1;
    }

    return 0;

}

__global__ void
clear_zeros(sparse_occ_grid_t sog) {

    const int bidx = blockIdx.x;
    const int bidy = blockIdx.y;

    const int tidx = threadIdx.x;

    int idx = (bidx*gridDim.y + bidy)*blockDim.x + tidx;

    // Check bounds
    if (idx >= sog.n_kv)
        return;

    // Mark val=0 for deletion
    is_lteps cond;
    if (cond(sog.d_vals[idx])) {
        sog.d_keys[idx] = -1L;
        sog.d_vals[idx] = 0;
    }
}

sparse_occ_grid_t*
sparse_occ_grid_merge(sparse_occ_grid_t *g1, sparse_occ_grid_t *g2) {

    int n_kv = g1->n_kv + g2->n_kv;

    // Make tmp memory
    int64_t *tmp_keys_1, *tmp_keys_2;
    float *tmp_vals_1, *tmp_vals_2;
    if (cudaSuccess != cudaMalloc(&tmp_keys_1, sizeof(int64_t)*n_kv)) printf("error\n");
    if (cudaSuccess != cudaMalloc(&tmp_vals_1, sizeof(float)*n_kv)  ) printf("error\n");
    if (cudaSuccess != cudaMalloc(&tmp_keys_2, sizeof(int64_t)*n_kv)) printf("error\n");
    if (cudaSuccess != cudaMalloc(&tmp_vals_2, sizeof(float)*n_kv)  ) printf("error\n");
    cudaSafe(cudaDeviceSynchronize());

    thrust::device_ptr<int64_t> g1_keys(g1->d_keys);
    thrust::device_ptr<int64_t> g2_keys(g2->d_keys);
    thrust::device_ptr<float> g1_vals(g1->d_vals);
    thrust::device_ptr<float> g2_vals(g2->d_vals);

    thrust::device_ptr<int64_t> d_tmp_keys_1(tmp_keys_1);
    thrust::device_ptr<float> d_tmp_vals_1(tmp_vals_1);

    cudaSafe(cudaDeviceSynchronize());
    auto merge_end = thrust::merge_by_key(g1_keys, g1_keys + g1->n_kv,
                         g2_keys, g2_keys + g2->n_kv,
                         g1_vals, g2_vals,
                         d_tmp_keys_1, d_tmp_vals_1);
    cudaSafe(cudaDeviceSynchronize());

    thrust::device_ptr<int64_t> d_tmp_keys_2(tmp_keys_2);
    thrust::device_ptr<float> d_tmp_vals_2(tmp_vals_2);

    auto reduce_end = thrust::reduce_by_key(d_tmp_keys_1,
                                            merge_end.first,
                                            d_tmp_vals_1,
                                            d_tmp_keys_2,
                                            d_tmp_vals_2);
    cudaSafe(cudaDeviceSynchronize());

    int reduce_n_kv = reduce_end.first - d_tmp_keys_2;

    sparse_occ_grid_t *sog = sparse_occ_grid_create(g1->res, g1->n_dim);
    sog->n_kv = reduce_n_kv;

    cudaMalloc(&sog->d_keys, sizeof(int64_t)*sog->n_kv);
    cudaMalloc(&sog->d_vals, sizeof(float)*sog->n_kv);
    cudaMemcpy(sog->d_keys, tmp_keys_2, sizeof(int64_t)*sog->n_kv, cudaMemcpyDeviceToDevice);
    cudaMemcpy(sog->d_vals, tmp_vals_2, sizeof(float)*sog->n_kv, cudaMemcpyDeviceToDevice);
    cudaSafe(cudaDeviceSynchronize());

    int64_t tic_verifysorted = utime_now();
    bool sorted = thrust::is_sorted(d_tmp_keys_2, d_tmp_keys_2 + sog->n_kv);
    int64_t toc_verifysorted = utime_now();
    double t_verifysorted = (toc_verifysorted - tic_verifysorted)/1e3;
    if (!sorted) printf("\tTook %5.3f ms to verify keys are %s\n", t_verifysorted, sorted ? "sorted":"not sorted");

    // Copy to host memory
    sog->keys = (int64_t*) malloc(sizeof(int64_t)*sog->n_kv);
    sog->vals = (float*) malloc(sizeof(float)*sog->n_kv);
    cudaMemcpy(sog->keys, sog->d_keys, sizeof(int64_t) * sog->n_kv, cudaMemcpyDeviceToHost);
    cudaMemcpy(sog->vals, sog->d_vals, sizeof(float) * sog->n_kv, cudaMemcpyDeviceToHost);
    cudaSafe(cudaDeviceSynchronize());

    cudaFree(tmp_keys_1);
    cudaFree(tmp_keys_2);
    cudaFree(tmp_vals_1);
    cudaFree(tmp_vals_2);
    cudaSafe(cudaDeviceSynchronize());

    return sog;
}

void
sparse_occ_grid_clean(sparse_occ_grid_t *grid) {

    // Make sure keys and vals are on device
    if (grid->d_keys == NULL) {
        cudaMalloc(&grid->d_keys, sizeof(int64_t)*grid->n_kv);
        cudaMemcpy(grid->d_keys, grid->keys, sizeof(int64_t)*grid->n_kv, cudaMemcpyHostToDevice);
    }

    if (grid->d_vals == NULL) {
        cudaMalloc(&grid->d_vals, sizeof(float)*grid->n_kv);
        cudaMemcpy(grid->d_vals, grid->vals, sizeof(float)*grid->n_kv, cudaMemcpyHostToDevice);
    }


    // Remove unnecesary items
    int threads = 1024;

    dim3 blocks;
    blocks.x = 1024;
    blocks.y = grid->n_kv / (threads*blocks.x) + 1;

    printf("Clearing with %dx%d blocks and %d threads\n", blocks.x, blocks.y, threads);
    clear_zeros<<<blocks, threads>>>(*grid);
    cudaSafe(cudaDeviceSynchronize());

    thrust::device_ptr<int64_t> d_keys(grid->d_keys);
    thrust::device_ptr<float> d_vals(grid->d_vals);

    // Count
    int count = thrust::count_if(d_vals, d_vals + grid->n_kv, is_lteps());
    printf("Have %d elements to remove\n", count);

    auto new_keys_end = thrust::remove_if(d_keys, d_keys + grid->n_kv, is_negative());
    auto new_vals_end = thrust::remove_if(d_vals, d_vals + grid->n_kv, is_lteps());
    printf("Cleaned SOG from %d to %d vals\n", grid->n_kv, new_keys_end - d_keys);
    printf("Cleaned SOG from %d to %d keys\n", grid->n_kv, new_vals_end - d_vals);
    grid->n_kv = new_keys_end - d_keys;

    cudaMemcpy(grid->keys, grid->d_keys, sizeof(int64_t) * grid->n_kv, cudaMemcpyDeviceToHost);
    cudaMemcpy(grid->vals, grid->d_vals, sizeof(float) * grid->n_kv, cudaMemcpyDeviceToHost);
    cudaSafe(cudaDeviceSynchronize());

}

void
sparse_occ_grid_save(sparse_occ_grid_t *grid, const char* fn) {

    FILE *fp = fopen(fn, "w");

    fwrite(grid->n_dim, sizeof(grid->n_dim[0]), 3, fp);
    fwrite(&grid->res, sizeof(grid->res), 1, fp);

    fwrite(grid->pose, sizeof(grid->pose[0]), 6, fp);
    fwrite(&grid->utime, sizeof(grid->utime), 1, fp);

    fwrite(&grid->n_kv, sizeof(grid->n_kv), 1, fp);

    fwrite(grid->keys, sizeof(grid->keys[0]), grid->n_kv, fp);
    fwrite(grid->vals, sizeof(grid->vals[0]), grid->n_kv, fp);

    fclose(fp);
}

sparse_occ_grid_t*
sparse_occ_grid_load(const char* fn) {

    FILE *fp = fopen(fn, "r");

    sparse_occ_grid_t *grid = (sparse_occ_grid_t*) calloc(sizeof(sparse_occ_grid_t), 1);

    fread(grid->n_dim, sizeof(grid->n_dim[0]), 3, fp);
    fread(&grid->res, sizeof(grid->res), 1, fp);

    fread(grid->pose, sizeof(grid->pose[0]), 6, fp);
    fread(&grid->utime, sizeof(grid->utime), 1, fp);

    fread(&grid->n_kv, sizeof(grid->n_kv), 1, fp);

    grid->keys = (int64_t*) malloc(sizeof(int64_t)*grid->n_kv);
    grid->vals = (float*) malloc(sizeof(float)*grid->n_kv);

    fread(grid->keys, sizeof(grid->keys[0]), grid->n_kv, fp);
    fread(grid->vals, sizeof(grid->vals[0]), grid->n_kv, fp);

    fclose(fp);

    return grid;
}

sparse_occ_grid_t*
sparse_occ_grid_get_subset(sparse_occ_grid_t *grid, double x_rel[6], int64_t n_dim[3]) {

    sparse_occ_grid_t *res = sparse_occ_grid_create(grid->res, n_dim);

    res->keys = (int64_t*) malloc(sizeof(int64_t)*n_dim[0]*n_dim[1]*n_dim[2]);
    res->vals = (float*) malloc(sizeof(float)*n_dim[0]*n_dim[1]*n_dim[2]);

    // Populate
    double x, y, z;
    int i_grid, j_grid, k_grid;

    is_lteps cond;

    double H[4*4];
    ssc_homo4x4(H, x_rel);

    Eigen::Matrix<double, 4, 4> H_eg;
    for (int i=0; i<4; i++) {
        for (int j=0; j<4; j++) {
            H_eg(i, j) = H[i*4 + j];
        }
    }

    for (int i=0; i<n_dim[0]; i++) {
        for (int j=0; j<n_dim[1]; j++) {
            for (int k=0; k<n_dim[2]; k++) {
                sparse_occ_grid_get_xyz(res, i, j, k, &x, &y, &z);

                Eigen::Matrix<double, 4, 1> x_res;
                x_res(0, 0) = x;
                x_res(1, 0) = y;
                x_res(2, 0) = z;
                x_res(3, 0) = 1;

                Eigen::Matrix<double, 4, 1> x_grid = H_eg * x_res;

                sparse_occ_grid_get_ijk(grid, x_grid(0, 0), x_grid(1, 0), x_grid(2, 0),
                        &i_grid, &j_grid, &k_grid);

                float v = sparse_occ_grid_lookup(grid, i_grid, j_grid, k_grid);

                if (!cond(v)) {
                    int64_t idx;
                    sparse_occ_grid_ijk_to_idx(res, i, j, k, &idx);

                    res->keys[res->n_kv] = idx;
                    res->vals[res->n_kv] = v;

                    res->n_kv++;
                }
            }
        }
    }

    if (res->n_kv>0) {
        res->keys = (int64_t*) realloc(res->keys, sizeof(int64_t)*res->n_kv);
        res->vals = (float*) realloc(res->vals, sizeof(float)*res->n_kv);
    } else {
        free(res->keys);
        free(res->vals);
        res->keys = NULL;
        res->vals = NULL;
    }

    return res;

}
