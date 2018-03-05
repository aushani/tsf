#include <iostream>

#include <boost/assert.hpp>

#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/remove.h>

#include "occ_grid_builder.h"

struct DeviceData {
  DeviceData(float resolution, float max_range, int max_observations);
  DeviceData(const DeviceData &dd);
  ~DeviceData();

  int num_observations = 0;
  int max_voxel_visits_per_ray = 0;

  float resolution = 0.0f;

  float *hit_x = nullptr;
  float *hit_y = nullptr;
  float *hit_z = nullptr;

  float *origin_x = nullptr;
  float *origin_y = nullptr;
  float *origin_z = nullptr;

  Location *locations = nullptr;
  float *log_likelihood_updates = nullptr;

  Location *locations_reduced = nullptr;
  float *log_likelihood_updates_reduced = nullptr;

  bool own_gpu_memory = false;
};

DeviceData::DeviceData(float resolution, float max_range, int max_observations)
    : resolution(resolution), max_voxel_visits_per_ray(max_range / resolution) {
  // Allocate memory on the device.
  cudaError_t err = cudaMalloc(&hit_x, sizeof(float) * max_observations);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&hit_y, sizeof(float) * max_observations);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&hit_z, sizeof(float) * max_observations);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&origin_x, sizeof(float) * max_observations);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&origin_y, sizeof(float) * max_observations);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&origin_z, sizeof(float) * max_observations);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&locations, sizeof(Location) * max_observations * max_voxel_visits_per_ray);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&log_likelihood_updates, sizeof(float) * max_observations * max_voxel_visits_per_ray);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&locations_reduced, sizeof(Location) * max_observations * max_voxel_visits_per_ray);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }

  err = cudaMalloc(&log_likelihood_updates_reduced, sizeof(float) * max_observations * max_voxel_visits_per_ray);
  if (err != cudaSuccess) {
    std::cerr << "[OccGridBuilder] Error allocating memory" << std::endl;
    ::abort();
  }
}

DeviceData::DeviceData(const DeviceData &dd)
    : num_observations(dd.num_observations),
      max_voxel_visits_per_ray(dd.max_voxel_visits_per_ray),
      resolution(dd.resolution),
      hit_x(dd.hit_x),
      hit_y(dd.hit_y),
      hit_z(dd.hit_z),
      origin_x(dd.origin_x),
      origin_y(dd.origin_y),
      origin_z(dd.origin_z),
      locations(dd.locations),
      log_likelihood_updates(dd.log_likelihood_updates),
      locations_reduced(dd.locations_reduced),
      log_likelihood_updates_reduced(dd.log_likelihood_updates_reduced),
      own_gpu_memory(false) {}

DeviceData::~DeviceData() {
  if (own_gpu_memory) {
    cudaFree(hit_x);
    cudaFree(hit_y);
    cudaFree(hit_z);
    cudaFree(origin_x);
    cudaFree(origin_y);
    cudaFree(origin_z);
    cudaFree(locations);
    cudaFree(log_likelihood_updates);
    cudaFree(locations_reduced);
    cudaFree(log_likelihood_updates_reduced);
  }
}

OccGridBuilder::OccGridBuilder(int max_observations, float resolution, float max_range)
    : max_observations_(max_observations),
      resolution_(resolution),
      device_data_(new DeviceData(resolution, max_range, max_observations)) {
  device_data_->own_gpu_memory = true;
}

OccGridBuilder::~OccGridBuilder() {}

__global__ void RayTracingKernel(DeviceData data) {
  // Constants
  const float kLogLikelihoodFree = -0.1;
  const float kLogLikelihoodOccupied = 1.0;
  const float kLogLikelihoodUnknown = 0.0;

  // Figure out which hit this thread is processing
  const int bidx = blockIdx.x;
  const int tidx = threadIdx.x;
  const int threads = blockDim.x;

  const int hit_idx = tidx + bidx * threads;
  if (hit_idx >= data.num_observations) {
    return;
  }

  float hit[3] = {data.hit_x[hit_idx], data.hit_y[hit_idx], data.hit_z[hit_idx]};
  float origin[3] = {data.origin_x[hit_idx], data.origin_y[hit_idx], data.origin_z[hit_idx]};

  // The following is an implementation of Bresenham's line algorithm to sweep out the ray from the origin of the ray to
  // the hit point.
  // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  float ad[3] = {0.0f, 0.0f, 0.0f};  // absolute value of diff * 2
  int sgn[3] = {0, 0, 0};            // which way am i going
  int cur_loc[3] = {0, 0, 0};        // what location am i currently at (starts at origin of ray)
  int end_loc[3] = {0, 0, 0};        // what location am i ending at (ends at the hit)
  int dominant_dim = 0;              // which dim am i stepping through

  for (int i = 0; i < 3; ++i) {
    cur_loc[i] = origin[i] / data.resolution;
    end_loc[i] = hit[i] / data.resolution;

    ad[i] = abs(end_loc[i] - cur_loc[i]) * 2;

    sgn[i] = (end_loc[i] > cur_loc[i]) - (cur_loc[i] > end_loc[i]);

    if (ad[i] > ad[dominant_dim]) {
      dominant_dim = i;
    }
  }

  float err[3];
  for (int i = 0; i < 3; i++) {
    err[i] = ad[i] - ad[dominant_dim] / 2;
  }

  // walk down ray
  size_t mem_step_size = data.num_observations;
  size_t mem_idx = hit_idx;
  bool valid = true;
  Location loc;
  for (int step = 0; step < (data.max_voxel_visits_per_ray - 1); ++step) {
    loc.i = cur_loc[0];
    loc.j = cur_loc[1];
    loc.k = cur_loc[2];
    float llUpdate = kLogLikelihoodUnknown;

    // Are we done? Have we reached the hit point?
    // Don't quit the loop just yet. We need to 0 out the rest of log likelihood updates.
    if ((sgn[dominant_dim] > 0) ? (cur_loc[dominant_dim] >= end_loc[dominant_dim])
                                : (cur_loc[dominant_dim] <= end_loc[dominant_dim])) {
      valid = false;
    }

    if (valid) {
      // step forwards
      for (int dim = 0; dim < 3; ++dim) {
        if (dim != dominant_dim) {
          if (err[dim] >= 0) {
            cur_loc[dim] += sgn[dim];
            err[dim] -= ad[dominant_dim];
          }
        }
      }

      for (int dim = 0; dim < 3; ++dim) {
        if (dim == dominant_dim) {
          cur_loc[dim] += sgn[dim];
        } else {
          err[dim] += ad[dim];
        }
      }

      llUpdate = kLogLikelihoodFree;
    }

    // Now write out key value pair
    data.locations[mem_idx] = loc;
    data.log_likelihood_updates[mem_idx] = llUpdate;

    mem_idx += mem_step_size;
  }

  loc.i = end_loc[0];
  loc.j = end_loc[1];
  loc.k = end_loc[2];

  // Now write out key value pair
  data.locations[mem_idx] = loc;
  data.log_likelihood_updates[mem_idx] = kLogLikelihoodOccupied;
}

struct NoUpdate {
  __host__ __device__ bool operator()(const float x) const { return fabs(x) < 1e-6; }
};

struct LocationComparator {
  __host__ __device__ bool operator()(const thrust::tuple<Location, float> &lhs,
                                      const thrust::tuple<Location, float> &rhs) const {
    Location loc1 = lhs.head;
    Location loc2 = rhs.head;
    if (loc1.i != loc2.i) {
      return loc1.i < loc2.i;
    } else if (loc1.j != loc2.j) {
      return loc1.j < loc2.j;
    } else {
      return loc1.k < loc2.k;
    }
  }
};

struct LocationEquality {
  __host__ __device__ bool operator()(const Location &lhs, const Location &rhs) const {
    if (lhs.i != rhs.i) {
      return false;
    } else if (lhs.j != rhs.j) {
      return false;
    } else {
      return lhs.k == rhs.k;
    }
  }
};

OccGrid OccGridBuilder::GenerateOccGrid(const float *hits_x, const float *hits_y, const float *hits_z,
        const float *origin_x, const float *origin_y, const float *origin_z, int n_hits) {
  // Check for empty data
  if (n_hits == 0) {
    std::vector<Location> location_vector(0);
    std::vector<float> ll_vector(0);
    return OccGrid(location_vector, ll_vector, resolution_);
  }

  // First, we need to send the data to the GPU device
  device_data_->num_observations = n_hits;
  size_t sz_copy = sizeof(float) * n_hits;
  cudaMemcpy(device_data_->hit_x, hits_x, sz_copy, cudaMemcpyHostToDevice);
  cudaMemcpy(device_data_->hit_y, hits_y, sz_copy, cudaMemcpyHostToDevice);
  cudaMemcpy(device_data_->hit_z, hits_z, sz_copy, cudaMemcpyHostToDevice);
  cudaMemcpy(device_data_->origin_x, origin_x, sz_copy, cudaMemcpyHostToDevice);
  cudaMemcpy(device_data_->origin_y, origin_y, sz_copy, cudaMemcpyHostToDevice);
  cudaMemcpy(device_data_->origin_z, origin_z, sz_copy, cudaMemcpyHostToDevice);

  // Now run ray tracing on the GPU device
  int blocks = device_data_->num_observations / kThreadsPerBlock_ + 1;
  RayTracingKernel<<<blocks, kThreadsPerBlock_>>>(*device_data_);
  cudaError_t err = cudaDeviceSynchronize();

  // Accumulate all the updates
  size_t num_updates = device_data_->num_observations * device_data_->max_voxel_visits_per_ray;

  // First prune unnecessary updates
  thrust::device_ptr<Location> dp_locations(device_data_->locations);
  thrust::device_ptr<float> dp_updates(device_data_->log_likelihood_updates);

  thrust::device_ptr<Location> dp_locations_end =
      thrust::remove_if(dp_locations, dp_locations + num_updates, dp_updates, NoUpdate());
  thrust::device_ptr<float> dp_updates_end = thrust::remove_if(dp_updates, dp_updates + num_updates, NoUpdate());
  num_updates = dp_locations_end - dp_locations;

  // Now reduce updates to resulting log likelihoods
  thrust::sort_by_key(dp_locations, dp_locations + num_updates, dp_updates, LocationComparator());

  thrust::device_ptr<Location> dp_locs_reduced(device_data_->locations_reduced);
  thrust::device_ptr<float> dp_ll_reduced(device_data_->log_likelihood_updates_reduced);

  thrust::pair<thrust::device_ptr<Location>, thrust::device_ptr<float> > new_ends = thrust::reduce_by_key(
      dp_locations, dp_locations + num_updates, dp_updates, dp_locs_reduced, dp_ll_reduced, LocationEquality());
  num_updates = new_ends.first - dp_locs_reduced;

  // Copy result from GPU device to host
  std::vector<Location> location_vector(num_updates);
  std::vector<float> ll_vector(num_updates);
  cudaMemcpy(location_vector.data(), dp_locs_reduced.get(), sizeof(Location) * num_updates, cudaMemcpyDeviceToHost);
  cudaMemcpy(ll_vector.data(), dp_ll_reduced.get(), sizeof(float) * num_updates, cudaMemcpyDeviceToHost);

  OccGrid res(location_vector, ll_vector, resolution_);
  return res;
}
