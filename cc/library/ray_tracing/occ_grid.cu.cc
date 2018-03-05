#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <assert.h>

#include "occ_grid.h"
#include "util.h"

#include "perls-math/ssc.h"

#include <thrust/reduce.h>
#include <thrust/merge.h>
#include <thrust/sort.h>
#include <Eigen/Core>
#include <Eigen/LU>

OccGrid::OccGrid(const std::vector<Location> &locations, const std::vector<float> &log_likelihoods, float resolution) :
  locations_(locations), log_likelihoods_(log_likelihoods), resolution_(resolution) {
  assert(locations.size() == log_likelihoods_.size());
}

OccGrid::OccGrid(const OccGrid &og) :
  locations_(og.locations_), log_likelihoods_(og.log_likelihoods_), resolution_(og.resolution_) {
}

OccGrid::~OccGrid() {

}

float OccGrid::GetLogLikelihood(Location loc) {
  std::vector<Location>::iterator it = std::lower_bound(locations_.begin(), locations_.end(), loc);
  if (it != locations_.end() && (*it) == loc) {
    size_t pos = it - locations_.begin();
    return log_likelihoods_[pos];
  }

  // unknown
  return 0.0f;
}

float OccGrid::GetLogLikelihood(double x, double y, double z) {
  Location loc;
  loc.i = x / resolution_;
  loc.j = y / resolution_;
  loc.k = z / resolution_;

  return GetLogLikelihood(loc);
}

float OccGrid::resolution() const { return resolution_; }

const std::vector<Location>& OccGrid::GetLocations() const { return locations_; }

const std::vector<float>& OccGrid::GetLogLikelihoods() const { return log_likelihoods_; }

OccGrid OccGrid::MergeWith(OccGrid& og) {
  size_t max_sz = locations_.size() + og.locations_.size();

  Location loc;
  std::vector<Location> locs_new_1(max_sz, Location()), locs_new_2(max_sz, Location());
  std::vector<float> ll_new_1(max_sz, 0.0f), ll_new_2(max_sz, 0.0f);

  thrust::merge_by_key(locations_.begin(), locations_.end(),
                    og.locations_.begin(), og.locations_.end(),
                    log_likelihoods_.begin(), og.log_likelihoods_.begin(),
                    locs_new_1.begin(), ll_new_1.begin());
  cudaDeviceSynchronize();

  auto reduce_end = thrust::reduce_by_key(locs_new_1.begin(), locs_new_1.end(),
                                          ll_new_1.begin(),
                                          locs_new_2.begin(), ll_new_2.begin());
  cudaDeviceSynchronize();

  size_t sz = reduce_end.first - locs_new_2.begin();
  locs_new_2.resize(sz);
  ll_new_2.resize(sz);
  return OccGrid(locs_new_2, ll_new_2, resolution_);
  /*
  std::vector<Location> locations_new;
  std::vector<float> log_likelihoods_new;

  size_t idx_this = 0;
  size_t idx_og = 0;

  while (idx_this < locations_.size() || idx_og < og.locations_.size()) {

    if (idx_this >= locations_.size()) {
      locations_new.insert(locations_new.end(), og.locations_.begin() + idx_og, og.locations_.end());
      log_likelihoods_new.insert(log_likelihoods_new.end(), og.log_likelihoods_.begin() + idx_og, og.log_likelihoods_.end());
      break;
    } else if (idx_og >= og.locations_.size()) {
      locations_new.insert(locations_new.end(), locations_.begin() + idx_this, locations_.end());
      log_likelihoods_new.insert(log_likelihoods_new.end(), log_likelihoods_.begin() + idx_this, log_likelihoods_.end());
      break;
    }

    Location loc_this = locations_[idx_this];
    Location loc_og = og.locations_[idx_og];

    if (loc_this == loc_og) {
      float new_val = log_likelihoods_[idx_this] + og.log_likelihoods_[idx_og];

      if (std::abs(new_val) > 0) {
        locations_new.push_back(loc_this);
        log_likelihoods_new.push_back(new_val);
      }

      idx_this++;
      idx_og++;
    } else if (loc_this < loc_og) {
      locations_new.push_back(loc_this);
      log_likelihoods_new.push_back(log_likelihoods_[idx_this]);
      idx_this++;
    } else {  // if (loc_this > loc_og)
      locations_new.push_back(loc_og);
      log_likelihoods_new.push_back(og.log_likelihoods_[idx_og]);
      idx_og++;
    }
  }

  return OccGrid(locations_new, log_likelihoods_new, resolution_);
  */
}

void OccGrid::Save(const char *filename) {
  FILE *fp = fopen(filename, "w");

  fwrite(&resolution_, sizeof(float), 1, fp);

  size_t sz = locations_.size();
  fwrite(&sz, sizeof(size_t), 1, fp);

  // do this out because of memory packing
  for (int i=0; i<sz; i++) {
    Location loc = locations_[i];
    fwrite(&loc.i, sizeof(int), 1, fp);
    fwrite(&loc.j, sizeof(int), 1, fp);
    fwrite(&loc.k, sizeof(int), 1, fp);
  }

  fwrite(log_likelihoods_.data(), sizeof(float), sz, fp);

  fclose(fp);
}

void OccGrid::SaveDense(const char *filename, int dim[3]) {
  FILE *fp = fopen(filename, "w");

  for (int i=0; i<dim[0]; i++) {
    for (int j=0; j<dim[1]; j++) {
      for (int k=0; k<dim[2]; k++) {

        Location loc;
        loc.i = i - dim[0]/2;
        loc.j = j - dim[1]/2;
        loc.k = k - dim[2]/2;

        float ll = GetLogLikelihood(loc);

        // Convert to probability
        float p = 1.0f - 1.0f / (1+expf(ll));

        // scale from -1 to 1
        float towrite = 2*p - 1;

        fwrite(&towrite, sizeof(float), 1, fp);
      }
    }
  }

  fclose(fp);
}

OccGrid OccGrid::Load(const char *filename) {
  float resolution = 0.0;
  std::vector<Location> locations;
  std::vector<float> log_likelihoods;

  FILE *fp = fopen(filename, "r");

  fread(&resolution, sizeof(float), 1, fp);

  size_t sz = 0;
  fread(&sz, sizeof(size_t), 1, fp);

  for (size_t i=0; i<sz; i++) {
    Location loc;
    fread(&loc.i, sizeof(int), 1, fp);
    fread(&loc.j, sizeof(int), 1, fp);
    fread(&loc.k, sizeof(int), 1, fp);

    locations.push_back(loc);
  }

  for (size_t i=0; i<sz; i++) {
    float ll;
    fread(&ll, sizeof(float), 1, fp);
    log_likelihoods.push_back(ll);
  }

  fclose(fp);

  return OccGrid(locations, log_likelihoods, resolution);
}

OccGrid OccGrid::LoadDense(float resolution, const char *filename) {
  std::vector<Location> locations;
  std::vector<float> log_likelihoods;

  FILE *fp = fopen(filename, "r");

  // Find dimensions
  fseek(fp, 0L, SEEK_END);
  size_t sz = ftell(fp);
  rewind(fp);

  int n = pow(sz/sizeof(float), 1.0/3.0) + 0.5;
  printf("n is %d\n", n);
  printf("n is %5.3f\n", pow(sz/sizeof(float), 1.0/3.0));
  int dim[3] = {n, n, n};

  for (int i=0; i<dim[0]; i++) {
    for (int j=0; j<dim[1]; j++) {
      for (int k=0; k<dim[2]; k++) {

        Location loc;
        loc.i = i - dim[0]/2;
        loc.j = j - dim[1]/2;
        loc.k = k - dim[2]/2;

        float p;
        fread(&p, sizeof(float), 1, fp);

        // Convert to log odds
        float ll = logf(p / (1 - p));

        if (std::abs(ll)>0.1) {
          locations.push_back(loc);
          log_likelihoods.push_back(ll);
        }
      }
    }
  }

  fclose(fp);

  return OccGrid(locations, log_likelihoods, resolution);
}

//struct TransformOp {
//  __host__ __device__ Location operator()(const Location &loc) {
//    return loc;
//  }
//}:

OccGrid
OccGrid::Transform(double x_rel[6], double max_range) {
  std::vector<Location> locs;
  std::vector<float> ll;

  double H[4*4];
  ssc_homo4x4(H, x_rel);

  Eigen::Matrix<double, 4, 4> H_eg;
  for (int i=0; i<4; i++) {
      for (int j=0; j<4; j++) {
          H_eg(i, j) = H[i*4 + j];
      }
  }

  Eigen::Matrix<double, 4, 1> x_origin;
  x_origin(0, 0) = 0.0;
  x_origin(1, 0) = 0.0;
  x_origin(2, 0) = 0.0;
  x_origin(3, 0) = 1.0;

  Eigen::Matrix<double, 4, 1> x_origin_t = H_eg * x_origin;

  Eigen::Matrix<double, 4, 4> H_t = H_eg.inverse().eval();

  for (size_t i = 0; i<locations_.size(); i++) {
    Location loc = locations_[i];
    float val = log_likelihoods_[i];

    float x = loc.i * resolution_;
    float y = loc.j * resolution_;
    float z = loc.k * resolution_;

    // Do a quick check before we do unneccesary work
    if (max_range > 0) {
      // with fudge factor
      if (std::abs(x - x_origin_t(0, 0)) > 2*max_range) {
        continue;
      }
      if (std::abs(y - x_origin_t(1, 0)) > 2*max_range) {
        continue;
      }
      if (std::abs(z - x_origin_t(2, 0)) > 2*max_range) {
        continue;
      }
    }


    Eigen::Matrix<double, 4, 1> x_grid;
    x_grid(0, 0) = x;
    x_grid(1, 0) = y;
    x_grid(2, 0) = z;
    x_grid(3, 0) = 1;

    Eigen::Matrix<double, 4, 1> x_res = H_t * x_grid;

    Location loc_res;
    loc_res.i = x_res(0, 0) / resolution_;
    loc_res.j = x_res(1, 0) / resolution_;
    loc_res.k = x_res(2, 0) / resolution_;

    if (max_range > 0) {
      if (std::abs(x_res(0, 0)) > max_range) {
        continue;
      }
      if (std::abs(x_res(1, 0)) > max_range) {
        continue;
      }
      if (std::abs(x_res(2, 0)) > max_range) {
        continue;
      }
    }

    locs.push_back(loc_res);
    ll.push_back(val);
  }

  thrust::sort_by_key(locs.begin(), locs.end(), ll.begin());
  return OccGrid(locs, ll, resolution_);
}

OccGrid
OccGrid::FilterByRange(double max_range) {
  std::vector<Location> locs;
  std::vector<float> ll;

  for (size_t i=0; i<locations_.size(); i++) {
    Location loc = locations_[i];

    if (std::abs(loc.i * resolution_) > max_range)
      continue;
    if (std::abs(loc.j * resolution_) > max_range)
      continue;
    if (std::abs(loc.k * resolution_) > max_range)
      continue;

    locs.push_back(loc);
    ll.push_back(log_likelihoods_[i]);
  }

  return OccGrid(locs, ll, resolution_);
}
