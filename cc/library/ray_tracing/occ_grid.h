#pragma once
#ifndef _OCC_GRID_H_
#define _OCC_GRID_H_

#include <vector>

struct Location {
  int i = 0;
  int j = 0;
  int k = 0;

  bool operator<(const Location &rhs) const {
    if (i != rhs.i) {
      return i < rhs.i;
    } else if (j != rhs.j) {
      return j < rhs.j;
    } else {
      return k < rhs.k;
    }
  }

  bool operator==(const Location &rhs) const {
    return i==rhs.i && j==rhs.j && k==rhs.k;
  }
};

class OccGrid {
 public:
  OccGrid(const std::vector<Location> &locations, const std::vector<float> &log_likelihoods, const float resolution);
  OccGrid(const OccGrid &og);
  ~OccGrid();

  float GetLogLikelihood(Location loc);
  float GetLogLikelihood(double x, double y, double z);

  float resolution() const;

  const std::vector<Location>& GetLocations() const;
  const std::vector<float>& GetLogLikelihoods() const;

  OccGrid MergeWith(OccGrid& og);

  void Save(const char *filename);
  void SaveDense(const char *filename, int dim[3]);
  static OccGrid Load(const char* filename);
  static OccGrid LoadDense(float resolution, const char* filename);

  OccGrid Transform(double x_rel[6], double max_range=-1.0);
  OccGrid FilterByRange(double max_range);

 private:
  // parallel containers
  std::vector<Location> locations_;
  std::vector<float> log_likelihoods_;

  float resolution_;
};

#endif
