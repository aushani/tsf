#ifndef _FLOW_POSE_SOLVER_H_
#define _FLOW_POSE_SOLVER_H_

#include <vector>

#include "library/sensors/velodyne.h"

#include "library/kitti/pose.hpp"

#include "library/scan_matching/scan_match_3d.h"

class PoseSolver
{
  public:

    PoseSolver(std::vector<velodyne_returns_t*> vrs, std::vector<Pose> poses);
    ~PoseSolver();

    std::vector<Pose> solve();

  private:

    struct sm_data_t {
        int i;
    };

    std::vector<velodyne_returns_t*> _vrs;
    std::vector<Pose> _poses;

    ScanMatch3d _sm;
};

#endif
