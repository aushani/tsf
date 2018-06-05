#ifndef _KITTI_VELODYNE_HPP_
#define _KITTI_VELODYNE_HPP_

// C, C++
#include <string>
#include <vector>
#include <tuple>

// perls
#include "thirdparty/perls-common/units.h"

// dascar
#include "library/sensors/velodyne.h"

// viewer
#include "library/viewer/nodes/points.hpp"

namespace osg
{

class KittiVelodyne : public osg::Group
{
  public:

    KittiVelodyne();
    ~KittiVelodyne();

    void set_point_cloud(const velodyne_returns_t *vr);

  private:

    // world to sensor transforms for each velodyne
    double _x_ws[4][6]; // computed using latest pose of car

    // Currently rendered point cloud
    osg::ref_ptr<Points> _p_active;

};

} // namespace osg

#endif // _KITTI_VELODYNE_HPP_
