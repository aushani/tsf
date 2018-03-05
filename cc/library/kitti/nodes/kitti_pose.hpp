#ifndef _KITTI_POSE_HPP_
#define _KITTI_POSE_HPP_

// C, C++
#include <string>
#include <vector>

// OSG
#include <osg/MatrixTransform>

#include "library/kitti/pose.hpp"

namespace osg
{

class KittiPose : public osg::Group
{
  public:

    KittiPose();
    ~KittiPose();

    void set_poses(std::vector<Pose> poses);

    void set_frame(int frame);

  private:

    std::vector<Pose> _poses;

};

} // namespace osg

#endif
