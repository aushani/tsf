#ifndef _VIEWER_LIB_NODES_POSE_HPP_
#define _VIEWER_LIB_NODES_POSE_HPP_

// C, C++
#include <string>

// OSG
#include <osg/MatrixTransform>

// LCM
#include "library/sensors/pose_t.hpp"
#include <lcm/lcm-cpp.hpp>

// local headers

namespace osg
{

class Pose : public osg::MatrixTransform
{
  public:

    Pose(lcm::LCM* lcm);

    void handle_pose(const lcm::ReceiveBuffer* rbuf,
                     const std::string& chan,
                     const pose_t* msg);

  protected:
    virtual ~Pose() = default;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_POSE_HPP_
