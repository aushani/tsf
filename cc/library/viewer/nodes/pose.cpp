/*
 * viewer/lib/nodes/pose.cpp
 */

// C, C++
#include <iostream>

// local headers
#include "library/viewer/util/utils.hpp"

#include "pose.hpp"

namespace osg
{

Pose::Pose(lcm::LCM* lcm) :
        osg::MatrixTransform()
{
    lcm->subscribe("POSE", &Pose::handle_pose, this);
    setMatrix(osg::Matrixd::identity());
}

void Pose::handle_pose(const lcm::ReceiveBuffer* rbuf,
                       const std::string& chan,
                       const pose_t* msg)
{
    osg::Matrixd M;
    ViewerUtils::xyz_quat_to_matrixd(M, msg->pos, msg->orientation);
    setMatrix(M);
}

} // namespace osg
