// C, C++
#include <iostream>

#include <osg/ShapeDrawable>

#include "perls-math/ssc.h"
#include "perls-math/so3.h"

// local headers
#include "library/viewer/util/utils.hpp"
#include "library/viewer/nodes/composite_shape_group.hpp"

#include "kitti_pose.hpp"

namespace osg
{

KittiPose::KittiPose() :
    osg::Group()
{
}

KittiPose::~KittiPose()
{
}

void
KittiPose::set_poses(std::vector<Pose> poses)
{
    _poses = poses;
}

void
KittiPose::set_frame(int frame)
{
    //printf("Set frame to %d\n", frame);

    // If we're not ready don't try to render
    if (_poses.size() <= frame)
        return;

    // Render all other poses in this frame's pose
    Pose p_frame = _poses[frame];

    //printf("Pose: %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n", p_frame.x, p_frame.y, p_frame.z, p_frame.r, p_frame.p, p_frame.h);

    // for numerical accuracy
    double x0 = p_frame.x;
    double y0 = p_frame.y;
    double z0 = p_frame.z;

    double pose_xyzrph[6] = {p_frame.x-x0, p_frame.y-y0, p_frame.z-z0, p_frame.r, p_frame.p, p_frame.h};

    osg::ref_ptr<osg::CompositeShapeGroup> csg = new osg::CompositeShapeGroup();

    for (int i=0; i<_poses.size(); i++) {

        Pose p_i = _poses[i];

        // Transform that frame's pose into this frame
        double pi_xyzrph[6] = {p_i.x-x0, p_i.y-y0, p_i.z-z0, p_i.r, p_i.p, p_i.h};

        double rel_pose[6];
        ssc_tail2tail(rel_pose, NULL, pose_xyzrph, pi_xyzrph);

        double quat[4];
        so3_rph2quat(&rel_pose[3], quat);

        //printf("act pose %d: %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n", i, pi_xyzrph[0], pi_xyzrph[1], pi_xyzrph[2], pi_xyzrph[3], pi_xyzrph[4], pi_xyzrph[5]);
        //printf("rel pose %d: %5.3f %5.3f %5.3f %5.3f %5.3f %5.3f\n", i, rel_pose[0], rel_pose[1], rel_pose[2], rel_pose[3], rel_pose[4], rel_pose[5]);

        osg::ref_ptr<osg::Box> box = new osg::Box(osg::Vec3(rel_pose[0], rel_pose[1], rel_pose[2]), 0.25);
        box->setRotation(osg::Quat(quat[1], quat[2], quat[3], quat[0]));

        osg::ref_ptr<osg::ShapeDrawable> shape = new osg::ShapeDrawable(box);
        if (i==frame)
            shape->setColor(osg::Vec4(0, 1, 0, 0.6));
        else
            shape->setColor(osg::Vec4(0.4, 0.6, 0.4, 0.6));

        csg->addChild(shape);
    }

    // rmeove old children
    while (getNumChildren() > 0)
        removeChild(0, 1);

    addChild(csg);
}

} // namespace osg
