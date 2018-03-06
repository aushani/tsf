/*
 */

// C, C++
#include <cstdio>
#include <iostream>

// perls
#include "thirdparty/perls-math/so3.h"
#include "thirdparty/perls-math/ssc.h"

// viewer
#include "kitti_velodyne.hpp"

namespace osg
{

KittiVelodyne::KittiVelodyne() :
        osg::Group(),
        _x_ws{{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}},
        _p_active(nullptr)
{
    // initialize fasttrig for perls so3
    fasttrig_init();
}

KittiVelodyne::~KittiVelodyne()
{

}

void
KittiVelodyne::set_point_cloud(const velodyne_returns_t *vr)
{
    osg::ref_ptr<Points> p = new Points(vr->num_returns, vr->xyz, vr->intensity);

    if (_p_active != nullptr)
        this->removeChild(_p_active);

    this->addChild(p);
    _p_active = p;
}

} // namespace osg

