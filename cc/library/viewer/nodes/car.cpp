/*
 * viewer/lib/nodes/car.cpp
 */

// C, C++
#include <iostream>

// OSG
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>

// perls
#include <perls-common/units.h>

// local headers
#include "library/viewer/util/utils.hpp"

#include "car.hpp"

namespace osg
{

Car::Car() :
        osg::MatrixTransform()
{
    const double k_d2r = UNITS_DEGREE_TO_RADIAN;

    // read car file into osg::Node ptr
    osg::ref_ptr<osg::Node> car =
            osgDB::readNodeFile(ViewerUtils::get_src_dir() + _k_car_file);

    // TODO: throw an exception
    if (car == nullptr) {
        std::cerr << "error reading car file" << std::endl;
    } else {

      // scale and rotate car to +x, z down
      // TODO: magic numbers, specific to lexus model
      osg::Matrixd H(osg::Quat(180*k_d2r, osg::Vec3d(0,0,1)));
      H.postMultRotate(osg::Quat(-90*k_d2r, osg::Vec3d(1,0,0)));
      H.postMultScale(osg::Vec3d(_k_scale, _k_scale, _k_scale));
      H.postMultTranslate(_k_pos);
      setMatrix(H);

      addChild(car);

      // re-normalize normals after scaling
      osg::ref_ptr<osg::StateSet> car_stateset = car->getOrCreateStateSet();
      car_stateset->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
    }
}

} // namespace osg
