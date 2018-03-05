#ifndef _VIEWER_LIB_NODES_CAR_HPP_
#define _VIEWER_LIB_NODES_CAR_HPP_

// C, C++
#include <string>

// OSG
#include <osg/Vec3d>
#include <osg/MatrixTransform>

namespace osg
{

class Car : public osg::MatrixTransform
{
  public:

    Car();

  private:

    // file module location (relative to utils.cpp)
    const std::string _k_car_file = "/models/lexus/lexus_hs.obj";
    const double _k_scale = 0.075;
    const osg::Vec3d _k_pos = osg::Vec3d(1.5, 0, 0.2);

  protected:
    virtual ~Car() = default;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_CAR_HPP_
