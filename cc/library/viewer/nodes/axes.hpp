#ifndef _VIEWER_LIB_NODES_AXES_HPP_
#define _VIEWER_LIB_NODES_AXES_HPP_

// C, C++
#include <string>

// OSG
#include <osg/MatrixTransform>

namespace osg
{

class Axes : public osg::MatrixTransform
{
  public:

    Axes();

  private:

    // file module location (relative to utils.cpp)
    const std::string _k_axes_file = "/models/axes.osgt";
    const double _k_scale = 1.5;

  protected:
    virtual ~Axes() = default;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_AXES_HPP_
