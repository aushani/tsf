#ifndef _VIEWER_LIB_NODES_COLORFULBOX_HPP_
#define _VIEWER_LIB_NODES_COLORFULBOX_HPP_

// OSG
#include <osg/ShapeDrawable>

namespace osg
{

class ColorfulBox : public osg::ShapeDrawable
{
  public:
    ColorfulBox(osg::Vec4 color);

  protected:
    virtual ~ColorfulBox() = default;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_COLORFULBOX_HPP_
