#ifndef _VIEWER_LIB_NODES_COMPOSITESHAPEGROUP_HPP_
#define _VIEWER_LIB_NODES_COMPOSITESHAPEGROUP_HPP_

// OSG
#include <osg/Group>
#include <osg/ShapeDrawable>
#include <osg/Shape>

namespace osg
{

class CompositeShapeGroup : public osg::Group
{
  public:
    CompositeShapeGroup();

    osg::ref_ptr<osg::CompositeShape> get_cshape() const
    { return _cshape; }

    osg::ref_ptr<osg::ShapeDrawable> get_sdrawable() const
    { return _sdrawable; }

  private:
    osg::ref_ptr<osg::CompositeShape> _cshape;
    osg::ref_ptr<osg::ShapeDrawable> _sdrawable;

  protected:
    virtual ~CompositeShapeGroup() = default;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_COMPOSITESHAPEGROUP_HPP_
