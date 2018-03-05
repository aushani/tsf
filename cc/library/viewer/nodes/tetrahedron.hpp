#ifndef _VIEWER_LIB_NODES_TETRAHEDRON_HPP_
#define _VIEWER_LIB_NODES_TETRAHEDRON_HPP_

// OSG
#include <osg/Geometry>

namespace osg
{

class Tetrahedron : public osg::Geometry
{
  public:
    Tetrahedron(osg::Vec4 color);

    static const char* get_class_name()
    { return "Tetrahedron"; };

    const char* className() const override
    { return get_class_name(); };

    void change_color(osg::Vec4 color);

  protected:
    virtual ~Tetrahedron() = default;

  private:
    osg::ref_ptr<osg::Vec4Array> _colors;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_TETRAHEDRON_HPP_
