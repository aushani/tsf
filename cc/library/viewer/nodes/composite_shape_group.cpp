/*
 * viewer/lib/nodes/composite_shape_group.cpp
 */

// OSG
#include <osg/BlendFunc>

// local headers
#include "composite_shape_group.hpp"

namespace osg
{

CompositeShapeGroup::CompositeShapeGroup() :
        osg::Group(),
        _cshape(new osg::CompositeShape)
{
    _sdrawable = new osg::ShapeDrawable(_cshape);

    // use alpha to draw transparent objects
    osg::ref_ptr<osg::StateSet> set = _sdrawable->getOrCreateStateSet();
    set->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    set->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA,
                                                 GL_ONE_MINUS_SRC_ALPHA),
                              osg::StateAttribute::ON);

    addChild(_sdrawable);
}

} // namespace osg
