/*
 * viewer/lib/nodes/tetrahedron.cpp
 */

// local headers
#include "tetrahedron.hpp"

namespace osg
{

Tetrahedron::Tetrahedron(osg::Vec4 color) :
        osg::Geometry(),
        _colors(new osg::Vec4Array())
{
    // center to front, half of width, height
    const double k_dims[3] = { 1, 0.5, 0.2 };

    setUseDisplayList(false);
    setUseVertexBufferObjects(true);

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    vertices->push_back(osg::Vec3( k_dims[0],          0,          0));
    vertices->push_back(osg::Vec3(-k_dims[1],  k_dims[1],          0));
    vertices->push_back(osg::Vec3(-k_dims[1], -k_dims[1],          0));
    vertices->push_back(osg::Vec3(         0,          0, -k_dims[2]));
    setVertexArray(vertices);

    osg::ref_ptr<osg::DrawElementsUInt> face0 =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    osg::Plane plane0(vertices->at(0), vertices->at(3), vertices->at(1));
    face0->push_back(0);
    face0->push_back(3);
    face0->push_back(1);
    addPrimitiveSet(face0);

    osg::ref_ptr<osg::DrawElementsUInt> face1 =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    osg::Plane plane1(vertices->at(1), vertices->at(3), vertices->at(2));
    face1->push_back(1);
    face1->push_back(3);
    face1->push_back(2);
    addPrimitiveSet(face1);

    osg::ref_ptr<osg::DrawElementsUInt> face2 =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    osg::Plane plane2(vertices->at(2), vertices->at(3), vertices->at(0));
    face2->push_back(2);
    face2->push_back(3);
    face2->push_back(0);
    addPrimitiveSet(face2);

    osg::ref_ptr<osg::DrawElementsUInt> face3 =
            new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
    osg::Plane plane3(vertices->at(0), vertices->at(1), vertices->at(2));
    face3->push_back(0);
    face3->push_back(1);
    face3->push_back(2);
    addPrimitiveSet(face3);

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
    normals->push_back(plane0.getNormal());
    normals->push_back(plane1.getNormal());
    normals->push_back(plane2.getNormal());
    normals->push_back(plane3.getNormal());
    setNormalArray(normals);
    setNormalBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

    _colors->push_back(color);
    setColorArray(_colors);
    setColorBinding(osg::Geometry::BIND_OVERALL);
}

void Tetrahedron::change_color(osg::Vec4 color)
{
    _colors->erase(_colors->begin());
    _colors->push_back(color);
    _colors->dirty();
}

} // namespace osg
