/*
 * viewer/lib/nodes/points.cpp
 *
 * Point cloud code based on https://roboticcreatures.wordpress.com/20
 *      11/12/29/loading-3d-point-clouds-pcd-format-in-openscenegraph/.
 */

// C, C++
#include <iostream>

// OSG
#include <osg/Vec3d>
#include <osg/MatrixTransform>
#include <osg/Point>

// perls
#include <perls-common/units.h>

// local headers
#include "library/viewer/util/utils.hpp"

#include "points.hpp"

namespace osg
{

Points::Points(osg::ColorMap::Type cmap) :
        osg::Geometry(),
        _vertices(new osg::Vec3Array),
        _colors(new osg::Vec4Array),
        _cmap(cmap)
{
    setUseDisplayList(false);

    // // VBOs necessary for updating points dynamically
    // setUseVertexBufferObjects(true);

    // // Set VBO to STREAMING since attributes are changing per frame
    // osg::ref_ptr<osg::VertexBufferObject> df =
    //         getOrCreateVertexBufferObject();
    // df->setUsage (GL_STREAM_DRAW);

    setVertexArray(_vertices);
    setColorArray(_colors);
    setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    _draw_arrays = new osg::DrawArrays(
        osg::PrimitiveSet::POINTS, 0, _vertices->size());
    _draw_arrays->setDataVariance(osg::Object::DYNAMIC);
    addPrimitiveSet(_draw_arrays);

    // set initial bounding box to view all point
    //setInitialBound(
    //    osg::BoundingBox(-_k_bbox,-_k_bbox,-_k_bbox,_k_bbox,_k_bbox,_k_bbox));

    osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    state->setAttribute(new osg::Point(3), osg::StateAttribute::ON);
}

void Points::add_vertices(int num_points,
                          double *xyz,
                          uint8_t *intensity)
{
    for (int i = 0; i < num_points; i++) {
        _vertices->push_back(osg::Vec3(xyz[3*i], xyz[3*i+1], xyz[3*i+2]));
        //_colors->push_back(osg::ColorMap::Mapper::at(intensity[i], _cmap));

        double z = (-xyz[3*i + 2])/5.0;
        if (z<0) z = 0;
        if (z>1) z = 1;
        _colors->push_back(osg::ColorMap::Mapper::at(z * 255, _cmap));
    }
}

void Points::add_vertices(Eigen::MatrixXd& xyz)
{
    int num_points = xyz.cols();
    for (int i = 0; i < num_points; i++) {
        _vertices->push_back(osg::Vec3(xyz(0, i), xyz(1, i), xyz(2, i)));

        double z = (xyz(2, i)+4)/12;
        if (z<0) z = 0;
        if (z>1) z = 1;
        _colors->push_back(osg::ColorMap::Mapper::at(z * 255, _cmap));
    }
}

void Points::add_vertices(Eigen::MatrixXd& xyz, Eigen::VectorXd &depth)
{
    int num_points = xyz.cols();
    for (int i = 0; i < num_points; i++) {
        _vertices->push_back(osg::Vec3(xyz(0, i), xyz(1, i), xyz(2, i)));

        double d = depth(i)/40;
        if (d<0) d=0;
        if (d>1) d=1;

        _colors->push_back(osg::ColorMap::Mapper::at(d * 255, _cmap));
    }
}

Points::Points(int num_points,
               double *xyz,
               uint8_t *intensity,
               osg::ColorMap::Type cmap) :
        Points(cmap)
{
    add_vertices(num_points, xyz, intensity);

    setVertexArray(_vertices);
    setColorArray(_colors);
    setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    _draw_arrays = new osg::DrawArrays(
        osg::PrimitiveSet::POINTS, 0, _vertices->size());
    addPrimitiveSet(_draw_arrays);

    //_geode->addDrawable(this);
    osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

Points::Points(Eigen::MatrixXd& xyz,
               osg::ColorMap::Type cmap) :
        Points(cmap)
{
    add_vertices(xyz);

    setVertexArray(_vertices);
    setColorArray(_colors);
    setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    _draw_arrays = new osg::DrawArrays(
        osg::PrimitiveSet::POINTS, 0, _vertices->size());
    addPrimitiveSet(_draw_arrays);

    //_geode->addDrawable(this);
    osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

Points::Points(Eigen::MatrixXd& xyz, Eigen::VectorXd &depth,
               osg::ColorMap::Type cmap) :
        Points(cmap)
{
    add_vertices(xyz, depth);

    setVertexArray(_vertices);
    setColorArray(_colors);
    setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    _draw_arrays = new osg::DrawArrays(
        osg::PrimitiveSet::POINTS, 0, _vertices->size());
    addPrimitiveSet(_draw_arrays);

    //_geode->addDrawable(this);
    osg::ref_ptr<osg::StateSet> state = getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

void Points::push_back(int num_points, double *xyz, uint8_t *intensity)
{
    add_vertices(num_points, xyz, intensity);

    // // force redraw
    // _vertices->dirty();
    // _colors->dirty();
    _draw_arrays->setCount(_vertices->size());
    // _draw_arrays->dirty();
    dirtyBound();
}

void Points::pop_front(int num_points)
{
    _vertices->erase(_vertices->begin(), _vertices->begin()+num_points);
    _colors->erase(_colors->begin(), _colors->begin()+num_points);

    // // force redraw
    // _vertices->dirty();
    // _colors->dirty();
    _draw_arrays->setCount(_vertices->size());
    // _draw_arrays->dirty();
    dirtyBound();
}

} // namespace osg
