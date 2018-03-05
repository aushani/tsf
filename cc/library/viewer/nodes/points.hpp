#ifndef _VIEWER_LIB_NODES_POINTS_HPP_
#define _VIEWER_LIB_NODES_POINTS_HPP_

// C, C++
#include <iostream>
#include <string>

// OSG
#include <osg/Geometry>
#include <osg/Drawable>
#include <osg/MatrixTransform>

// Eigen
#include <Eigen/Core>

// local
#include "library/viewer/util/colormap.hpp"

namespace osg
{

class Points : public osg::Geometry
{
  public:

    // for dynamic point cloud
    Points(osg::ColorMap::Type cmap = osg::ColorMap::Type::JET);

    // initialize all arrays before adding to root node
    // for static point cloud
    Points(int num_points,
           double *xyz,
           uint8_t *intensity,
           osg::ColorMap::Type cmap = osg::ColorMap::Type::JET);

    Points(Eigen::MatrixXd& xyz,
           osg::ColorMap::Type cmap = osg::ColorMap::Type::JET);

    Points(Eigen::MatrixXd& xyz, Eigen::VectorXd& depth,
           osg::ColorMap::Type cmap = osg::ColorMap::Type::JET);

    // add points to back of points queue
    void push_back(int num_points, double *xyz, uint8_t *intensity);

    // remove from front of points queue
    void pop_front(int num_points);

  private:

    void add_vertices(int num_points, double *xyz, uint8_t *intensity);
    void add_vertices(Eigen::MatrixXd& xyz);
    void add_vertices(Eigen::MatrixXd& xyz, Eigen::VectorXd& depth);

    // geometry parameters
    osg::ref_ptr<osg::Vec3Array> _vertices;
    osg::ref_ptr<osg::Vec4Array> _colors;
    osg::ref_ptr<osg::DrawArrays> _draw_arrays;

    osg::ColorMap::Type _cmap;

  protected:
    virtual ~Points() = default;
};

} // namespace osg

#endif // _VIEWER_LIB_NODES_POINTS_HPP_
