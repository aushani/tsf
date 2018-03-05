#ifndef _LAYERED_IMAGE_NODE_H_
#define _LAYERED_IMAGE_NODE_H_

// OSG
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/Geode>
#include <osg/Projection>

#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>
#include <osgUtil/Optimizer>
#include <osgViewer/ViewerEventHandlers>

// Eigen
#include <Eigen/Core>

#include "library/sensors/velodyne.h"

#include "library/ray_tracing/sparse_occ_grid.h"

#include "library/kitti/nodes/kitti_image.hpp"

#include "library/flow/flow_image.h"
#include "library/flow/depth_buffer.h"
#include "library/flow/image_roi_grid.h"

namespace osg
{

class LayeredImage : public osg::KittiImage
{

  public:

    LayeredImage(int width, int height, double x0=0, double y0=0, double scale=1.0);
    ~LayeredImage();

    void set_occ_grid(sparse_occ_grid_t *grid) {_sog = grid;}

    void set_flow_image(sparse_occ_grid_t *sog, flow_image_t *flow);

    void set_selected_position(double x, double y);
    void clear_selected_position();

    void set_point_cloud(velodyne_returns_t *vr);
    void set_depth_map(const depth_buffer_t *depth_map);
    void set_image_roi_grid(const image_roi_grid_t *image_roi_grid);

    void render_point_cloud(bool state);
    void render_depth_map(bool state);
    void render_flow(bool state);
    void render_roi(bool state);
    void render_roi_depth_mask(bool state);

    void set_image_calib(Eigen::MatrixXd P_rect, Eigen::MatrixXd R_rect, Eigen::MatrixXd T_cv);

  private:

    Eigen::MatrixXd project_into_camera_frame(Eigen::MatrixXd p_x);

    osg::ref_ptr<osg::Geode> _point_cloud;
    osg::ref_ptr<osg::Geode> _depth_map_geode;
    osg::ref_ptr<osg::Geode> _flow;
    osg::ref_ptr<osg::Geode> _roi;
    osg::ref_ptr<osg::Geode> _roi_sidebar;
    osg::ref_ptr<osg::Geode> _roi_depth_mask_geode;

    osg::ref_ptr<osg::Texture2D> _depth_texture;
    osg::ref_ptr<osg::Texture2D> _roi_texture;
    osg::ref_ptr<osg::Texture2D> _roi_depth_mask_texture;

    sparse_occ_grid_t *_sog = NULL;
    depth_buffer_t *_depth_map = NULL;
    image_roi_grid_t *_roi_grid = NULL;

    // Camera intrinsics and extrinsics
    Eigen::MatrixXd _P_rect, _R_rect, _T_cv;

    int arash_count = 0;
};

}

#endif
