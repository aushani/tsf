#include <iostream>

#include "layered_image.hpp"

#include "library/viewer/nodes/composite_shape_group.hpp"
#include "library/viewer/nodes/points.hpp"

#include "library/flow/image_roi.h"

namespace osg
{

LayeredImage::LayeredImage(int width, int height, double x0, double y0, double scale):
    osg::KittiImage(width, height, x0, y0, scale),
    _point_cloud(new osg::Geode()),
    _depth_map_geode(new osg::Geode()),
    _flow(new osg::Geode()),
    _roi(new osg::Geode()),
    _roi_sidebar(new osg::Geode()),
    _roi_depth_mask_geode(new osg::Geode()),
    _depth_texture(new osg::Texture2D()),
    _roi_texture(new osg::Texture2D()),
    _roi_depth_mask_texture(new osg::Texture2D())
{
    add_to_hud(_point_cloud);
    add_to_hud(_depth_map_geode);
    add_to_hud(_flow);
    add_to_hud(_roi);
    add_to_hud(_roi_sidebar);
    add_to_hud(_roi_depth_mask_geode);

    // Add the depth texture
    set_up_texture(_depth_texture, _depth_map_geode, 0, 0, _width, _height, 12);

    // Add the roi texture
    set_up_texture(_roi_texture, _roi_sidebar, _width, 0, 1.5*16, 1.5*16*14, 12);

    // Add the roi depth mask texture
    set_up_texture(_roi_depth_mask_texture, _roi_depth_mask_geode, _width, 0, 1.5*16, 1.5*16*14, 12);

    // State set for display on top of image
    osg::ref_ptr<osg::StateSet> ss_disp = new osg::StateSet();

    _point_cloud->setStateSet(ss_disp);
    //_depth_map_geode->setStateSet(ss_disp);
    _flow->setStateSet(ss_disp);
    _roi->setStateSet(ss_disp);

    ss_disp->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss_disp->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    ss_disp->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    ss_disp->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    ss_disp->setRenderBinDetails(12, "RenderBin");

}

LayeredImage::~LayeredImage()
{
    if (_depth_map) depth_buffer_destroy(_depth_map);
    if (_roi_grid) image_roi_grid_destroy(_roi_grid);
}

void
LayeredImage::set_selected_position(double x, double y)
{
    if (_roi_grid == NULL)
        return;

    // Remove old roi
    clear_selected_position();

    double z_max = _sog->res * _sog->n_dim[2]/2;
    double z_min = -z_max;

    double x_center, y_center, z_center;
    sparse_occ_grid_center_xyz(_sog, x, y, 0, &x_center, &y_center, &z_center);

    int grid_i, grid_j, grid_k;
    sparse_occ_grid_get_ijk(_sog, x, y, 0, &grid_i, &grid_j, &grid_k);

    for (double z=z_min; z<z_max; z+=_sog->res) {

        Eigen::MatrixXd p_roi(4, 4);

        p_roi(0, 0) = x_center - _sog->res/2;
        p_roi(1, 0) = y_center + _sog->res/2;
        p_roi(2, 0) = z + _sog->res/2;
        p_roi(3, 0) = 1;

        p_roi(0, 1) = x_center - _sog->res/2;
        p_roi(1, 1) = y_center + _sog->res/2;
        p_roi(2, 1) = z - _sog->res/2;
        p_roi(3, 1) = 1;

        p_roi(0, 2) = x_center - _sog->res/2;
        p_roi(1, 2) = y_center - _sog->res/2;
        p_roi(2, 2) = z - _sog->res/2;
        p_roi(3, 2) = 1;

        p_roi(0, 3) = x_center - _sog->res/2;
        p_roi(1, 3) = y_center - _sog->res/2;
        p_roi(2, 3) = z + _sog->res/2;
        p_roi(3, 3) = 1;

        Eigen::MatrixXd p_c = project_into_camera_frame(p_roi);

        double x1_c, y1_c, z1_c;
        double x2_c, y2_c, z2_c;

        for (int i=0; i<4; i++) {

            x1_c = p_c(0, i)/p_c(2, i);
            y1_c = p_c(1, i)/p_c(2, i);
            z1_c = p_c(2, i);

            x2_c = p_c(0, (i+1)%4)/p_c(2, (i+1)%4);
            y2_c = p_c(1, (i+1)%4)/p_c(2, (i+1)%4);
            z2_c = p_c(2, (i+1)%4);

            if (z1_c < 0) continue;
            if (z2_c < 0) continue;

            if (x1_c < 0) x1_c = 0;
            if (x1_c > _width) x1_c = _width;

            if (y1_c < 0) y1_c = 0;
            if (y1_c > _height) y1_c = _height;

            if (x2_c < 0) x2_c = 0;
            if (x2_c > _width) x2_c = _width;

            if (y2_c < 0) y2_c = 0;
            if (y2_c > _height) y2_c = _height;

            osg::Vec3 sp(x1_c, _height-y1_c, 0);
            osg::Vec3 ep(x2_c, _height-y2_c, 0);
            osg::ref_ptr<Geometry> beam( new osg::Geometry);
            osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
            points->push_back(sp);
            points->push_back(ep);
            osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
            color->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
            beam->setVertexArray(points.get());
            beam->setColorArray(color.get());
            //beam->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
            beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));

            _roi->addDrawable(beam);

        }
    }

    image_roi_t *roi = image_roi_grid_get_at(_roi_grid, grid_i, grid_j);

    osg::ref_ptr<osg::Image> im = new osg::Image();
    im->allocateImage(roi->width, roi->height, _image->r(), _image->getPixelFormat(), _image->getDataType());

    osg::ref_ptr<osg::Image> im_dm = new osg::Image();
    im_dm->allocateImage(roi->width, roi->height, _image->r(), _image->getPixelFormat(), _image->getDataType());


    for (int i=0; i<roi->width; i++) {
        for (int j=0; j<roi->height; j++) {

            float r = image_roi_get(roi, i, j, 0);
            float g = image_roi_get(roi, i, j, 1);
            float b = image_roi_get(roi, i, j, 2);

            osg::Vec4 color(r, g, b, 1.0f);

            int d_err_byte = image_roi_get_depth_err_byte(roi, i, j);

            osg::ColorMap::Type cmap = osg::ColorMap::Type::JET;
            osg::Vec4 mask_color = osg::ColorMap::Mapper::at(d_err_byte, cmap);

            mask_color[3] = 0.01f;

            im->setColor(color, i, j, 0);
            im_dm->setColor(mask_color, i, j, 0);
        }
    }

    _roi_texture->setImage(im);
    _roi_depth_mask_texture->setImage(im_dm);

    image_roi_destroy(roi);
}

void
LayeredImage::clear_selected_position()
{
    // Remove old roi
    while (_roi->getNumDrawables() > 0)
        _roi->removeDrawables(0, _flow->getNumDrawables());
}

Eigen::MatrixXd
LayeredImage::project_into_camera_frame(Eigen::MatrixXd p_x)
{
    Eigen::MatrixXd p_c = _P_rect * _R_rect * _T_cv * p_x;

    return p_c;
}

void
LayeredImage::set_image_calib(Eigen::MatrixXd P_rect, Eigen::MatrixXd R_rect, Eigen::MatrixXd T_cv)
{
    _P_rect = P_rect;
    _R_rect = R_rect;
    _T_cv = T_cv;
}

void
LayeredImage::set_point_cloud(velodyne_returns_t *vr)
{
    printf("Have scan of %d points\n", vr->num_returns);

    printf("Make point array\n");
    // Create point array
    Eigen::MatrixXd p_x(4, vr->num_returns);
    for (int i=0; i<vr->num_returns; i++) {
        p_x(0, i) = vr->xyz[3*i+0];
        p_x(1, i) = vr->xyz[3*i+1];
        p_x(2, i) = vr->xyz[3*i+2];
        p_x(3, i) = 1;
    }

    printf("Project\n");
    Eigen::MatrixXd p_c = project_into_camera_frame(p_x);

    printf("Draw\n");
    int count = 0;
    for (int i=0; i<vr->num_returns; i++) {

        double x_c = p_c(0, i)/p_c(2, i);
        double y_c = p_c(1, i)/p_c(2, i);

        if (x_c < 0 || x_c > _width)
            continue;

        if (y_c < 0 || y_c > _height)
           continue;

        if (p_c(2, i)<0)
           continue;

        count++;
    }

    Eigen::MatrixXd p_render(3, count);
    Eigen::VectorXd d_render(count);
    int i_at = 0;
    for (int i=0; i<vr->num_returns; i++) {

        double x_c = p_c(0, i)/p_c(2, i);
        double y_c = p_c(1, i)/p_c(2, i);

        if (x_c < 0 || x_c > _width)
            continue;

        if (y_c < 0 || y_c > _height)
           continue;

        if (p_c(2, i)<0)
           continue;

        p_render(0, i_at) = x_c;
        p_render(1, i_at) = _height-y_c-1;
        p_render(2, i_at) = 0;

        d_render(i_at) = p_c(2, i);

        i_at++;
    }

    printf("Remove drawables\n");
    while (_point_cloud->getNumDrawables() > 0)
        _point_cloud->removeDrawables(0, _point_cloud->getNumDrawables());

    // Add points
    osg::ref_ptr<Points> p = new Points(p_render, d_render);
    _point_cloud->addDrawable(p);

    printf("Done drawing\n");
}

void
LayeredImage::set_depth_map(const depth_buffer_t *depth_map)
{
    if (_depth_map) depth_buffer_destroy(_depth_map);
    _depth_map = depth_buffer_copy(depth_map);

    osg::ref_ptr<osg::Image> im = new osg::Image();

    im->allocateImage(_image->s(), _image->t(), _image->r(), _image->getPixelFormat(), _image->getDataType());
    //im->setOrigin(osg::Image::Origin::TOP_LEFT);

    osg::ColorMap::Type cmap = osg::ColorMap::Type::JET;

    for (int x=0; x<_width; x++) {
        for (int y=0; y<_height; y++) {

            float depth = depth_buffer_get(_depth_map, x, y);
            osg::Vec4 color;

            if (depth==0)
                color = osg::Vec4(1, 1, 1, 0.25f);
            else {
                double r=0, g=0, b=0;

                if (depth<25.0) {
                    b = 1-depth/25.0;
                    g = depth/25.0;
                } else if (depth < 50.0) {
                    g = 1-(depth-25.0)/25.0;
                    r = (depth-25.0)/25.0;
                } else {
                    r = 1;
                }

                double val = (depth)/40;
                if (val<0) val = 0;
                if (val>1) val = 1;

                color = osg::ColorMap::Mapper::at(val * 255, cmap);
            }

            im->setColor(color, x, _height-y-1, 0);
        }
    }

    printf("Setting depth texture\n");
    _depth_texture->setImage(im);
}

void
LayeredImage::set_image_roi_grid(const image_roi_grid_t *image_roi_grid)
{
    if (_roi_grid) image_roi_grid_destroy(_roi_grid);
    _roi_grid = image_roi_grid_make_host_copy(image_roi_grid);
}

void
LayeredImage::set_flow_image(sparse_occ_grid_t *sog, flow_image_t *flow)
{
    // Remove old flow
    while (_flow->getNumDrawables() > 0)
        _flow->removeDrawables(0, _flow->getNumDrawables());

    sparse_occ_grid_it_t it;
    sparse_occ_grid_it_init(sog, &it, 0);

    int64_t key;
    float val;

    int i, j, k;

    double x1, y1, z1;
    double x2, y2, z2;

    double x1_c, y1_c, z1_c;
    double x2_c, y2_c, z2_c;

    while (sparse_occ_grid_it_next(&it, &key, &val)) {

        if (val < l_thresh)
            continue;

        sparse_occ_grid_idx_to_ijk(sog, key, &i, &j, &k);

        flow_t f = flow_image_get_flow(flow, i, j);

        if (!f.valid)
            continue;

        sparse_occ_grid_get_xyz(sog, key, &x1, &y1, &z1);

        x2 = x1 + f.u*flow->res;
        y2 = y1 + f.v*flow->res;
        z2 = z1;

        Eigen::MatrixXd p(4, 2);

        p(0, 0) = x1;
        p(1, 0) = y1;
        p(2, 0) = z1;
        p(3, 0) = 1;

        p(0, 1) = x2;
        p(1, 1) = y2;
        p(2, 1) = z2;
        p(3, 1) = 1;

        Eigen::MatrixXd p_c = project_into_camera_frame(p);

        x1_c = p_c(0, 0)/p_c(2, 0);
        y1_c = p_c(1, 0)/p_c(2, 0);
        z1_c = p_c(2, 0);

        x2_c = p_c(0, 1)/p_c(2, 1);
        y2_c = p_c(1, 1)/p_c(2, 1);
        z2_c = p_c(2, 1);

        // Check to make sure we're in camera view
        if (x1_c < 0 || x1_c > _width) continue;
        if (y1_c < 0 || y1_c > _height) continue;
        if (z1_c < 0) continue;

        if (x2_c < 0 || x2_c > _width) continue;
        if (y2_c < 0 || y2_c > _height) continue;
        if (z2_c < 0) continue;

        osg::Vec3 sp(x1_c, _height-y1_c, 0);
        osg::Vec3 ep(x2_c, _height-y2_c, 0);
        osg::ref_ptr<Geometry> beam( new osg::Geometry);
        osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
        points->push_back(sp);
        points->push_back(ep);
        osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
        color->push_back(osg::Vec4(1.0,0.0,0.0,1.0));
        beam->setVertexArray(points.get());
        beam->setColorArray(color.get());
        //beam->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
        beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES,0,2));

        _flow->addDrawable(beam);
    }
}

void
LayeredImage::render_point_cloud(bool state)
{
    _point_cloud->setNodeMask(state);
}

void
LayeredImage::render_depth_map(bool state)
{
    _depth_map_geode->setNodeMask(state);
}

void
LayeredImage::render_flow(bool state)
{
    _flow->setNodeMask(state);
}

void
LayeredImage::render_roi(bool state)
{
    _roi->setNodeMask(state);
    _roi_sidebar->setNodeMask(state);
}

void
LayeredImage::render_roi_depth_mask(bool state)
{
    _roi_depth_mask_geode->setNodeMask(state);

    if (state) arash_count++;
}

}
