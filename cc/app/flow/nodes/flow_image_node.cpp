#include <osg/Geometry>
#include <osg/LineWidth>

#include "library/viewer/nodes/colorful_box.hpp"
#include "library/viewer/nodes/composite_shape_group.hpp"

#include "flow_image_node.hpp"

#include "thirdparty/perls-math/ssc.h"

namespace osg
{

FlowImage::FlowImage() :
    osg::Group(),
    _cb(new osg::FlowImageCallback())
{
    setUpdateCallback(_cb);

    pthread_mutex_init(&_mutex, nullptr);

    occ_grid = NULL;
    flow = NULL;

    _color_flow = osg::Vec4(0, 0, 1, 0);
}

FlowImage::~FlowImage()
{
    if (occ_grid)
        sparse_occ_grid_destroy(occ_grid);

    if (flow)
        flow_image_destroy(flow);
}

void
FlowImage::lock()
{
    pthread_mutex_lock(&_mutex);
}

void
FlowImage::unlock()
{
    pthread_mutex_unlock(&_mutex);
}

void
FlowImage::set_flow_image(sparse_occ_grid_t *og, flow_image_t *f)
{
    lock();

    if (occ_grid)
        sparse_occ_grid_destroy(occ_grid);
    if (flow)
        flow_image_destroy(flow);

    occ_grid = sparse_occ_grid_copy(og, 1, 0);
    flow = flow_image_copy(f);

    rendered = false;

    unlock();
}

void
FlowImage::set_rel_pose(double rp[6])
{
    lock();

    for (int i=0; i<6; i++)
        rel_pose[i] = rp[i];

    unlock();
}

void
FlowImage::set_render_in_ego_frame(bool ef)
{
    lock();
    _ego_frame = ef;
    rendered = false;
    unlock();
}

bool
FlowImage::get_render_in_ego_frame()
{
    return _ego_frame;
}

void
FlowImage::set_selected_position(double x, double y)
{
    lock();
    sel_x = x;
    sel_y = y;
    sel_active = true;
    rendered_sel = false;
    unlock();
}

void
FlowImage::set_color(osg::Vec4 color)
{
    lock();
    _color_flow = color;
    unlock();
}

osg::Vec4
FlowImage::get_color()
{
    return _color_flow;
}

FlowImageCallback::FlowImageCallback() :
    osg::Callback()
{
}

bool
FlowImageCallback::run(osg::Object *object, osg::Object *data)
{
    osg::FlowImage *f = (osg::FlowImage*) object;
    if (f == NULL)
        return true;

    f->lock();

    if (f->init && f->rendered == false && f->occ_grid != NULL && f->flow != NULL) {

        while (f->getNumChildren() > 0)
            f->removeChild(0, 1);

        // Iterate over occ grid and add occupied cells
        sparse_occ_grid_it_t git;
        sparse_occ_grid_it_init(f->occ_grid, &git, 0);

        int64_t key;
        float val;
        double x, y, z;
        int i, j, k;

        while (sparse_occ_grid_it_next(&git, &key, &val)) {

            if (val<=0)
                continue;

            sparse_occ_grid_get_xyz(f->occ_grid, key, &x, &y, &z);
            sparse_occ_grid_get_ijk(f->occ_grid, x, y, z, &i, &j, &k);

            flow_t f_ij = flow_image_get_flow(f->flow, i, j);

            //if ( (f_ij.u == 0 && f_ij.v == 0) || f_ij.valid==0)
            if (f_ij.valid==0)
                continue;

            double dx = -f_ij.u * f->flow->res;
            double dy = -f_ij.v * f->flow->res;

            if (!f->get_render_in_ego_frame()) {

                double x_1[6] = {x, y, 0, 0, 0, 0};
                double x_2[6] = {x + dx, y + dy, 0, 0, 0, 0};

                double x_21[6];
                ssc_tail2tail(x_21, NULL, f->rel_pose, x_2);

                dx = (x_21[0] - x_1[0]);
                dy = (x_21[1] - x_1[1]);
            }

            osg::Vec3 sp(x, y, z);
            osg::Vec3 ep(x+dx, y+dy, z);

            osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();

            // set vertices
            vertices->push_back(osg::Vec3(x, y, z));
            vertices->push_back(osg::Vec3(x+dx, y+dy, z));

            osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();
            osg::ref_ptr<osg::DrawElementsUInt> line =
                    new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
            line->push_back(0);
            line->push_back(1);
            geometry->addPrimitiveSet(line);

            osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(4.0);
            geometry->getOrCreateStateSet()->setAttribute(linewidth);

            // turn off lighting
            geometry->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

            osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
            colors->push_back(f->get_color());
            geometry->setColorArray(colors);
            geometry->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);

            geometry->setVertexArray(vertices);

            f->addChild(geometry);
        }

        f->rendered = true;
    }

    if (f->init && f->rendered_sel == false && f->occ_grid != NULL && f->flow != NULL) {

        if (f->sel_node)
            f->removeChild(f->sel_node);

        if (f->sel_active) {
            osg::ref_ptr<osg::CompositeShapeGroup> csg_sel = new osg::CompositeShapeGroup();
            csg_sel->get_sdrawable()->setColor(osg::Vec4(0, 0, 1, 0.5));

            // Render selected position
            double x, y, z;
            sparse_occ_grid_center_xyz(f->occ_grid, f->sel_x, f->sel_y, 0, &x, &y, &z);
            double z_max = f->occ_grid->res * f->occ_grid->n_dim[2]/2;
            double z_min = -z_max;
            float scale = f->occ_grid->res * 0.95;
            for (double z=z_min; z<z_max; z+=f->occ_grid->res)
                csg_sel->get_cshape()->addChild(new osg::Box(osg::Vec3(x, y, z), scale));
            f->addChild(csg_sel);

            f->sel_node = csg_sel;
        }

        f->rendered_sel = true;
    }

    f->unlock();

    return true;
}

} // namespace osg
