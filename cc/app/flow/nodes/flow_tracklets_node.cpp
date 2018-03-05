#include <osg/Geometry>
#include <osg/LineWidth>

#include "library/viewer/nodes/colorful_box.hpp"
#include "library/viewer/nodes/composite_shape_group.hpp"

#include "flow_tracklets_node.hpp"

#include "perls-math/ssc.h"

namespace osg
{

FlowTrackletsNode::FlowTrackletsNode() :
    osg::Group(),
    _cb(new osg::FlowTrackletsCallback())
{
    setUpdateCallback(_cb);

    pthread_mutex_init(&_mutex, nullptr);

    _color_flow = osg::Vec4(0, 0, 1, 0);

    _sog = NULL;
}

FlowTrackletsNode::~FlowTrackletsNode()
{
}

void
FlowTrackletsNode::update(sparse_occ_grid_t *sog, double rp[6])
{
    if (_sog)
        sparse_occ_grid_destroy(_sog);
    _sog = sparse_occ_grid_copy(sog, 1, 0);

    rendered = false;

    set_rel_pose(rp);
}

void
FlowTrackletsNode::lock()
{
    pthread_mutex_lock(&_mutex);
    if (tracklets != NULL) tracklets->lock();
}

void
FlowTrackletsNode::unlock()
{
    if (tracklets != NULL) tracklets->unlock();
    pthread_mutex_unlock(&_mutex);
}

void
FlowTrackletsNode::set_color(osg::Vec4 color)
{
    //lock();
    _color_flow = color;
    //unlock();
}

osg::Vec4
FlowTrackletsNode::get_color()
{
    return _color_flow;
}

void
FlowTrackletsNode::set_rel_pose(double rp[6])
{
    lock();

    for (int i=0; i<6; i++)
        rel_pose[i] = rp[i];

    unlock();
}

void
FlowTrackletsNode::set_render_in_ego_frame(bool ef)
{
    lock();
    _ego_frame = ef;
    rendered = false;
    unlock();
}

bool
FlowTrackletsNode::get_render_in_ego_frame()
{
    return _ego_frame;
}


FlowTrackletsCallback::FlowTrackletsCallback() :
    osg::Callback()
{
}

bool
FlowTrackletsCallback::run(osg::Object *object, osg::Object *data)
{
    osg::FlowTrackletsNode *f = (osg::FlowTrackletsNode*) object;
    if (f == NULL || f->tracklets == NULL)
        return true;

    f->lock();

    if (f->rendered == false && f->_sog != NULL) {

        while (f->getNumChildren() > 0)
            f->removeChild(0, 1);

        int nx = f->tracklets->get_nx();
        int ny = f->tracklets->get_ny();
        double res = f->tracklets->get_res();

        for (int i=0; i<nx; i++) {
            for (int j=0; j<ny; j++) {

                StateFilter *sf = f->tracklets->get_tracklet(i, j);

                if (sf == NULL)
                    continue;

                // If too young don't render
                if (sf->get_age() < 3)
                    continue;

                double x = (i - nx/2) * res;
                double y = (j - ny/2) * res;

                for (int k = 0; k < f->_sog->n_dim[2]; k++) {

                    float val = sparse_occ_grid_lookup(f->_sog, i, j, k);
                    if (val <= 0)
                        continue;

                    double z = (k - f->_sog->n_dim[2]/2) * res;

                    StateFilter::State mu = sf->get_mu();
                    double t = mu(2, 0);
                    double v = mu(3, 0);

                    double dx = -cos(t) * v * 0.1;
                    double dy = -sin(t) * v * 0.1;

                    if (!f->get_render_in_ego_frame()) {

                        double x_1[6] = {x, y, 0, 0, 0, 0};
                        double x_2[6] = {x + dx, y + dy, 0, 0, 0, 0};

                        double x_21[6];
                        ssc_tail2tail(x_21, NULL, f->rel_pose, x_2);

                        dx = (x_21[0] - x_1[0]);
                        dy = (x_21[1] - x_1[1]);
                    }

                    if (fabs(dx)<res && fabs(dy)<res) continue;

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

            }
        }

        f->rendered = true;
    }

    f->unlock();

    return true;
}

} // namespace osg
