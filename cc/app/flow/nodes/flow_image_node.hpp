#ifndef _FLOW_IMAGE_NODE_H_
#define _FLOW_IMAGE_NODE_H_

// OSG
#include <osg/MatrixTransform>

#include "library/flow/flow_image.h"
#include "library/ray_tracing/sparse_occ_grid.h"

namespace osg
{

class FlowImageCallback : public osg::Callback
{

  public:

    FlowImageCallback();

    bool run(osg::Object *object, osg::Object *data) override;

  protected:

    virtual ~FlowImageCallback() = default;
};


class FlowImage : public osg::Group
{

  public:

    FlowImage();
    ~FlowImage();

    void set_flow_image(sparse_occ_grid_t *og, flow_image_t *flow);
    void set_selected_position(double x, double y);

    void set_color(osg::Vec4 color);
    osg::Vec4 get_color();

    void set_rel_pose(double rp[6]);

    void set_render_in_ego_frame(bool ef);
    bool get_render_in_ego_frame();

    void lock();
    void unlock();

    sparse_occ_grid_t *occ_grid;
    flow_image_t *flow;
    bool rendered = false;
    bool rendered_sel = false;

    bool init = false;

    double sel_x, sel_y;
    bool sel_active = false;

    osg::Node *sel_node= NULL;

    double rel_pose[6];

  private:

    osg::ref_ptr<osg::FlowImageCallback> _cb;

    pthread_mutex_t _mutex;

    osg::Vec4 _color_flow;

    bool _ego_frame;
};

} // namespace osg

#endif
