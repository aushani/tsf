#ifndef _FLOW_TRACKLETS_NODE_H_
#define _FLOW_TRACKLETS_NODE_H_

// OSG
#include <osg/MatrixTransform>

#include "library/flow/flow_tracklets.hpp"
#include "library/flow/state_filter.hpp"

#include "library/ray_tracing/sparse_occ_grid.h"

namespace osg
{

class FlowTrackletsCallback : public osg::Callback
{

  public:

    FlowTrackletsCallback();

    bool run(osg::Object *object, osg::Object *data) override;

  protected:

    virtual ~FlowTrackletsCallback() = default;
};


class FlowTrackletsNode : public osg::Group
{

  public:

    FlowTrackletsNode();
    ~FlowTrackletsNode();

    void set_ft(FlowTracklets *ft) {tracklets = ft;}

    void set_rel_pose(double rp[6]);

    void set_render_in_ego_frame(bool ef);
    bool get_render_in_ego_frame();

    void update(sparse_occ_grid_t *sog, double rp[6]);

    void set_color(osg::Vec4 color);
    osg::Vec4 get_color();

    void lock();
    void unlock();

    FlowTracklets *tracklets = NULL;

    bool rendered = false;

    sparse_occ_grid_t *_sog;

    double rel_pose[6];

  private:

    osg::ref_ptr<osg::FlowTrackletsCallback> _cb;

    pthread_mutex_t _mutex;

    osg::Vec4 _color_flow;

    bool _ego_frame;

};

} // namespace osg

#endif
