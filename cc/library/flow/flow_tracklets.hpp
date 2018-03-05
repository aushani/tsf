#ifndef _FLOW_TRACKLETS_H_
#define _FLOW_TRACKLETS_H_

#include <vector>

#include "library/flow/flow_image.h"

#include "library/flow/state_filter.hpp"

#include "library/kitti/pose.hpp"

#include "library/ray_tracing/sparse_occ_grid.h"

class FlowTracklets
{
  public:

    FlowTracklets(int n_x, int n_y, double res);
    FlowTracklets(const FlowTracklets& ft);
    ~FlowTracklets();

    StateFilter* get_tracklet(int i, int j);
    bool has_tracklet(int i, int j);

    void process_flow_image(flow_image_t *f, double x_12[6]);

    void update_locations(Pose p1, Pose p2);

    void lock();
    void unlock();

    int get_nx() {return _nx;}
    int get_ny() {return _ny;}
    double get_res() {return _res;}

    sparse_occ_grid_t* apply_to_sog(sparse_occ_grid_t* sog, double dt);

  private:

    void put_tracklet(int i, int j, StateFilter *sf);

    void put_tracklet_next_frame(int i, int j, StateFilter *sf);

    void clear_tracklet(int i, int j);
    void clear_tracklet_next_frame(int i, int j);


    StateFilter* get_tracklet_next_frame(int i, int j);
    bool has_tracklet_next_frame(int i, int j);


    StateFilter **_tracklets, **_next;
    int _nx, _ny;

    double _res;

    pthread_mutex_t _mutex;

};

#endif
