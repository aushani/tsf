#include "flow_tracklets.hpp"

#include "thirdparty/perls-math/so3.h"
#include "thirdparty/perls-math/ssc.h"

#include <map>

FlowTracklets::FlowTracklets(int n_x, int n_y, double res)
{
    _nx = n_x;
    _ny = n_y;
    _res = res;

    int sz = n_x * n_y;

    _tracklets = (StateFilter**) calloc(sizeof(StateFilter*), sz);
    _next = (StateFilter**) calloc(sizeof(StateFilter*), sz);

    pthread_mutex_init(&_mutex, nullptr);
}

FlowTracklets::FlowTracklets(const FlowTracklets& ft)
{
    printf("NEED TO IMPLEMENT THIS!\n");
}

FlowTracklets::~FlowTracklets()
{
    for (int i=0; i<_nx; i++)
        for (int j=0; j<_ny; j++) {
            clear_tracklet(i, j);
            //clear_tracklet_next_frame(i, j);
        }

    free(_tracklets);
    free(_next);
}

void
FlowTracklets::lock()
{
    pthread_mutex_lock(&_mutex);
}

void
FlowTracklets::unlock()
{
    pthread_mutex_unlock(&_mutex);
}

StateFilter*
FlowTracklets::get_tracklet(int i, int j)
{
    if (i>=0 && i<_nx && j>=0 && j<=_ny)
        return _tracklets[i * _ny + j];
    else
        return NULL;
}

StateFilter*
FlowTracklets::get_tracklet_next_frame(int i, int j)
{
    if (i>=0 && i<_nx && j>=0 && j<=_ny)
        return _next[i * _ny + j];
    else
        return NULL;
}

void
FlowTracklets::put_tracklet(int i, int j, StateFilter *sf)
{
    clear_tracklet(i, j);
    _tracklets[i*_ny + j] = sf;
}

void
FlowTracklets::put_tracklet_next_frame(int i, int j, StateFilter *sf)
{
    clear_tracklet_next_frame(i, j);
    _next[i*_ny + j] = sf;
}

bool
FlowTracklets::has_tracklet(int i, int j)
{
    return get_tracklet(i, j) != NULL;
}

bool
FlowTracklets::has_tracklet_next_frame(int i, int j)
{
    return get_tracklet_next_frame(i, j) != NULL;
}

void
FlowTracklets::clear_tracklet(int i, int j)
{
    StateFilter *sf = get_tracklet(i, j);
    if (sf) delete sf;

    _tracklets[i*_ny + j] = NULL;
}

void
FlowTracklets::clear_tracklet_next_frame(int i, int j)
{
    StateFilter *sf = get_tracklet_next_frame(i, j);
    if (sf) delete sf;

    _next[i*_ny + j] = NULL;
}

void
FlowTracklets::process_flow_image(flow_image_t *f, double x_12[6])
{
    lock();

    double res = f->res;
    double dt = 0.1;

    for (int i=0; i<f->n_dim[0]; i++) {
        for (int j=0; j<f->n_dim[1]; j++) {

            flow_t flow = flow_image_get_flow(f, i, j);

            double x = (i - _nx/2) * res;
            double y = (j - _ny/2) * res;

            if (flow.valid) {

                // Look up state filter and update it, or create a new one if need be

                double dx = res*flow.u;
                double dy = res*flow.v;

                double x_1[6] = {x, y, 0, 0, 0, 0};
                double x_2[6] = {x + dx, y + dy, 0, 0, 0, 0};

                double x_21[6];
                ssc_tail2tail(x_21, NULL, x_12, x_2);

                dx = -(x_21[0] - x_1[0]);
                dy = -(x_21[1] - x_1[1]);

                int i_to = i + flow.u;
                int j_to = j + flow.v;

                //int i_to = i + (int) (dx/res + 0.5);
                //int j_to = j + (int) (dy/res + 0.5);

                if (has_tracklet(i, j)) {

                    StateFilter *sf = get_tracklet(i, j);

                    // Propagate it forward
                    sf->run_process_model(dt);

                    // Make an observation
                    StateFilter::FlowObs z;

                    //z(0, 0) = flow.u*res/dt;
                    //z(1, 0) = flow.v*res/dt;
                    //double mahal = sf->flow_observation(z);

                    z(0, 0) = (i_to  - _nx/2) * res;
                    z(1, 0) = (j_to  - _ny/2) * res;

                    double mahal = sf->flow_observation_pos(z);

                    if (mahal < 5.9915) { // 95%
                        // Put it in next frame
                        put_tracklet_next_frame(i_to, j_to, sf);
                    } else {
                        //printf("StateFilter failed mahal: %5.3f\n", mahal);
                        clear_tracklet(i, j);;
                    }

                } else {

                    StateFilter *sf = new StateFilter();

                    // Init it
                    double x_from = x;
                    double y_from = y;

                    sf->init(x_from, y_from, flow.u*res/dt, flow.v*res/dt);

                    // Place it
                    put_tracklet(i, j, sf);
                    put_tracklet_next_frame(i_to, j_to, sf);
                }

            } else {

                // Whatever tracklet used to be here is dead to us
                clear_tracklet(i, j);
            }
        }
    }

    unlock();
}

void
FlowTracklets::update_locations(Pose p1, Pose p2)
{
    lock();

    /*
    // First clear out current frame
    // Everything in _tracklets is either also in next, or has been cleared out
    // because it doesn't exist anymore
    memset(_tracklets, 0, sizeof(StateFilter*)*_nx*_ny);

    // Go through everything in next and just update poses and put them in tracklets
    for (int i=0; i<_nx; i++) {
        for (int j=0; j<_ny; j++) {

            StateFilter *sf = get_tracklet_next_frame(i, j);
            if (sf == NULL)
                continue;

            double x = i*_res;
            double y = j*_res;

            // Project (x, y) in frame 1 to frame 2
            double x_1[6] = {x, y, 0, 0, 0, 0};

            // for numerical accuracy
            double x0 = p1.x;
            double y0 = p1.y;
            double z0 = p1.z;
            double x_1w[6] = {p1.x-x0, p1.y-y0, p1.z-z0, p1.r, p1.p, p1.h};
            double x_w[6];
            ssc_head2tail(x_w, NULL, x_1w, x_1);

            double x_2w[6] = {p2.x-x0, p2.y-y0, p2.z-z0, p2.r, p2.p, p2.h};
            double x_2[6];
            ssc_tail2tail(x_2, NULL, x_2w, x_w);

            int i_to = round(x_2[0]/_res);
            int j_to = round(x_2[1]/_res);

            // Need to transform the StateFilter as well
            sf->change_coordinate_frames(p1, p2);

            // Check bounds
            if (i_to < 0 || i_to >= _nx || j_to < 0 || j_to >= _ny) {
                clear_tracklet_next_frame(i, j);
                continue;
            } else {
                put_tracklet(i_to, j_to, sf);
            }
        }
    }

    // Now clear out _next
    memset(_next, 0, sizeof(StateFilter*)*_nx*_ny);
    */

    StateFilter **tmp = _tracklets;
    _tracklets = _next;
    _next = tmp;

    // Now clear out _next
    memset(_next, 0, sizeof(StateFilter*)*_nx*_ny);

    unlock();
}

sparse_occ_grid_t*
FlowTracklets::apply_to_sog(sparse_occ_grid_t* sog, double dt) {

    const double t_step = 0.001;

    std::map<int64_t, float> kv_out;

    sparse_occ_grid_it_t git;
    sparse_occ_grid_it_init(sog, &git, 0);

    int64_t key;
    float val;
    int i, j, k;
    int i_new, j_new, k_new;

    while (sparse_occ_grid_it_next(&git, &key, &val)) {

        sparse_occ_grid_idx_to_ijk(sog, key, &i, &j, &k);

        int64_t key_new;

        if (has_tracklet_next_frame(i, j)) {

            // Project tracklet forward
            StateFilter *sf = get_tracklet_next_frame(i, j);
            StateFilter::State mu = sf->get_mu();

            double x = mu(0, 0);
            double y = mu(1, 0);
            double heading = mu(2, 0);
            double vel_flow = mu(3, 0);
            double heading_dot = mu(4, 0);

            for (double t_proj = 0; t_proj < dt; t_proj += t_step) {

                double st = sin(heading);
                double ct = cos(heading);

                x       += vel_flow * ct * t_step;
                y       += vel_flow * st * t_step;
                heading += heading_dot * t_step;
            }

            sparse_occ_grid_get_ijk(sog, x, y, 0, &i_new, &j_new, &k_new);
            k_new = k;

            sparse_occ_grid_ijk_to_idx(sog, i_new, j_new, k_new, &key_new);

        } else {
            key_new = key;
        }

        // Check for negative key (ie, out of range)
        if (key_new < 0) {
            continue;
        }

        if (kv_out.count(key_new) == 0)
            kv_out[key_new] = val;
        else
            kv_out[key_new] += val;

    }

    int n_kv = kv_out.size();

    // Allocate memory for new keys and values
    int64_t *keys = (int64_t*) calloc(sizeof(int64_t), n_kv);
    float *vals = (float*) calloc(sizeof(float), n_kv);

    int count = 0;
    for (auto it = kv_out.begin(); it != kv_out.end(); it++) {
        keys[count] = it->first;
        vals[count] = it->second;
        count++;
    }

    if (count != n_kv) {
        printf("WARNING: Count is off\n");
    }

    sparse_occ_grid_t *sog_out = sparse_occ_grid_create(sog->res, sog->n_dim);
    sog_out->keys = keys;
    sog_out->vals = vals;
    sog_out->n_kv = n_kv;
    sog_out->d_keys = NULL;
    sog_out->d_vals = NULL;

    return sog_out;
}
