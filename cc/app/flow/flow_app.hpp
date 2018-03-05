#ifndef _FLOW_APP_H_
#define _FLOW_APP_H_

#include <vector>
#include <Eigen/Core>

#include "library/sensors/velodyne.h"

#include "library/ray_tracing/gpu_occ_grid_builder.h"

#include "library/flow/flow_solver.h"
#include "library/flow/depth_buffer_builder.h"
#include "library/flow/image_roi_grid_builder.h"

#include "library/kitti/tracklets.hpp"
#include "library/kitti/util.hpp"
#include "library/kitti/pose.hpp"

#include "library/flow/state_filter.hpp"
#include "library/flow/flow_tracklets.hpp"

#include "app/flow/viewer.hpp"

// Forward declaration
class FlowViewerWindow;

class FlowApp
{

  public:

    FlowApp();
    ~FlowApp();

    void set_viewer_window(FlowViewerWindow *vw);

    void prev_scan(bool compute_flow = true);
    void refresh();
    void next_scan(bool compute_flow = true);
    void update_params();
    int n_scans() {return _vrs.size();}
    int scan_at() {return _scan_at;}

    void update_scan(int i, bool compute_flow = true, bool tracklets_good = false);
    void project_flow(double dt);

    void set_em_iterations(int x);
    void set_smoothing_weight(float w);

    void set_selected_position(double x, double y);

    void set_velodyne_returns(std::vector<velodyne_returns_t*> vrs);
    void set_poses(std::vector<Pose> poses);
    void set_raw_poses(std::vector<Pose> raw_poses);
    void set_tracklets(Tracklets *t);
    void set_images(std::vector<osg::ref_ptr<osg::Image> > imgs);

    void load_calibration(std::string dirname);
    void load_bg_filter(std::string dir);

    const velodyne_returns_t* get_vr(int i);

    void save_tracklets(std::string dir, bool append);
    void save_matches(std::string dir, bool append);
    void save_eval(std::string dir, bool append);

    void set_forward(bool forward);
    void set_filter(bool filter);

    void set_ready(bool ready) {_ready = ready;}

    const static int default_em_iterations = 20;
    constexpr static float default_smoothing_weight = 1.0f;

  private:

    depth_buffer_t* build_depth_map(velodyne_returns_t *vr, depth_buffer_t *db);

    std::string get_object_type_at_location(double x, double y);
    bool find_corresponding_col(double x, double y, double *x2, double *y2);
    void find_corresponding_col_raw_pose_only(double x, double y, double *x2, double *y2);
    void find_corresponding_col_filtered_pose(double x, double y, double *x2, double *y2);

    void load_instrinsics(FILE *f_cc);
    void load_extrinsics(FILE *f_vc);

    bool in_camera_view(double x, double y, double z);

    std::vector<velodyne_returns_t*> _vrs;
    std::vector<Pose> _poses;
    std::vector<Pose> _raw_poses;
    std::vector<StateFilter> _filtered_poses; // compute real time for timing results to be correct
    std::vector<Pose> _filtered_poses_full; // store for rendering and what not
    std::vector<osg::ref_ptr<osg::Image> > _images;

    Tracklets *_tracklets;

    FlowViewerWindow* _vw = NULL;

    unsigned int _scan_at;

    gpu_occ_grid_builder_t *_builder;
    depth_buffer_builder_t *_depth_builder;
    image_roi_grid_builder_t *_rg_builder;
    flow_solver_t *_solver;

    sparse_occ_grid_t *_occ_grid_1 = NULL, *_occ_grid_2 = NULL;
    //sparse_occ_grid_multires_t *_g_mr1 = NULL, *_g_mr2 = NULL;

    flow_image_t *_flow_image = NULL;

    float *_sparse_depth = NULL;
    depth_buffer_t *_depth_buffer_1 = NULL;
    depth_buffer_t *_depth_buffer_2 = NULL;

    image_roi_grid_t *_roi_grid_1 = NULL;
    image_roi_grid_t *_roi_grid_2 = NULL;

    bool _calib_loaded = false;
    Eigen::MatrixXd _R_rect;
    Eigen::MatrixXd _P_rect;
    Eigen::MatrixXd _T_cv;

    bool _forward = true;
    bool _use_filter = true;

    int _uv_window = 15;

    bool _ready = false;

    int _nx, _ny, _nz;
    float _resolution;

    FlowTracklets _flow_tracklets;

    float *_wf;
    float *_wo;
    float _offset;
    float _bias;

    int _bg_window_size = 2;

};

#endif
