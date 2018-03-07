#include "thirdparty/perls-common/timestamp.h"
#include "thirdparty/perls-common/timeutil.h"
#include "thirdparty/perls-math/so3.h"
#include "thirdparty/perls-math/ssc.h"

#include "app/flow/flow_app.hpp"

#include "library/util/util.h"

#include "library/flow/image_roi.h"

#define GICP_MAX_ITERS 50
#define GICP_EPS 0.001
#define GICP_D_MAX 3
#define GICP_THREAD_POOL_MAX_THREADS 8
#define GICP_MDIST_THRESH 0.90
#define GICP_DEBUG false

FlowApp::FlowApp() :
    _nx(167), // 49.8 m
    _ny(167), // 49.8 m
    _nz(14), // 4.2 m
    _resolution(0.30),
    _flow_tracklets(_nx, _ny, _resolution)
{
    _vw = NULL;

    _scan_at = 0;

    int max_hits = 130000;
    int threads_per_block = 1024;
    int max_hits_per_ray = -1;

    int n_dim[3] = {_nx, _ny, _nz};

    printf("Making gpu occ grid builder...\n");
    _builder = gpu_occ_grid_builder_create(max_hits, threads_per_block, max_hits_per_ray, _resolution, n_dim);
    _builder->verbose = 1;
    printf("Done making gpu occ grid builder\n");

    printf("Making image roi grid builder...\n");
    int im_w = 1242;
    int im_h = 375;
    int roi_w = 16;
    int roi_h = 16*14;
    int depth = 3;
    _rg_builder = image_roi_grid_builder_create(n_dim, _resolution, im_w, im_h, roi_w, roi_h, depth);
    printf("Done making image roi grid builder\n");

    _solver = flow_solver_create(n_dim, default_em_iterations, default_smoothing_weight, threads_per_block);
    _solver->uv_window = _uv_window;

    _depth_builder = depth_buffer_builder_create(im_w, im_h);

}

FlowApp::~FlowApp()
{
    gpu_occ_grid_builder_destroy(_builder);
    flow_solver_destroy(_solver);
    depth_buffer_builder_destroy(_depth_builder);
    image_roi_grid_builder_destroy(_rg_builder);

    if (_occ_grid_1) sparse_occ_grid_destroy(_occ_grid_1);
    if (_occ_grid_2) sparse_occ_grid_destroy(_occ_grid_2);

    if (_depth_buffer_1) depth_buffer_destroy(_depth_buffer_1);
    if (_depth_buffer_2) depth_buffer_destroy(_depth_buffer_2);

    if (_roi_grid_1) image_roi_grid_destroy(_roi_grid_1);
    if (_roi_grid_2) image_roi_grid_destroy(_roi_grid_2);

    if (_flow_image) flow_image_destroy(_flow_image);

    if (_wf) free(_wf);
    if (_wo) free(_wo);
}

void
FlowApp::set_viewer_window(FlowViewerWindow *vw)
{
    _vw = vw;
    _vw->set_image_calib(_P_rect, _R_rect, _T_cv);

    update_scan(_scan_at);

    _vw->set_tracklets(_tracklets);
    _vw->set_flow_tracklets(&_flow_tracklets);
    _vw->set_sm_poses(_poses);
    _vw->set_raw_poses(_raw_poses);
    _vw->set_filtered_poses(_filtered_poses_full);
    _vw->set_frame(_scan_at);
    _vw->set_image(_images[_scan_at], _images[_scan_at+1]);
}

void
FlowApp::load_bg_filter(std::string dir)
{
    int dim = (2*_bg_window_size) + 1;
    int dim2 = dim*dim;

    _wf = (float*) calloc(sizeof(float), dim2 * _nz);
    _wo = (float*) calloc(sizeof(float), dim2 * _nz);

    char filename_f[1000];
    char filename_o[1000];
    sprintf(filename_f, "%s/wf.csv", dir.c_str());
    sprintf(filename_o, "%s/wo.csv", dir.c_str());
    FILE *fp_f = fopen(filename_f, "r");
    FILE *fp_o = fopen(filename_o, "r");

    for (int i=0; i<dim2*_nz; i++) {
        if (fscanf(fp_f, "%f,", &_wf[i]) != 1)
            printf("ERROR COULD NOT READ wf!!!\n");

        if (fscanf(fp_o, "%f,", &_wo[i]) != 1)
            printf("ERROR COULD NOT READ wo!!!\n");

        //printf("%5.3f\n", _wo[i]);
    }

    fclose(fp_f);
    fclose(fp_o);

    char filename_b[1000];
    sprintf(filename_b, "%s/b.csv", dir.c_str());
    FILE *fp_b = fopen(filename_b, "r");
    if (fscanf(fp_b, "%f", &_offset) != 1)
        printf("ERROR COULD NOT READ offset!\n");
    fclose(fp_b);

    _bias = -1.827; // chosen for 95% recall

    //printf("Offset: %5.3f\n", _offset);
}

void
FlowApp::prev_scan(bool compute_flow)
{
    if (_scan_at > 0) {
        _scan_at--;
        update_scan(_scan_at, compute_flow);
    }
}

void
FlowApp::refresh()
{
    update_scan(_scan_at);
}

void
FlowApp::next_scan(bool compute_flow)
{
    if (_scan_at < _vrs.size() - 2) {
        _scan_at++;
        update_scan(_scan_at, compute_flow, true);
    }
}

void
FlowApp::set_em_iterations(int x)
{
    _solver->em_iterations = x;
}

void
FlowApp::set_smoothing_weight(float w)
{
    printf("Smoothing iterations set to %3.1f\n", w);
    _solver->smoothing_weight = w;
}

void
FlowApp::set_forward(bool forward) {
    _forward = forward;
}

void
FlowApp::set_filter(bool filter) {
    _use_filter = filter;
}

void
FlowApp::update_scan(int frame, bool compute_flow, bool tracklets_good)
{
    if (!_ready)
        return;

    if (frame>=0 && frame<=_vrs.size()-1) {

        printf("------------------------------------\n\n");
        int64_t tic_total = utime_now();

        velodyne_returns_t *vr_1 = _vrs[frame];
        velodyne_returns_t *vr_2 = _vrs[frame+1];

        osg::ref_ptr<osg::Image> image_1 = _images[frame];
        osg::ref_ptr<osg::Image> image_2 = _images[frame+1];

        if (!_forward) {
            velodyne_returns_t *tmp = vr_1;
            vr_1 = vr_2;
            vr_2 = tmp;
        }

        // Filter out points that are too close to body
        int32_t *f1 = (int32_t*) malloc(vr_1->num_returns*sizeof(int32_t));
        int32_t *f2 = (int32_t*) malloc(vr_2->num_returns*sizeof(int32_t));

        for (int i=0; i<vr_1->num_returns; i++) {
            double x = vr_1->xyz[3*i + 0];
            double y = vr_1->xyz[3*i + 1];
            double z = vr_1->xyz[3*i + 2];

            double r2 = x*x + y*y + z*z;
            f1[i] = (fabs(x-1.5)<3) && (fabs(y)<1.75);
        }

        for (int i=0; i<vr_2->num_returns; i++) {
            double x = vr_2->xyz[3*i + 0];
            double y = vr_2->xyz[3*i + 1];
            double z = vr_2->xyz[3*i + 2];

            double r2 = x*x + y*y + z*z;
            f2[i] = (fabs(x-1.5)<3) && (fabs(y)<1.75);
        }

        // Update pose
        if (_filtered_poses.size() <= frame + 1) {

            printf("Filtering pose...\n");

            if (_filtered_poses.size() > frame) {

                // Get a copy of the last filtered pose
                StateFilter sf(_filtered_poses[frame]);

                // Run the process model forward
                sf.run_process_model(0.1);

                // Get the observation we just had
                sf.pose_observation(_raw_poses[frame+1]);

                // Put it on the list
                _filtered_poses.push_back(sf);
            }
            printf("Done filtering pose...\n");
        }

        Pose p1 = _filtered_poses[frame].get_pose();
        Pose p2 = _filtered_poses[frame+1].get_pose();
        //Pose p1 = _poses[i];
        //Pose p2 = _poses[i+1];
        double x_12[6];
        double x_1[6] = {p1.x, p1.y, p1.z, p1.r, p1.p, p1.h};
        double x_2[6] = {p2.x, p2.y, p2.z, p2.r, p2.p, p2.h};
        ssc_tail2tail(x_12, NULL, x_1, x_2);
        printf("x_12 = %7.5f, %7.5f, %7.5f\n", x_12[0], x_12[1], x_12[5]*180.0/M_PI);

        // Do we need to update locations from previous position?
        if (compute_flow && frame > 0 && tracklets_good) {

            int64_t tic_ft = utime_now();

            _flow_tracklets.update_locations(p1, p2);
            int64_t toc_ft = utime_now();
            double t_ft = (toc_ft - tic_ft)/1e3;
            printf("Took %5.3f ms to update flow tracklets positions\n", t_ft);

        }


        // Build occ grid
        double pose[6] = {0, 0, 0, 0, 0, 0};

        // Check to see if occ grid 2 should just become occ grid 1
        if (_occ_grid_2 && (vr_1->utime == _occ_grid_2->utime)) {

            sparse_occ_grid_destroy(_occ_grid_1);
            printf("Shifting over occ grid to to occ grid 1\n");
            _occ_grid_1 = _occ_grid_2;

            _occ_grid_2 = gpu_occ_grid_builder_build(_builder, vr_2, pose, f2, vr_2->utime);
        } else {

            // Just regenerate everything
            if (_occ_grid_1) sparse_occ_grid_destroy(_occ_grid_1);
            if (_occ_grid_2) sparse_occ_grid_destroy(_occ_grid_2);

            _occ_grid_1 = gpu_occ_grid_builder_build(_builder, vr_1, pose, f1, vr_1->utime);
            _occ_grid_2 = gpu_occ_grid_builder_build(_builder, vr_2, pose, f2, vr_2->utime);
        }

        free(f1);
        free(f2);

        // Filter occ grids
        if (_use_filter) {
            float b = _offset - _bias;
            sparse_occ_grid_build_filter(_occ_grid_1, _wf, _wo, b, _bg_window_size);
        }
        //sparse_occ_grid_build_filter(_occ_grid_2, wf, wo, b);

        // now build flow
        if (_flow_image) flow_image_destroy(_flow_image);

        if (compute_flow) {

            // Find x_12
            /*
            Pose p1 = _filtered_poses[i].get_pose();
            Pose p2 = _filtered_poses[i+1].get_pose();

            double x_12[6];
            double x_1[6] = {p1.x, p1.y, p1.z, p1.r, p1.p, p1.h};
            double x_2[6] = {p2.x, p2.y, p2.z, p2.r, p2.p, p2.h};
            ssc_tail2tail(x_12, NULL, x_1, x_2);
            printf("x_12 = %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n", x_12[0], x_12[1], x_12[2], x_12[3], x_12[4], x_12[5]);

            _flow_image = flow_solver_compute_cookie(_solver, _occ_grid_1, _occ_grid_2, x_12);
            */
            _flow_image = flow_solver_compute_cookie(_solver, _occ_grid_1, _occ_grid_2);

            // Update flow tracklets
            if (tracklets_good) {
                printf("Update flow tracklets...\n");
                int64_t tic_tracklets = utime_now();
                _flow_tracklets.process_flow_image(_flow_image, x_12);
                int64_t toc_tracklets = utime_now();
                double t_tracklets = (toc_tracklets - tic_tracklets)/1e3;
                printf("Took %5.3f ms to update flow tracklets!\n", t_tracklets);
            }

        } else {
            _flow_image = NULL;
        }

        int64_t tic_depth = utime_now();
        if (_depth_buffer_2 && (vr_1->utime == _depth_buffer_2->utime)) {
            // Swap
            depth_buffer_t *tmp = _depth_buffer_1;
            _depth_buffer_1 = _depth_buffer_2;
            _depth_buffer_2 = tmp;

            _depth_buffer_2 = build_depth_map(vr_2, _depth_buffer_2);
            _depth_buffer_2->utime = vr_2->utime;
        } else {
            _depth_buffer_1 = build_depth_map(vr_1, _depth_buffer_1);
            _depth_buffer_2 = build_depth_map(vr_2, _depth_buffer_2);

            _depth_buffer_1->utime = vr_1->utime;
            _depth_buffer_2->utime = vr_2->utime;
        }
        int64_t toc_depth = utime_now();
        double t_depth = (toc_depth - tic_depth)/1e6;
        printf("Took %5.3f ms to get depth maps\n", t_depth*1000);

        /*
        int64_t tic_rg = utime_now();
        if (_roi_grid_2 && (vr_1->utime == _roi_grid_2->utime)) {

            // Swap
            image_roi_grid_t *tmp = _roi_grid_1;
            _roi_grid_1 = _roi_grid_2;
            _roi_grid_2 = tmp;

            _roi_grid_2 = image_roi_grid_builder_build(_rg_builder, image_2, _depth_buffer_2, _roi_grid_2);
            _roi_grid_2->utime = vr_2->utime;
        } else {
            _roi_grid_1 = image_roi_grid_builder_build(_rg_builder, image_1, _depth_buffer_1, _roi_grid_1);
            _roi_grid_2 = image_roi_grid_builder_build(_rg_builder, image_2, _depth_buffer_2, _roi_grid_2);

            _roi_grid_1->utime = vr_1->utime;
            _roi_grid_2->utime = vr_2->utime;
        }
        int64_t toc_rg = utime_now();
        double t_rg = (toc_rg - tic_rg)/1e3;
        printf("Took %5.3f ms to get image roi grid\n", t_rg);
        */

        int64_t toc_total = utime_now();
        double t_total = (toc_total - tic_total)/1e6;
        printf("\n---- Took %5.3f ms end to end ----\n\n", t_total*1000);

        if (_vw) {

            int64_t tic_gui = utime_now();

            _vw->set_point_cloud(vr_1, vr_2);

            _vw->set_image(image_1, image_2);
            _vw->set_depth_map(_depth_buffer_1, _depth_buffer_2);
            //_vw->set_image_roi_grid(_roi_grid_1, _roi_grid_2);

            _vw->set_occ_grids(_occ_grid_1, _occ_grid_2);

            _vw->set_flow_image(_occ_grid_1, _flow_image, x_12);

            _vw->set_frame(frame);

            _vw->update_flow_tracklets(_occ_grid_1, x_12);

            int64_t toc_gui = utime_now();
            double t_gui = (toc_gui - tic_gui)/1e3;
            printf("Took %5.3f ms to update gui\n", t_gui);
        }
    }
}

void
FlowApp::project_flow(double dt) {

    if (_occ_grid_2) {

        int64_t tic_apply_ft = utime_now();
        sparse_occ_grid_t* sog_forward = _flow_tracklets.apply_to_sog(_occ_grid_2, dt);
        int64_t toc_apply_ft = utime_now();
        double t_apply_ft = (toc_apply_ft - tic_apply_ft)/1e3;
        printf("Took %5.3f ms to project flow trackets %5.3f sec\n", t_apply_ft, dt);

        if (_vw)
            _vw->set_occ_grid_proj(sog_forward);

    }
}


depth_buffer_t*
FlowApp::build_depth_map(velodyne_returns_t *vr, depth_buffer_t *db)
{
    int64_t tic_sd = utime_now();

    // Create point array
    Eigen::MatrixXd p_x(4, vr->num_returns);
    for (int i=0; i<vr->num_returns; i++) {
        p_x(0, i) = vr->xyz[3*i+0];
        p_x(1, i) = vr->xyz[3*i+1];
        p_x(2, i) = vr->xyz[3*i+2];
        p_x(3, i) = 1;
    }

    // Project into camera frame
    Eigen::MatrixXd p_c = _P_rect * _R_rect * _T_cv * p_x;

    int width = 1242;
    int height = 375;

    if (_sparse_depth == NULL) {
        _sparse_depth = (float*) malloc(sizeof(float) * height * width);
    }

    for (int i=0; i<height*width; i++)
        _sparse_depth[i] = -1;

    for (int i=0; i<vr->num_returns; i++) {

        double x_c = p_c(0, i)/p_c(2, i);
        double y_c = p_c(1, i)/p_c(2, i);

        if (p_c(2, i)<0)
           continue;

        int w = x_c + 0.5;
        int h = y_c + 0.5;

        if (h < 0 || h >= height)
            continue;

        if (w < 0 || w >= width)
           continue;

        int idx = w * height + h;

        if (_sparse_depth[idx] < 0 || p_c(2, i) < _sparse_depth[idx])
            _sparse_depth[idx] = p_c(2, i);
    }
    int64_t toc_sd = utime_now();
    double t_sd = (toc_sd - tic_sd)/1e3;
    printf("Took %5.3f ms to generate sparse depth map\n", t_sd);

    // Now we have sparse depth
    db = depth_buffer_builder_build(_depth_builder, _sparse_depth, db);

    return db;
}

std::string
FlowApp::get_object_type_at_location(double x, double y)
{
    // Check if selected position is in any tracklets
    Tracklets::tPose *tp;
    Tracklets::tTracklet *tt;
    for (int i=0; i<_tracklets->numberOfTracklets(); i++) {

        if (!_tracklets->getPose(i, _scan_at, tp))
            continue;

        tt = _tracklets->getTracklet(i);

        // Is this point within tracklet?
        double x_1[6] = {x, y, 0, 0, 0, 0};
        double t_1[6] = {tp->tx, tp->ty, tp->tz, tp->rx, tp->ry, tp->rz};
        double x_t[6];
        ssc_tail2tail(x_t, NULL, t_1, x_1);

        // Check if we're inside this track, otherwise this is not the track we
        // are looking for...
        if (fabs(x_t[0])<tt->l/2 && fabs(x_t[1])<tt->w/2) {
            //printf("Inside!\n");
        } else {
            continue;
        }

        // Now project to next frame
        return tt->objectType;
    }

    return "NO_OBJECT";
}

bool
FlowApp::find_corresponding_col(double x, double y, double *x2, double *y2)
{
    // Check if selected position is in any tracklets
    Tracklets::tPose *tp;
    Tracklets::tTracklet *tt;
    for (int i=0; i<_tracklets->numberOfTracklets(); i++) {

        if (!_tracklets->getPose(i, _scan_at, tp))
            continue;

        tt = _tracklets->getTracklet(i);

        // Is this point within tracklet?
        double x_1[6] = {x, y, 0, 0, 0, 0};
        double t_1[6] = {tp->tx, tp->ty, tp->tz, tp->rx, tp->ry, tp->rz};
        double x_t[6];
        ssc_tail2tail(x_t, NULL, t_1, x_1);

        // Check if we're inside this track, otherwise this is not the track we
        // are looking for...
        if (fabs(x_t[0])<tt->l/2 && fabs(x_t[1])<tt->w/2) {
            //printf("Inside!\n");
        } else {
            continue;
        }

        // Now project to next frame
        if (!_tracklets->getPose(i, _scan_at+1, tp))
            continue;

        double t_2[6] = {tp->tx, tp->ty, tp->tz, tp->rx, tp->ry, tp->rz};
        double x_2[6];
        ssc_head2tail(x_2, NULL, t_2, x_t);

        *x2 = x_2[0];
        *y2 = x_2[1];
        return true;
    }

    // If we've gotten here, the given point is not part of a tracklet
    //printf("not inside\n");

    // Project (x, y) in frame 1 to frame 2
    double x_1[6] = {x, y, 0, 0, 0, 0};

    Pose p1 = _poses[_scan_at];
    // for numerical accuracy
    double x0 = p1.x;
    double y0 = p1.y;
    double z0 = p1.z;
    double x_1w[6] = {p1.x-x0, p1.y-y0, p1.z-z0, p1.r, p1.p, p1.h};
    double x_w[6];
    ssc_head2tail(x_w, NULL, x_1w, x_1);

    Pose p2 = _poses[_scan_at+1];
    double x_2w[6] = {p2.x-x0, p2.y-y0, p2.z-z0, p2.r, p2.p, p2.h};
    double x_2[6];
    ssc_tail2tail(x_2, NULL, x_2w, x_w);

    //printf("\t %5.3f %5.3f %5.3f\n", x_1[0], x_1[1], x_1[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_1w[0], x_1w[1], x_1w[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_2[0], x_2[1], x_2[2]);

    *x2 = x_2[0];
    *y2 = x_2[1];

    return false;
}

void
FlowApp::find_corresponding_col_raw_pose_only(double x, double y, double *x2, double *y2)
{
    // Project (x, y) in frame 1 to frame 2
    double x_1[6] = {x, y, 0, 0, 0, 0};

    Pose p1 = _raw_poses[_scan_at];
    // for numerical accuracy
    double x0 = p1.x;
    double y0 = p1.y;
    double z0 = p1.z;
    double x_1w[6] = {p1.x-x0, p1.y-y0, p1.z-z0, p1.r, p1.p, p1.h};
    double x_w[6];
    ssc_head2tail(x_w, NULL, x_1w, x_1);

    Pose p2 = _raw_poses[_scan_at+1];
    double x_2w[6] = {p2.x-x0, p2.y-y0, p2.z-z0, p2.r, p2.p, p2.h};
    double x_2[6];
    ssc_tail2tail(x_2, NULL, x_2w, x_w);

    //printf("\t %5.3f %5.3f %5.3f\n", x_1[0], x_1[1], x_1[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_1w[0], x_1w[1], x_1w[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_2[0], x_2[1], x_2[2]);

    *x2 = x_2[0];
    *y2 = x_2[1];
}

void
FlowApp::find_corresponding_col_filtered_pose(double x, double y, double *x2, double *y2) {

    // Project (x, y) in frame 1 to frame 2
    double x_1[6] = {x, y, 0, 0, 0, 0};

    Pose p1 = _filtered_poses[_scan_at].get_pose();
    // for numerical accuracy
    double x0 = p1.x;
    double y0 = p1.y;
    double z0 = p1.z;
    double x_1w[6] = {p1.x-x0, p1.y-y0, p1.z-z0, p1.r, p1.p, p1.h};
    double x_w[6];
    ssc_head2tail(x_w, NULL, x_1w, x_1);

    Pose p2 = _filtered_poses[_scan_at+1].get_pose();
    double x_2w[6] = {p2.x-x0, p2.y-y0, p2.z-z0, p2.r, p2.p, p2.h};
    double x_2[6];
    ssc_tail2tail(x_2, NULL, x_2w, x_w);

    //printf("\t %5.3f %5.3f %5.3f\n", x_1[0], x_1[1], x_1[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_1w[0], x_1w[1], x_1w[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_2[0], x_2[1], x_2[2]);

    *x2 = x_2[0];
    *y2 = x_2[1];
}

bool
FlowApp::in_camera_view(double x, double y, double z)
{
    if (_calib_loaded) {

        Eigen::Matrix<double, 4, 1> p_x;
        p_x(0, 0) = x;
        p_x(1, 0) = y;
        p_x(2, 0) = 0; // ground
        p_x(3, 0) = 1;

        Eigen::Matrix<double, 3, 1> p_c = _P_rect * _R_rect * _T_cv * p_x;

        if (p_c(2, 0) < 0)
            return false;

        double x_c = p_c(0, 0) / p_c(2, 0);
        double y_c = p_c(1, 0) / p_c(2, 0);

        if (x_c < 0 || x_c > 1392)
            return false;

        if (y_c < 0 || y_c > 512)
            return false;

        return true;
    }

    return false;
}

void
FlowApp::set_selected_position(double x, double y)
{
    if (_vw) {

        _vw->set_occ_grid1_sel_pos(x, y);

        double x2, y2;
        find_corresponding_col(x, y, &x2, &y2);

        _vw->set_occ_grid2_sel_pos(x2, y2);

        // Find position in occ grid 1
        int i, j, k;
        sparse_occ_grid_get_ijk(_occ_grid_1, x, y, 0, &i, &j, &k);

        // Lookup flow
        flow_t f = flow_image_get_flow(_flow_image, i, j);
        printf("Flow at %d, %d is %d, %d\n", i, j, f.u, f.v);
        printf("Object is: %s\n", get_object_type_at_location(x, y).c_str());
        double xf = x + f.u*_flow_image->res;
        double yf = y + f.v*_flow_image->res;
        _vw->set_flow_image_sel(xf, yf);

        //_vw->set_image_sel_pos(x, y);
        _vw->set_occ_grid2_sel_pos(xf, yf);

        // Lookup flow tracklets
        StateFilter *sf = _flow_tracklets.get_tracklet(i, j);
        if (sf != NULL) {
            printf("\tGot StateFilter with age %d!\n", sf->get_age());

            StateFilter::State mu = sf->get_mu();

            double x = mu(0, 0);
            double y = mu(1, 0);
            double t = mu(2, 0);
            double v = mu(3, 0);

            double ct = cos(t);
            double st = sin(t);

            double f_x = v*ct;
            double f_y = v*st;

            printf("\tPos: %5.3f %5.3f, flow is: %5.3f %5.3f\n", x, y, f_x, f_y);

        }

        bool in_view = in_camera_view(x, y, 0);
        printf("In Camera View: %s\n", in_view ? "YES":"NO");
    }
}

void
FlowApp::save_eval(std::string dir, bool append)
{
    char fn_e[1000]={0,}, fn_t[1000]={0,}, fn_f[1000]={0,};
    sprintf(fn_e, "%s/eval.csv", dir.c_str());
    sprintf(fn_t, "%s/eval_tracklets.csv", dir.c_str());
    sprintf(fn_f, "%s/eval_ft.csv", dir.c_str());

    FILE *fp_e = fopen(fn_e, append ? "a":"w");
    FILE *fp_t = fopen(fn_t, append ? "a":"w");
    FILE *fp_f = fopen(fn_f, append ? "a":"w");

    sparse_occ_grid_it_t git;
    sparse_occ_grid_it_init(_occ_grid_1, &git, 0);

    int64_t key;
    float val;
    double x, y, z;
    double x2, y2;

    int i, j, k;
    int last_i=-1, last_j=-1;

    int count_written = 0;
    int count_tracklets = 0;

    while (sparse_occ_grid_it_next(&git, &key, &val)) {

        if (val<=0)
            continue;

        sparse_occ_grid_idx_to_ijk(_occ_grid_1, key, &i, &j, &k);

        //if (sparse_occ_grid_check_filter(_occ_grid_1, i, j))
        //    continue;

        // Did we already do this column?
        if (i==last_i && j==last_j)
            continue;

        // Get position and project to next occ grid
        sparse_occ_grid_get_xyz(_occ_grid_1, key, &x, &y, &z);
        bool is_tracklet = find_corresponding_col(x, y, &x2, &y2);

        // Are either of these positions out of camera view? If so, we can't rely on
        // the tracklets, so we shouldn't use this data
        if (!in_camera_view(x, y, 0) || !in_camera_view(x2, y2, 0))
            continue;

        flow_t f = flow_image_get_flow(_flow_image, i, j);

        int filter = sparse_occ_grid_check_filter(_occ_grid_1, i, j);

        double u = f.u*_occ_grid_1->res;
        double v = f.v*_occ_grid_1->res;

        // If this flow isn't valid and we've labeled this as background
        if (!f.valid && filter) {

            // Get flow from filtered odojetry estimate
            double x2_f, y2_f;
            find_corresponding_col_filtered_pose(x, y, &x2_f, &y2_f);

            u = x2_f - x;
            v = y2_f - y;
        }

        std::string type = get_object_type_at_location(x, y);

        fprintf(fp_e, "%f,%f,%f,%f,%f,%f,%d,%d,%s\n",
                x, y, x2, y2, u, v, filter, is_tracklet ? 1:0, type.c_str());
        if (is_tracklet) {
            fprintf(fp_t, "%f,%f,%f,%f,%f,%f,%d,%d,%s\n",
                    x, y, x2, y2, u, v, filter, is_tracklet ? 1:0, type.c_str());
            count_tracklets++;
        }

        StateFilter *sf = _flow_tracklets.get_tracklet(i, j);

        if (sf != NULL)  {

            StateFilter::State mu = sf->get_mu();

            double t = mu(2, 0);
            double v = mu(3, 0);

            double st = sin(t);
            double ct = cos(t);

            double ft_x = v*ct;
            double ft_y = v*st;

            int age = sf->get_age();

            fprintf(fp_f, "%f,%f,%f,%f,%f,%f,%d,%d,%d,%s\n",
                    x, y, x2, y2, ft_x, ft_y, filter, is_tracklet ? 1:0, age, type.c_str());
        }

        count_written++;

    } // while

    printf("Wrote %d flows, %d tracklet flows\n", count_written, count_tracklets);

    fclose(fp_e);
    fclose(fp_t);
    fclose(fp_f);
}

void
FlowApp::save_tracklets(std::string dir, bool append)
{
    char fn[1000]={0,};
    sprintf(fn, "%s/tracklets.csv", dir.c_str());

    FILE *fp = fopen(fn, append ? "a":"w");

    sparse_occ_grid_it_t git;
    sparse_occ_grid_it_init(_occ_grid_1, &git, 0);

    int64_t key;
    float val;
    double x, y, z;
    double x2, y2;

    int i, j, k;
    int last_i=-1, last_j=-1;

    int count_written = 0;

    while (sparse_occ_grid_it_next(&git, &key, &val)) {

        // Only care about columns with stuff in them
        if (val <= 0)
            continue;

        sparse_occ_grid_idx_to_ijk(_occ_grid_1, key, &i, &j, &k);

        // Did we already do this column?
        if (i==last_i && j==last_j)
            continue;

        // Get position and project to next occ grid
        sparse_occ_grid_get_xyz(_occ_grid_1, key, &x, &y, &z);

        // Is this position out of camera view? If so, we can't rely on
        // the tracklets, so we shouldn't use this data
        if (!in_camera_view(x, y, 0))
            continue;

        bool is_tracklet = find_corresponding_col(x, y, &x2, &y2);

        if (!is_tracklet) {
            // Need to do some kind of random filtering, otherwise we get WAY
            // too much data
            int r = rand();
            if (r % 10 != 0)
                continue;
        }

        // Write out columns of 5x5 neighborhood
        int window = 2;
        double x2_raw, y2_raw;
        int i2_raw, j2_raw, k2_raw;

        for (int di=-window; di<=window; di++) {
            for (int dj=-window; dj<=window; dj++) {

                // Get neighborhood in the other scan as well
                double x_i = x + di*_occ_grid_1->res;
                double y_i = y + dj*_occ_grid_1->res;
                find_corresponding_col_raw_pose_only(x_i, y_i, &x2_raw, &y2_raw);
                sparse_occ_grid_get_ijk(_occ_grid_2, x2_raw, y2_raw, 0, &i2_raw, &j2_raw, &k2_raw);

                for (int k_it = 0; k_it<_occ_grid_1->n_dim[2]; k_it++) {
                    float lp1 = sparse_occ_grid_lookup(_occ_grid_1, i+di, j+dj, k_it);
                    float lp2 = sparse_occ_grid_lookup(_occ_grid_2, i2_raw, j2_raw, k_it);

                    fprintf(fp, "%3.1f,%3.1f,", lp1, lp2);
                }
            }
        }

        // Write out whether or not this is a tracklet
        fprintf(fp, "%d\n", is_tracklet ? 1:0);

        count_written++;

    } // while

    printf("Wrote %d\n", count_written);

    fclose(fp);
}

void
FlowApp::save_matches(std::string dir, bool append)
{
    char fn_m[1000]={0,}, fn_t[1000]={0,};
    sprintf(fn_m, "%s/matches.bin", dir.c_str());
    sprintf(fn_t, "%s/matches_tracklets.bin", dir.c_str());

    FILE *fp = fopen(fn_m, append ? "a":"w");
    FILE *fp_t = fopen(fn_t, append ? "a":"w");

    sparse_occ_grid_it_t git;
    sparse_occ_grid_it_init(_occ_grid_1, &git, 0);

    int64_t key;
    float val;
    double x1, y1, z1;
    double x2, y2;

    int i, j, k;
    int i2, j2, k2;
    int last_i=-1, last_j=-1;

    int count_written = 0;
    int count_all = 0;

    while (sparse_occ_grid_it_next(&git, &key, &val)) {

        if (val<=0)
            continue;

        sparse_occ_grid_idx_to_ijk(_occ_grid_1, key, &i, &j, &k);

        if (sparse_occ_grid_check_filter(_occ_grid_1, i, j))
            continue;

        // Did we already do this column?
        if (i==last_i && j==last_j)
            continue;

        // Get position and project to next occ grid
        sparse_occ_grid_get_xyz(_occ_grid_1, key, &x1, &y1, &z1);
        bool is_tracklet = find_corresponding_col(x1, y1, &x2, &y2);

        // Are either of these positions out of camera view? If so, we can't rely on
        // the tracklets, so we shouldn't use this data
        if (!in_camera_view(x1, y1, 0) || !in_camera_view(x2, y2, 0))
            continue;

        sparse_occ_grid_get_ijk(_occ_grid_2, x2, y2, 0, &i2, &j2, &k2);

        int range = _uv_window;
        int ss_size = (2*range+1) * (2*range + 1);

        image_roi_t *roi1 = image_roi_grid_get_at(_roi_grid_1, i, j);

        // Loop over search space
        for (int di=-range; di<=range; di++) {
            for (int dj=-range; dj<=range; dj++) {

                count_all++;

                int match = (di==0) && (dj==0);

                if (!match) {
                    // Need to do some kind of random filtering, otherwise we get WAY
                    // too much data
                    int r = rand();
                    if (r % (ss_size-1) != 0)
                        continue;
                }

                // Check roi2
                image_roi_t *roi2 = image_roi_grid_get_at(_roi_grid_2, i2 + di, j2 + dj);
                if (roi2 == NULL) {
                    continue;
                }

                // Write out i, j column
                for (int k_it = 0; k_it<_occ_grid_1->n_dim[2]; k_it++) {
                    float log_prob = sparse_occ_grid_lookup(_occ_grid_1, i, j, k_it);
                    float prob = 1.0f - 1.0f / (1+expf(log_prob));

                    fwrite(&prob, 1, sizeof(prob), fp);
                    if (is_tracklet && fp_t != NULL) {
                        fwrite(&prob, 1, sizeof(prob), fp_t);
                    }
                }

                // Write out image ROI
                fwrite(roi1->data, roi1->width*roi1->height*(roi1->depth+1), sizeof(uint8_t), fp);
                if (is_tracklet && fp_t != NULL) {
                    fwrite(roi1->data, roi1->width*roi1->height*(roi1->depth+1), sizeof(uint8_t), fp_t);
                }

                // Write out potential corresponding column
                for (int k_it = 0; k_it<_occ_grid_2->n_dim[2]; k_it++) {
                    float log_prob = sparse_occ_grid_lookup(_occ_grid_2, i2 + di, j2 + dj, k_it);
                    float prob = 1.0f - 1.0f / (1+expf(log_prob));

                    fwrite(&prob, 1, sizeof(prob), fp);
                    if (is_tracklet && fp_t != NULL) {
                        fwrite(&prob, 1, sizeof(prob), fp_t);
                    }
                }

                // Write out potential corresponding image ROI
                fwrite(roi2->data, roi2->width*roi2->height*(roi2->depth+1), sizeof(uint8_t), fp);
                if (is_tracklet && fp_t != NULL) {
                    fwrite(roi2->data, roi2->width*roi2->height*(roi2->depth+1), sizeof(uint8_t), fp_t);
                }

                // Write out match label
                //fprintf(fp, "%d\n", match);
                fwrite(&match, 1, sizeof(match), fp);
                if (is_tracklet && fp_t != NULL) {
                    //fprintf(fp_t, "%d\n", match);
                    fwrite(&match, 1, sizeof(match), fp_t);
                }

                count_written++;

                image_roi_destroy(roi2);
            }
        }

        image_roi_destroy(roi1);

    } // while

    printf("Wrote %d / %d columns\n", count_written, count_all);

    fclose(fp);
    fclose(fp_t);
}

void
FlowApp::load_instrinsics(FILE *f_cc)
{
    int cam = 2;

    char p_str[50];
    sprintf(p_str, "P_rect_%02d: ", cam);

    char r_str[50];
    //sprintf(r_str, "R_rect_%02d: ", cam);
    sprintf(r_str, "R_rect_%02d: ", 0); // according to kitti paper

    char *line = NULL;
    size_t len;

    double p[12];
    double r[9];

    while (getline(&line, &len, f_cc) != -1) {

        if (strncmp(r_str, line, strlen(r_str)) == 0) {
            sscanf(&line[strlen(r_str)], "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                    &r[0], &r[1], &r[2], &r[3], &r[4], &r[5], &r[6], &r[7], &r[8]);
        } else if (strncmp(p_str, line, strlen(p_str)) == 0) {
            sscanf(&line[strlen(p_str)], "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                    &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7], &p[8], &p[9], &p[10], &p[11]);
        }
    }

    _R_rect.setZero();
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++)
            _R_rect(i, j) = r[i*3 + j];
    }
    _R_rect(3, 3) = 1.0;

    _P_rect.setZero();
    for (int i=0; i<3; i++) {
        for (int j=0; j<4; j++) {
            _P_rect(i, j) = p[i*4 + j];
        }
    }
}

void
FlowApp::load_extrinsics(FILE *f_vc)
{
    char calib_date[100], calib_time[100];
    double R[9];
    double t[3];
    fscanf(f_vc, "calib_time: %s %s\nR: %lf %lf %lf %lf %lf %lf %lf %lf %lf\nT: %lf %lf %lf", calib_date, calib_time,
            &R[0], &R[1], &R[2], &R[3], &R[4], &R[5],
            &R[6], &R[7], &R[8], &t[0], &t[1], &t[2]);

    //printf("Read: %s %s\nR: %f %f %f %f %f %f %f %f %f\nT: %f %f %f\n", calib_date, calib_time,
    //        R[0], R[1], R[2], R[3], R[4], R[5],
    //        R[6], R[7], R[8], t[0], t[1], t[2]);

    _T_cv.setZero();
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++)
            _T_cv(i, j) = R[i*3 + j];
        _T_cv(i, 3) = t[i];
    }
    _T_cv(3, 3) = 1;
}

void
FlowApp::load_calibration(std::string dirname)
{

    FILE *f_cc = fopen( (dirname + "/calib_cam_to_cam.txt").c_str(), "r");
    FILE *f_vc = fopen( (dirname + "/calib_velo_to_cam.txt").c_str(), "r");

    load_instrinsics(f_cc);
    load_extrinsics(f_vc);

    _calib_loaded = true;

    fclose(f_cc);
    fclose(f_vc);

    std::cout << "P_rect: " << std::endl << _P_rect << std::endl;
    std::cout << "R_rect: " << std::endl << _R_rect << std::endl;
    std::cout << "T_cv: " << std::endl << _T_cv << std::endl;

    Eigen::MatrixXd M = _P_rect * _R_rect * _T_cv;
    image_roi_grid_builder_set_M(_rg_builder, M);

    if (_vw) {
        _vw->set_image_calib(_P_rect, _R_rect, _T_cv);
    }
}

void
FlowApp::set_velodyne_returns(std::vector<velodyne_returns_t*> vrs)
{
    _vrs = vrs;

    _scan_at = 0;
    update_scan(_scan_at, false);
}

void
FlowApp::set_poses(std::vector<Pose> poses)
{
    _poses = poses;

    if (_vw) {
        _vw->set_sm_poses(_poses);
    }
}

void
FlowApp::set_images(std::vector<osg::ref_ptr<osg::Image> > imgs)
{
    _images = imgs;

    if (_vw) {
        _vw->set_image(imgs[_scan_at], imgs[_scan_at+1]);
    }
}

void
FlowApp::set_raw_poses(std::vector<Pose> raw_poses)
{
    _raw_poses = raw_poses;

    StateFilter sf;
    sf.init(_raw_poses[0]);
    _filtered_poses.push_back(sf);

    StateFilter sf_gen;
    sf_gen.init(_raw_poses[0]);
    _filtered_poses_full.push_back(sf.get_pose());

    for (int i=1; i<_raw_poses.size(); i++) {
        sf_gen.run_process_model(0.1);
        sf_gen.pose_observation(_raw_poses[i]);
        _filtered_poses_full.push_back(sf_gen.get_pose());
    }

    if (_vw) {
        _vw->set_raw_poses(_raw_poses);
        _vw->set_filtered_poses(_filtered_poses_full);
    }
}

void
FlowApp::set_tracklets(Tracklets *t)
{
    _tracklets = t;

    if (_vw) {
        _vw->set_tracklets(_tracklets);
        _vw->set_frame(_scan_at);
    }
}

const velodyne_returns_t*
FlowApp::get_vr(int i)
{
    return _vrs[i];
}
