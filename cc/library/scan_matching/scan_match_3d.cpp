// adapted from perls/src/segway/slam
#include <stdio.h>
#include <stdlib.h>

#include "thirdparty/ann/ANN.h"
#include "thirdparty/gicpts/gicp.h"

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_cdf.h>
#include <gsl/gsl_randist.h>

//#include <lcm/lcm-cpp.hpp>

#include "thirdparty/perls-math/so3.h"
#include "thirdparty/perls-common/timestamp.h"

#include "scan_match_3d.h"

#define DTOR UNITS_DEGREE_TO_RADIAN
#define RTOD UNITS_RADIAN_TO_DEGREE

using namespace std;

void
//ScanMatch3d::push_task (perllcm::isam_plink_t plink, velodyne_returns_t * si, velodyne_returns_t *sj) {
ScanMatch3d::push_task (velodyne_returns_t * si, velodyne_returns_t *sj, void *data, Matrix<double, 6, 1> x_ji_est) {

    if (_pool == NULL) {
        _pool = g_thread_pool_new((GFunc)ScanMatch3d::scan_match_3d_gicp, NULL,
                                  GICP_THREAD_POOL_MAX_THREADS, true, NULL);
        //_pool = g_thread_pool_new((GFunc)ScanMatch3d::scan_match_3d_icp, NULL,
        //                          ICP_THREAD_POOL_MAX_THREADS, true, NULL);
        //_pool = g_thread_pool_new((GFunc)ScanMatch3d::scan_match_3d_ndt, NULL,
        //                          NDT_THREAD_POOL_MAX_THREADS, true, NULL);
    }

    //ScanMatch3dData *smd = new ScanMatch3dData (plink, si, sj, _gq);
    ScanMatch3dData *smd = new ScanMatch3dData (si, sj, data, _gq);
    smd->x_ji_est = x_ji_est;
    g_thread_pool_push (_pool, smd, NULL);

}

int
ScanMatch3d::pop_result (ScanMatch3dData **smd_out) {

    //ScanMatch3dData *smd_out = (ScanMatch3dData*)g_async_queue_try_pop (_gq);
    *smd_out = (ScanMatch3dData*)g_async_queue_pop (_gq);
    if (smd_out == NULL) {
        return 0;
    }
    return g_async_queue_length(_gq);
}

int
ScanMatch3d::get_queue_unprocessed () {
    if (_pool == NULL)
        return 0;
    else
        return g_thread_pool_unprocessed (_pool);
}

/*
bool
mdist_check (perllcm::isam_plink_t plink, double x_ji_est[6], double mdist_thresh) {


    gsl_vector_view x_ji_v = gsl_vector_view_array (plink.x_ji.mu, 6);
    gsl_vector_view x_ji_est_v = gsl_vector_view_array (x_ji_est, 6);
    gsl_matrix_view Sigma_v = gsl_matrix_view_array (plink.x_ji.Sigma, 6, 6);
    GSLU_MATRIX_VIEW (invCov, 6, 6);
    gslu_matrix_inv (&invCov.matrix, &Sigma_v.matrix);
    GSLU_INDEX_VIEW (c, 3, {3, 4, 5});
    double dist = gslu_vector_mahal_circ_dist (&x_ji_est_v.vector,
                                               &x_ji_v.vector,
                                               &invCov.matrix,
                                               &c.vector);

    double mdist = gsl_cdf_chisq_Pinv (mdist_thresh, 6);

    if (dist < mdist) {
        cout << "[scan match]\tPassed Mahalanobis distance check (";
        cout << mdist_thresh <<") against prior: " << dist << " < " << mdist << endl;
        return true;
    } else {
        cout << "[scan match]\tFailed Mahalanobis distance check (";
        cout << mdist_thresh <<") against prior: " << dist << " >= " << mdist << endl;
        return false;
    }

}

perllcm::isam2_f_pose_pose_t
build_factor (perllcm::isam_plink_t plink, vector<double> x_ji_est) {

    perllcm::isam2_f_pose_pose_t f_out;

    f_out.utime = timestamp_now();
    f_out.node_id1 = plink.utime_j;
    f_out.node_id2 = plink.utime_i;
    f_out.sub_type = perllcm::isam2_f_pose_pose_t::SUB_TYPE_LASER_3D;
    f_out.n = 6;
    f_out.z = x_ji_est;
    f_out.n2 = 6*6;
    f_out.R.assign(f_out.n2, 0.0);
    f_out.R[0*6+0] = 0.1*0.1;
    f_out.R[1*6+1] = 0.1*0.1;
    f_out.R[2*6+2] = 0.1*0.1;
    f_out.R[3*6+3] = 2.0*2.0*DTOR*DTOR;
    f_out.R[4*6+4] = 2.0*2.0*DTOR*DTOR;
    f_out.R[5*6+5] = 2.0*2.0*DTOR*DTOR;
    
    return f_out;
}

void
lcmgl_draw_result (velodyne_returns_t *si, velodyne_returns_t *sj,
                   double *x_ji_0, double *x_ji_est) {
    
    double ci[3] = {0.0, 0.0, 1.0};
    double ci0[3] = {0.0, 1.0, 1.0};
    double cj[3] = {1.0, 0.0, 0.0};

    // lcmgl debug code
    lcm::LCM lcm;
    bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm.getUnderlyingLCM(), "DEBUG");

    lcmglRotated (180.0, 1.0, 0.0, 0.0);

    // draw j at origin
    lcmglPointSize(2);
    lcmglBegin(LCMGL_POINTS);
    lcmglColor3f(cj[0], cj[1], cj[2]);
    for (int i=0; i<sj->num_returns; i++) {
        lcmglVertex3d(sj->xyz[i*3+0],
                      sj->xyz[i*3+1],
                      sj->xyz[i*3+2]);
    }
    lcmglEnd();

    // draw i at initial guess
    lcmglPushMatrix();
        lcmglTranslated(x_ji_0[0], x_ji_0[1], x_ji_0[2]);
        lcmglRotated(x_ji_0[5]*RTOD, 0, 0, 1);
        lcmglRotated(x_ji_0[4]*RTOD, 0, 1, 0);
        lcmglRotated(x_ji_0[3]*RTOD, 1, 0, 0);

        lcmglPointSize(2);
        lcmglBegin(LCMGL_POINTS);
        lcmglColor3f(ci0[0], ci0[1], ci0[2]);
        for (int i=0; i<si->num_returns; i++) {
            lcmglVertex3d(si->xyz[i*3+0],
                          si->xyz[i*3+1],
                          si->xyz[i*3+2]);
        }
        lcmglEnd();
    lcmglPopMatrix();

    // draw i at resi;t
    lcmglPushMatrix();
        lcmglTranslated(x_ji_est[0], x_ji_est[1], x_ji_est[2]);
        lcmglRotated(x_ji_est[5]*RTOD, 0, 0, 1);
        lcmglRotated(x_ji_est[4]*RTOD, 0, 1, 0);
        lcmglRotated(x_ji_est[3]*RTOD, 1, 0, 0);

        lcmglPointSize(2);
        lcmglBegin(LCMGL_POINTS);
        lcmglColor3f(ci[0], ci[1], ci[2]);
        for (int i=0; i<si->num_returns; i++) {
            lcmglVertex3d(si->xyz[i*3+0],
                          si->xyz[i*3+1],
                          si->xyz[i*3+2]);
        }
        lcmglEnd();
    lcmglPopMatrix();

    bot_lcmgl_switch_buffer(lcmgl);
    bot_lcmgl_destroy(lcmgl);

    timeutil_usleep(1e6);
}
*/



void*
ScanMatch3d::scan_match_3d_gicp (void *user) {

    ScanMatch3dData *smd = (ScanMatch3dData *)user;

    int64_t tic = timestamp_now();

    dgc::gicp::GICPPointSet ps_i, ps_j;
    dgc_transform_t T_ji_0, T_ji_delta, T_ji_est;
    dgc_transform_identity(T_ji_delta);

    // set initial guess
    //double *x_ji_0 = smd->plink.x_ji.mu;
    const double *x_ji_0 = smd->x_ji_est.data();
    dgc_transform_identity (T_ji_0);
    dgc_transform_rotate_x (T_ji_0, x_ji_0[3]);
    dgc_transform_rotate_y (T_ji_0, x_ji_0[4]);
    dgc_transform_rotate_z (T_ji_0, x_ji_0[5]);
    dgc_transform_translate (T_ji_0, x_ji_0[0], x_ji_0[1], x_ji_0[2]);

    int pts_i = 0, pts_j = 0;
    for (int j=0; j<smd->si->num_returns; j++) {
        dgc::gicp::GICPPoint ptmp;
        ptmp.x = smd->si->xyz[3*j+0];
        ptmp.y = smd->si->xyz[3*j+1];
        ptmp.z = smd->si->xyz[3*j+2];
        ps_i.AppendPoint(ptmp);
        pts_i++;
    }
    for (int j=0; j<smd->sj->num_returns; j++) {
        dgc::gicp::GICPPoint ptmp;
        ptmp.x = smd->sj->xyz[3*j+0];
        ptmp.y = smd->sj->xyz[3*j+1];
        ptmp.z = smd->sj->xyz[3*j+2];
        ps_j.AppendPoint(ptmp);
        pts_j++;
    }
    //cout << "\tnum points: " << pts_i << " and " << pts_j << endl;

    //g_mutex_lock(smd->mutex);
    ps_i.SetGICPEpsilon(GICP_EPS);
    ps_j.SetGICPEpsilon(GICP_EPS);
    ps_i.BuildKDTree();
    ps_j.BuildKDTree();
    ps_i.ComputeMatrices();
    ps_j.ComputeMatrices();
    //g_mutex_unlock(smd->mutex);

    // align the point clouds
    ps_j.SetDebug(GICP_DEBUG);
    ps_j.SetMaxIterationInner(8);
    ps_j.SetMaxIteration(GICP_MAX_ITERS);
    //g_mutex_lock(smd->mutex);
    int iterations = ps_j.AlignScan(&ps_i, T_ji_0, T_ji_delta, GICP_D_MAX);
    //g_mutex_unlock(smd->mutex);

    if(GICP_DEBUG) {
        dgc_transform_print(T_ji_0, "T_ji_0");
        dgc_transform_print(T_ji_delta, "T_ji_delta");
        dgc_transform_print(T_ji_est, "T_ji_est");
    }

    dgc_transform_copy(T_ji_est, T_ji_0);
    dgc_transform_left_multiply(T_ji_est, T_ji_delta);

    // decompose the result
    vector<double> x_ji_est;
    x_ji_est.resize(6);
    x_ji_est[0] = T_ji_est[0][3];
    x_ji_est[1] = T_ji_est[1][3];
    x_ji_est[2] = T_ji_est[2][3];
    double R_ij_est[9] = {T_ji_est[0][0], T_ji_est[0][1], T_ji_est[0][2],
                          T_ji_est[1][0], T_ji_est[1][1], T_ji_est[1][2],
                          T_ji_est[2][0], T_ji_est[2][1], T_ji_est[2][2]};
    so3_rot2rph (R_ij_est, &(x_ji_est[3]));

    //smd->f_out = build_factor (smd->plink, x_ji_est);

    smd->success = false;
    if (iterations < GICP_MAX_ITERS) {
        cout << "[scan match]\tGICP converged in " << iterations << " iterations.";
        cout << " (" << (timestamp_now() - tic)/1e6 << "s)" << endl;
        smd->success = true;
        smd->x_ji_res << x_ji_est[0], x_ji_est[1], x_ji_est[2], x_ji_est[3], x_ji_est[4], x_ji_est[5];
        //if (mdist_check (smd->plink, &(x_ji_est[0]), GICP_MDIST_THRESH)) {
        //    smd->success = true;
        //    if (GICP_DEBUG) {
        //        lcmgl_draw_result (smd->si, smd->sj, smd->plink.x_ji.mu, &x_ji_est[0]);
        //    }
        //}
    } else {
        smd->x_ji_est << 0, 0, 0, 0, 0, 0;

        cout << "[scan match]\tGICP failed to converge in " << iterations << " iterations.";
        cout << " (" << (timestamp_now() - tic)/1e6 << "s)" << endl;
    }

    g_async_queue_push (smd->gq, smd);

    return NULL;

}

/*
void*
ScanMatch3d::scan_match_3d_icp (void *user) {

    ScanMatch3dData *smd = (ScanMatch3dData *)user;

    int64_t tic = timestamp_now();

    // put scans into pcl format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_i->width    = smd->si->num_returns;
    cloud_i->height   = 1;
    cloud_i->is_dense = false;
    cloud_i->points.resize (cloud_i->width * cloud_i->height);
    for (size_t i = 0; i < cloud_i->points.size (); ++i) {
        cloud_i->points[i].x = smd->si->xyz[3*i+0];
        cloud_i->points[i].y = smd->si->xyz[3*i+1];
        cloud_i->points[i].z = smd->si->xyz[3*i+2];
    }

    cloud_j->width    = smd->sj->num_returns;
    cloud_j->height   = 1;
    cloud_j->is_dense = false;
    cloud_j->points.resize (cloud_j->width * cloud_j->height);
    for (size_t i = 0; i < cloud_j->points.size (); ++i) {
        cloud_j->points[i].x = smd->sj->xyz[3*i+0];
        cloud_j->points[i].y = smd->sj->xyz[3*i+1];
        cloud_j->points[i].z = smd->sj->xyz[3*i+2];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ji (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_i);
    icp.setInputTarget(cloud_j);

    // Set the max correspondence distance  (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (ICP_D_MAX);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (ICP_MAX_ITERS);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    // set initial guess
    double *x_ji_0 = smd->plink.x_ji.mu;
    double R_ij[9] = {0};
    double rph_ji[3] = {x_ji_0[3], x_ji_0[4], x_ji_0[5]};
    so3_rotxyz (R_ij, rph_ji);
    Eigen::Matrix4f T_ji;
    T_ji << R_ij[0], R_ij[1], R_ij[2], x_ji_0[0],
            R_ij[3], R_ij[4], R_ij[5], x_ji_0[1],
            R_ij[6], R_ij[7], R_ij[8], x_ji_0[2],
            0,       0,       0,       1;

    icp.align(*cloud_ji, T_ji);

    if (ICP_DEBUG) {
        cout << "[icp]\t\tT_ji_0:" << endl << T_ji << endl;
        cout << "[icp]\t\tICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
        cout << "[icp]\t\tT_ji_est:" << endl <<  icp.getFinalTransformation() << endl;
    }

    // decompose the result
    vector<double> x_ji_est;
    x_ji_est.resize(6);
    Eigen::Matrix4f T_ji_est = icp.getFinalTransformation();
    x_ji_est[0] = T_ji_est(0,3);
    x_ji_est[1] = T_ji_est(1,3);
    x_ji_est[2] = T_ji_est(2,3);
    double R_ij_est[9] = {T_ji_est(0,0), T_ji_est(0,1), T_ji_est(0,2),
                          T_ji_est(1,0), T_ji_est(1,1), T_ji_est(1,2),
                          T_ji_est(2,0), T_ji_est(2,1), T_ji_est(2,2)};
    so3_rot2rph (R_ij_est, &x_ji_est[3]);

    smd->f_out = build_factor (smd->plink, x_ji_est);

    smd->success = false;
    if (icp.hasConverged()) {
        cout << "[scan match]\tICP converged in " << ICP_MAX_ITERS << " iterations.";
        cout << " (" << (timestamp_now() - tic)/1e6 << "s)" << endl;
        if (mdist_check (smd->plink, &(x_ji_est[0]), ICP_MDIST_THRESH)) {
            smd->success = true;
            if (ICP_DEBUG) {
                lcmgl_draw_result (smd->si, smd->sj, smd->plink.x_ji.mu, &x_ji_est[0]);
            }
        }
    } else {
        cout << "[scan match]\tICP failed to converge in " << ICP_MAX_ITERS << " iterations.";
        cout << " (" << (timestamp_now() - tic)/1e6 << "s)" << endl;
    }

    g_async_queue_push (smd->gq, smd);

    return NULL;
}
*/

/*
void*
ScanMatch3d::scan_match_3d_ndt (void *user) {

    ScanMatch3dData *smd = (ScanMatch3dData *)user;

    int64_t tic = timestamp_now();

    // put scans into pcl format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_i->width    = smd->si->num_returns;
    cloud_i->height   = 1;
    cloud_i->is_dense = false;
    cloud_i->points.resize (cloud_i->width * cloud_i->height);
    for (size_t i = 0; i < cloud_i->points.size (); ++i) {
        cloud_i->points[i].x = smd->si->xyz[3*i+0];
        cloud_i->points[i].y = smd->si->xyz[3*i+1];
        cloud_i->points[i].z = smd->si->xyz[3*i+2];
    }

    cloud_j->width    = smd->sj->num_returns;
    cloud_j->height   = 1;
    cloud_j->is_dense = false;
    cloud_j->points.resize (cloud_j->width * cloud_j->height);
    for (size_t i = 0; i < cloud_j->points.size (); ++i) {
        cloud_j->points[i].x = smd->sj->xyz[3*i+0];
        cloud_j->points[i].y = smd->sj->xyz[3*i+1];
        cloud_j->points[i].z = smd->sj->xyz[3*i+2];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ji (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setInputSource(cloud_i);
    ndt.setInputTarget(cloud_j);

    // Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (NDT_D_MAX);
    // Set the maximum number of iterations (criterion 1)
    ndt.setMaximumIterations (NDT_MAX_ITERS);
    // Set the transformation epsilon (criterion 2)
    ndt.setTransformationEpsilon (0.01);
    ndt.setStepSize (0.1);

    // set initial guess
    double *x_ji_0 = smd->plink.x_ji.mu;
    double R_ij[9] = {0};
    double rph_ji[3] = {x_ji_0[3], x_ji_0[4], x_ji_0[5]};
    so3_rotxyz (R_ij, rph_ji);
    Eigen::Matrix4f T_ji;
    T_ji << R_ij[0], R_ij[1], R_ij[2], x_ji_0[0],
            R_ij[3], R_ij[4], R_ij[5], x_ji_0[1],
            R_ij[6], R_ij[7], R_ij[8], x_ji_0[2],
            0,       0,       0,       1;

    ndt.align(*cloud_ji, T_ji);

    if (NDT_DEBUG) {
        cout << "[NDT]\t\tT_ji_0:" << endl << T_ji << endl;
        cout << "[NDT]\t\tNDT has converged:" << ndt.hasConverged() << " score: " << ndt.getFitnessScore() << endl;
        cout << "[NDT]\t\tT_ji_est:" << endl <<  ndt.getFinalTransformation() << endl;
    }

    // decompose the result
    vector<double> x_ji_est;
    x_ji_est.resize(6);
    Eigen::Matrix4f T_ji_est = ndt.getFinalTransformation();
    x_ji_est[0] = T_ji_est(0,3);
    x_ji_est[1] = T_ji_est(1,3);
    x_ji_est[2] = T_ji_est(2,3);
    double R_ij_est[9] = {T_ji_est(0,0), T_ji_est(0,1), T_ji_est(0,2),
                          T_ji_est(1,0), T_ji_est(1,1), T_ji_est(1,2),
                          T_ji_est(2,0), T_ji_est(2,1), T_ji_est(2,2)};
    so3_rot2rph (R_ij_est, &x_ji_est[3]);

    smd->f_out = build_factor (smd->plink, x_ji_est);

    smd->success = false;
    if (ndt.hasConverged()) {
        cout << "[scan match]\tNDT converged in " << NDT_MAX_ITERS << " iterations.";
        cout << " (" << (timestamp_now() - tic)/1e6 << "s)" << endl;
        if (mdist_check (smd->plink, &(x_ji_est[0]), NDT_MDIST_THRESH)) {
            smd->success = true;
            if (NDT_DEBUG) {
                lcmgl_draw_result (smd->si, smd->sj, smd->plink.x_ji.mu, &x_ji_est[0]);
            }
        }
    } else {
        cout << "[scan match]\tNDT failed to converge in " << NDT_MAX_ITERS << " iterations.";
        cout << " (" << (timestamp_now() - tic)/1e6 << "s)" << endl;
    }

    g_async_queue_push (smd->gq, smd);

    return NULL;
}
*/
