#include "thirdparty/perls-math/ssc.h"

#include "pose_solver.hpp"

PoseSolver::PoseSolver(std::vector<velodyne_returns_t*> vrs, std::vector<Pose> poses) :
    _vrs(vrs),
    _poses(poses),
    _sm()
{

}

PoseSolver::~PoseSolver()
{

}

std::vector<Pose>
PoseSolver::solve()
{
    // Set up scan matching
    int sm_pending = 0;

    for (int i=0; i<_vrs.size()-1; i++) {

        velodyne_returns_t *vr1 = velodyne_returns_copy(_vrs[i]);
        velodyne_returns_t *vr2 = velodyne_returns_copy(_vrs[i+1]);

        //printf("Scan matching %d points to %d points\n", vr1->num_returns, vr2->num_returns);

        sm_data_t *data = (sm_data_t*) malloc(sizeof(sm_data_t));
        data->i = i;

        // Estimate transformation
        Pose p1 = _poses[i];
        Pose p2 = _poses[i+1];

        double x1[6] = {p1.x, p1.y, p1.z, p1.r, p1.p, p1.h};
        double x2[6] = {p2.x, p2.y, p2.z, p2.r, p2.p, p2.h};

        double x_21[6];
        ssc_tail2tail(x_21, NULL, x1, x2);

        Eigen::Matrix<double, 6, 1> x_21_est(x_21);

        _sm.push_task(vr2, vr1, data, x_21_est);
        sm_pending++;
    }

    // Wait for scan matching results

    ScanMatch3dData *smd_out;
    std::vector<Eigen::Matrix<double, 6, 1> > rel_pose(_vrs.size());

    while (sm_pending>0) {

        _sm.pop_result(&smd_out);

        sm_data_t *data = (sm_data_t*) smd_out->data;

        rel_pose[data->i] = smd_out->x_ji_res;

        //printf("Result for %d is:\n", data->i);
        //printf("\tEst: %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        //        smd_out->x_ji_est[0], smd_out->x_ji_est[1], smd_out->x_ji_est[2], smd_out->x_ji_est[3], smd_out->x_ji_est[4], smd_out->x_ji_est[5]);
        //printf("\tRes: %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
        //        smd_out->x_ji_res[0], smd_out->x_ji_res[1], smd_out->x_ji_res[2], smd_out->x_ji_res[3], smd_out->x_ji_res[4], smd_out->x_ji_res[5]);

        delete smd_out;
        free(data);

        sm_pending--;
    }

    // Now recompute poses

    for (int i=1; i<_poses.size(); i++) {

        // Get previous pose
        Pose p = _poses[i-1];
        double x[6] = {p.x, p.y, p.z, p.r, p.p, p.h};

        // Get relative pose
        Eigen::Matrix<double, 6, 1> x_ij = rel_pose[i-1];

        // Apply ssc
        double res[6];
        ssc_head2tail(res, NULL, x, x_ij.data());

        // Update pose
        _poses[i].x = res[0];
        _poses[i].y = res[1];
        _poses[i].z = res[2];
        _poses[i].r = res[3];
        _poses[i].p = res[4];
        _poses[i].h = res[5];
    }

    //printf("Done!\n");

    return _poses;
}
