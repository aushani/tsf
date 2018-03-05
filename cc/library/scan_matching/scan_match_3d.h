// adpated from perls/src/segway/slam
#ifndef _SLAM_SCAN_MATCH_3D_H_
#define _SLAM_SCAN_MATCH_3D_H_

#include <glib.h>

#include <Eigen/Core>

//#include "perls-lcmtypes++/perllcm/isam_plink_t.hpp" // replace with something outside of isam
//#include "perls-lcmtypes++/perllcm/isam2_f_pose_pose_t.hpp" // replace with something outside of isam

#define GICP_MAX_ITERS 50
#define GICP_EPS 0.001
#define GICP_D_MAX 3
#define GICP_THREAD_POOL_MAX_THREADS 8
#define GICP_MDIST_THRESH 0.90
#define GICP_DEBUG false

#define ICP_MAX_ITERS 200
#define ICP_D_MAX 3
#define ICP_THREAD_POOL_MAX_THREADS 4
#define ICP_MDIST_THRESH 0.90
#define ICP_DEBUG false

#define NDT_MAX_ITERS 200
#define NDT_D_MAX 2.0
#define NDT_THREAD_POOL_MAX_THREADS 10
#define NDT_MDIST_THRESH 0.90
#define NDT_DEBUG false

//#include "dascar-lcmtypes++/dascarlcm/velodyne_t.hpp"
//#include "dascar-sensors/velodyne.h"
#include "library/sensors/velodyne.h"

using namespace Eigen;

class ScanMatch3dData {

  public:

    velodyne_returns_t *si;
    velodyne_returns_t *sj;

    void *data;

    bool success;

    Matrix<double, 6, 1> x_ji_est;
    Matrix<double, 6, 1> x_ji_res;

    GAsyncQueue *gq;

    ScanMatch3dData (velodyne_returns_t * si_, velodyne_returns_t *sj_, void *data_, GAsyncQueue *gq_) :
        si(si_),
        sj(sj_),
        data(data_),
        success(false),
        gq(gq_) {

    }

    ~ScanMatch3dData (void) {

        velodyne_returns_destroy (si);
        velodyne_returns_destroy (sj);
    }
};

static void
free_scan_match_3d_data (void *smd) {
    delete (ScanMatch3dData *)smd;
}

class ScanMatch3d {

  private:

    GThreadPool *_pool;
    GAsyncQueue *_gq;

    static void*
    scan_match_3d_gicp (void *user);
    //static void*
    //scan_match_3d_icp (void *user);
    //static void*
    //scan_match_3d_ndt (void *user);

  public:

    ScanMatch3d () : _pool(NULL) {
        _gq = g_async_queue_new_full (&free_scan_match_3d_data);

    }

    ~ScanMatch3d () {
        g_async_queue_unref (_gq);
        if (_pool != NULL)
            g_thread_pool_free (_pool, true, false);
    }

    void
    //push_task (perllcm::isam_plink_t plink, velodyne_returns_t * si, velodyne_returns_t *sj);
    push_task (velodyne_returns_t * si, velodyne_returns_t *sj, void *data, Matrix<double, 6, 1> x_ji_est);

    // fills smd_out (or sets it to null)
    // returns 1 on success
    int
    pop_result (ScanMatch3dData** smd_out);

    int
    get_queue_unprocessed ();

};

#endif //_SEG_SCAN_MATCH_3D_H_
