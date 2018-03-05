#ifndef _FLOW_VIEWER_HPP_
#define _FLOW_VIEWER_HPP_

// OSG
#include <osg/MatrixTransform>

// Qt
#include <QMainWindow>
#include <QProgressBar>
#include <QApplication>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>

// local
#include "library/viewer/qt/action_button.hpp"
#include "library/viewer/qt/viewer_widget.hpp"

#include "library/sensors/velodyne.h"

#include "library/kitti/tracklets.hpp"
#include "library/kitti/nodes/kitti_pose.hpp"
#include "library/kitti/nodes/kitti_velodyne.hpp"
#include "library/kitti/nodes/occ_grid.hpp"
#include "library/kitti/nodes/osg_tracklets.hpp"

#include "app/flow/nodes/flow_image_node.hpp"
#include "app/flow/nodes/flow_tracklets_node.hpp"
#include "app/flow/nodes/layered_image.hpp"

#include "app/flow/flow_app.hpp"
#include "library/flow/depth_buffer.h"
#include "library/flow/image_roi_grid.h"

//class QMenu;

// Foward declaration
class FlowApp;
class FlowViewerWindow;

void* fa_thread_routine(void* state_ptr);

// use mutex_fa to read/write this
struct FlowState
{
    enum Status { NO_STATE, DEFAULT };
    enum Command { NONE, PREV_SCAN, REFRESH, NEXT_SCAN, UPDATE_PARAMS, PROJECT_FLOW};

    Status status;
    Command cmd;
    bool ready;
};

// use mutex_fa to read/write this
struct GuiState
{
    QAction* action_prev_scan;
    QAction* action_refresh;
    QAction* action_next_scan;
    QAction* action_update_params;
    QAction* action_project_flow;
    QStatusBar* status_bar;
    QProgressBar* busy_bar;

    FlowViewerWindow *fvw;
};

// sent to fa_thread to handle graph builder actions
// and feeding back to interface elements
struct State
{
    FlowApp* fa;

    FlowState f_state;
    GuiState gui_state;

    pthread_mutex_t mutex_fa;

    // condition variable used by main thread to signal that
    // cmd has been set to something other than NONE
    pthread_cond_t cond_cmd;
};

class FlowViewerWindow : public QMainWindow
{
    Q_OBJECT
  public:

    FlowViewerWindow(osg::ArgumentParser& args,
                      QWidget* parent,
                      Qt::WindowFlags f);
    ~FlowViewerWindow();

    int start();

    void set_flow_app(FlowApp* fa)
    { _state.fa = fa; }

    void set_point_cloud(velodyne_returns_t *vr1, velodyne_returns_t *vr2);
    void set_depth_map(const depth_buffer_t *dm1, const depth_buffer_t *dm2);
    void set_image_roi_grid(const image_roi_grid_t *rg1, const image_roi_grid_t *rg2);

    void set_occ_grids(sparse_occ_grid_t *sog_1, sparse_occ_grid_t *sog_2);
    void set_occ_grid_proj(sparse_occ_grid_t *sog);
    void set_occ_grid1_sel_pos(double x, double y);
    void set_occ_grid2_sel_pos(double x, double y);
    void set_occ_grid_proj_sel_pos(double x, double y);

    void set_flow_image(sparse_occ_grid_t *sog, flow_image_t *f, double x_12[6]);
    void set_flow_image_sel(double x, double y);

    void set_image_sel_pos(double x, double y);
    void set_image_calib(Eigen::MatrixXd P_rect, Eigen::MatrixXd R_rect, Eigen::MatrixXd T_cv);

    void set_flow_tracklets(FlowTracklets *ft) {_flow_tracklets->set_ft(ft);}
    void update_flow_tracklets(sparse_occ_grid_t *sog, double rp[6]) {_flow_tracklets->update(sog, rp);}

    void set_sm_poses(std::vector<Pose> poses)
    { _k_sm_pose->set_poses(poses);}

    void set_raw_poses(std::vector<Pose> poses)
    { _k_raw_pose->set_poses(poses);}

    void set_filtered_poses(std::vector<Pose> poses)
    { _k_filtered_pose->set_poses(poses);}

    void set_tracklets(Tracklets *t)
    { _tracklets->set_tracklets(t);}

    void set_image(osg::ref_ptr<osg::Image> img1, osg::ref_ptr<osg::Image> img2);

    void set_frame(int frame);

    int get_em_iterations()
    {return _spinbox_em_iterations->value();}

    int get_smoothing_weight()
    {return _spinbox_smoothing_weight->value();}

    double get_proj_dt()
    {return _spinbox_proj_dt->value();}

  public slots:

    void slot_prev_scan();
    void slot_refresh();
    void slot_next_scan();
    void slot_update_params();
    void slot_project_flow();
    void slot_cb_vel(bool state);
    void slot_cb_og1(bool state);
    void slot_cb_og2(bool state);
    void slot_cb_og_proj(bool state);
    void slot_cb_f(bool state);
    void slot_cb_ft(bool state);
    void slot_cb_sm_pose(bool state);
    void slot_cb_raw_pose(bool state);
    void slot_cb_filtered_pose(bool state);
    void slot_cb_ego_frame(bool state);
    void slot_cb_forward(bool state);
    void slot_cb_filter(bool state);
    void slot_cb_tracklets(bool state);
    void slot_cb_image_1(bool state);
    void slot_cb_image_2(bool state);
    void slot_cb_image_vel(bool state);
    void slot_cb_image_depth_map(bool state);
    void slot_cb_image_flow(bool state);
    void slot_cb_image_roi(bool state);
    void slot_cb_image_roi_depth_mask(bool state);
    void slot_cleanup();

  private:

    void init(osg::ApplicationUsage* au);

    void create_actions();
    void delete_actions();

    osg::ref_ptr<ViewerWidget> _vwidget;

    QWidget* _main_widget;
    QHBoxLayout* _main_layout;

    QWidget* _panel_widget;
    QVBoxLayout* _panel_layout;

    QMenu* _menu_fa;
    ActionButton* _button_prev_scan;
    ActionButton* _button_refresh;
    ActionButton* _button_next_scan;
    ActionButton* _button_update_params;
    ActionButton* _button_project_flow;

    QSpinBox* _spinbox_em_iterations;
    QDoubleSpinBox* _spinbox_smoothing_weight;
    QDoubleSpinBox* _spinbox_proj_dt;

    State _state;

    pthread_t _fa_thread;      // handles calling FlowApp functions
    pthread_attr_t _attr;

    osg::KittiPose* _k_sm_pose, *_k_raw_pose, *_k_filtered_pose;
    osg::KittiVelodyne* _k_velodyne;
    osg::OccGrid *_occ_grid_1=NULL, *_occ_grid_2=NULL;
    osg::OccGrid *_occ_grid_proj=NULL;
    osg::FlowImage *_flow_image=NULL;
    osg::FlowTrackletsNode *_flow_tracklets=NULL;
    osg::OsgTracklets* _tracklets;
    osg::LayeredImage* _layered_image_1;
    osg::LayeredImage* _layered_image_2;
};

// from osgpick example
// class to handle events with a pick
class PickHandler : public osgGA::GUIEventHandler
{
  public:
    PickHandler(State* state) : _state(state) {};
    ~PickHandler() = default;

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

  private:

    State* _state;
};

class FlowViewer
{
  public:
    FlowViewer(osg::ArgumentParser& args);
    ~FlowViewer();

    void set_flow_app(FlowApp* fa);

    int start()
    {
        int rc = _gvwindow->start();
        if (rc != EXIT_SUCCESS) return rc;
        return _app->exec();
    }

  private:

    // Qt
    QApplication* _app;
    FlowViewerWindow* _gvwindow;
};

#endif // _FLOW_VIEWER_HPP_
