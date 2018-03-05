#include <pthread.h>
#include <string>
#include <iostream>

// OSG
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/TerrainManipulator>
#include <osgUtil/Optimizer>
#include <osgViewer/ViewerEventHandlers>
#include <osg/MatrixTransform>
#include <osg/Geode>
#include <osg/Projection>

// Qt
#include <QStatusBar>
#include <QAction>
#include <QLabel>
//#include <QMenuBar>

// perls
#include "perls-common/units.h"

// local
#include "library/viewer/util/utils.hpp"
#include "library/viewer/nodes/axes.hpp"
#include "library/viewer/nodes/car.hpp"
#include "library/viewer/ga/terrain_trackpad_manipulator.hpp"

#include "app/flow/viewer.hpp"

class QAction;

void* fa_thread_routine(void* state_ptr)
{
    State* state = (State*) state_ptr;
    FlowState* f_state = &state->f_state;
    GuiState* gui_state = &state->gui_state;

    pthread_mutex_lock(&state->mutex_fa);

    while(true) {
        // by next unlock, we will have started cond_wait
        f_state->ready = true;

        // wait for a command
        while(f_state->cmd == FlowState::Command::NONE) {
            // cond_wait will unlock automatically until signal
            pthread_cond_wait(&state->cond_cmd, &state->mutex_fa);
        }

        f_state->ready = false;

        switch(f_state->cmd) {

            case FlowState::Command::PREV_SCAN:
                // need to use this because qt slots (like showMessage() of
                // QStatusBar) cannot be otherwise called in other threads
                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Going to next scan..."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, true));

                pthread_mutex_unlock(&state->mutex_fa);
                state->fa->prev_scan(); // blocks here
                pthread_mutex_lock(&state->mutex_fa);

                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Done."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, false));

                f_state->status = FlowState::Status::DEFAULT;
                f_state->cmd = FlowState::Command::NONE;
                // don't unlock: need to get back to cond_wait

                break;

            case FlowState::Command::NEXT_SCAN:
                // need to use this because qt slots (like showMessage() of
                // QStatusBar) cannot be otherwise called in other threads
                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Going to next scan..."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, true));

                pthread_mutex_unlock(&state->mutex_fa);
                state->fa->next_scan(); // blocks here
                pthread_mutex_lock(&state->mutex_fa);

                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Done."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, false));

                f_state->status = FlowState::Status::DEFAULT;
                f_state->cmd = FlowState::Command::NONE;
                // don't unlock: need to get back to cond_wait

                break;

            case FlowState::Command::UPDATE_PARAMS:
                // need to use this because qt slots (like showMessage() of
                // QStatusBar) cannot be otherwise called in other threads
                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Updating parameters..."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, true));

                pthread_mutex_unlock(&state->mutex_fa);

                state->fa->set_em_iterations(state->gui_state.fvw->get_em_iterations());
                state->fa->set_smoothing_weight(state->gui_state.fvw->get_smoothing_weight());

                pthread_mutex_lock(&state->mutex_fa);

                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Done."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, false));

                f_state->status = FlowState::Status::DEFAULT;
                f_state->cmd = FlowState::Command::NONE;
                // don't unlock: need to get back to cond_wait

                break;

            case FlowState::Command::PROJECT_FLOW:
                // need to use this because qt slots (like showMessage() of
                // QStatusBar) cannot be otherwise called in other threads
                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Projecting flow..."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, true));

                pthread_mutex_unlock(&state->mutex_fa);
                state->fa->project_flow(state->gui_state.fvw->get_proj_dt());
                pthread_mutex_lock(&state->mutex_fa);

                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Done."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, false));

                f_state->status = FlowState::Status::DEFAULT;
                f_state->cmd = FlowState::Command::NONE;
                // don't unlock: need to get back to cond_wait

                break;

            case FlowState::Command::REFRESH:
                // need to use this because qt slots (like showMessage() of
                // QStatusBar) cannot be otherwise called in other threads
                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Refreshing..."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, true));

                pthread_mutex_unlock(&state->mutex_fa);

                // Update parameters
                state->fa->set_em_iterations(state->gui_state.fvw->get_em_iterations());
                state->fa->set_smoothing_weight(state->gui_state.fvw->get_smoothing_weight());

                state->fa->refresh();

                pthread_mutex_lock(&state->mutex_fa);

                QMetaObject::invokeMethod(gui_state->status_bar,
                                          "showMessage",
                                          Qt::QueuedConnection,
                                          Q_ARG(QString, "Done."));
                QMetaObject::invokeMethod(gui_state->busy_bar,
                                          "setVisible",
                                          Qt::QueuedConnection,
                                          Q_ARG(bool, false));

                f_state->status = FlowState::Status::DEFAULT;
                f_state->cmd = FlowState::Command::NONE;
                // don't unlock: need to get back to cond_wait

                break;


            default:
                // don't unlock: need to get back to cond_wait
                break;
        }
    }

    pthread_exit(nullptr);
}

FlowViewerWindow::FlowViewerWindow(osg::ArgumentParser& args,
                                     QWidget* parent,
                                     Qt::WindowFlags f) :
        QMainWindow(parent, f),
        _k_sm_pose(new osg::KittiPose()),
        _k_raw_pose(new osg::KittiPose()),
        _k_filtered_pose(new osg::KittiPose()),
        _k_velodyne(new osg::KittiVelodyne()),
        _occ_grid_1(new osg::OccGrid()),
        _occ_grid_2(new osg::OccGrid()),
        _occ_grid_proj(new osg::OccGrid()),
        _flow_image(new osg::FlowImage()),
        _flow_tracklets(new osg::FlowTrackletsNode()),
        _tracklets(new osg::OsgTracklets()),
        _layered_image_1(new osg::LayeredImage(1242, 375, 0, 0.67*375, 0.67)),
        _layered_image_2(new osg::LayeredImage(1242, 375, 0, 0, 0.67))
{
    osgViewer::ViewerBase::ThreadingModel tm =
            ViewerUtils::setup_qt_threading(args);

    _vwidget = new ViewerWidget(0, Qt::Widget, tm);

    create_actions();

    // setup GUI
    _main_widget = new QWidget;
    _main_layout = new QHBoxLayout;
    _panel_widget = new QWidget;
    _panel_layout = new QVBoxLayout;

    _main_layout->setContentsMargins(0, 0, 0, 1); // no border
    _panel_layout->setContentsMargins(3, 3, 3, 3);

    _main_widget->setLayout(_main_layout);
    _main_layout->addWidget(_panel_widget);
    _main_layout->addWidget(_vwidget);
    _main_layout->setSpacing(0);

    _panel_widget->setLayout(_panel_layout);
    _panel_layout->addWidget(_button_prev_scan);
    _panel_layout->addWidget(_button_refresh);
    _panel_layout->addWidget(_button_next_scan);
    _panel_layout->addWidget(_button_update_params);
    _panel_layout->addWidget(_button_project_flow);

    QLabel *label_em_iterations = new QLabel("Iterations");
    _panel_layout->addWidget(label_em_iterations);
    _panel_layout->addWidget(_spinbox_em_iterations);

    QLabel *label_smoothing_weight = new QLabel("Smoothing Weight");
    _panel_layout->addWidget(label_smoothing_weight);
    _panel_layout->addWidget(_spinbox_smoothing_weight);

    QLabel *label_proj_dt = new QLabel("Projected dt");
    _panel_layout->addWidget(label_proj_dt);
    _panel_layout->addWidget(_spinbox_proj_dt);

    QCheckBox *cb_forward = new QCheckBox("Forward Flow");
    cb_forward->setChecked(true);
    _panel_layout->addWidget(cb_forward);
    connect(cb_forward, SIGNAL(clicked(bool)), this, SLOT(slot_cb_forward(bool)));

    QCheckBox *cb_filter = new QCheckBox("Use Filter");
    cb_filter->setChecked(true);
    _panel_layout->addWidget(cb_filter);
    connect(cb_filter, SIGNAL(clicked(bool)), this, SLOT(slot_cb_filter(bool)));

    QLabel *label_render = new QLabel("Rendering Options");
    _panel_layout->addWidget(label_render);

    QCheckBox *cb_vel = new QCheckBox("Velodyne");
    cb_vel->setChecked(true);
    _panel_layout->addWidget(cb_vel);
    connect(cb_vel, SIGNAL(clicked(bool)), this, SLOT(slot_cb_vel(bool)));

    QCheckBox *cb_og1 = new QCheckBox("Occ Grid 1");
    cb_og1->setChecked(true);
    _panel_layout->addWidget(cb_og1);
    connect(cb_og1, SIGNAL(clicked(bool)), this, SLOT(slot_cb_og1(bool)));

    QCheckBox *cb_og2 = new QCheckBox("Occ Grid 2");
    cb_og2->setChecked(true);
    _panel_layout->addWidget(cb_og2);
    connect(cb_og2, SIGNAL(clicked(bool)), this, SLOT(slot_cb_og2(bool)));

    QCheckBox *cb_og_proj = new QCheckBox("Occ Grid Proj");
    cb_og_proj->setChecked(true);
    _panel_layout->addWidget(cb_og_proj);
    connect(cb_og_proj, SIGNAL(clicked(bool)), this, SLOT(slot_cb_og_proj(bool)));

    QCheckBox *cb_f = new QCheckBox("Flow");
    cb_f->setChecked(true);
    _panel_layout->addWidget(cb_f);
    connect(cb_f, SIGNAL(clicked(bool)), this, SLOT(slot_cb_f(bool)));

    QCheckBox *cb_ft = new QCheckBox("Flow Tracklets");
    cb_ft->setChecked(true);
    _panel_layout->addWidget(cb_ft);
    connect(cb_ft, SIGNAL(clicked(bool)), this, SLOT(slot_cb_ft(bool)));

    QCheckBox *cb_sm_pose = new QCheckBox("SM Pose");
    cb_sm_pose->setChecked(false);
    _panel_layout->addWidget(cb_sm_pose);
    connect(cb_sm_pose, SIGNAL(clicked(bool)), this, SLOT(slot_cb_sm_pose(bool)));
    _k_sm_pose->setNodeMask(false);

    QCheckBox *cb_raw_pose = new QCheckBox("Raw Pose");
    cb_raw_pose->setChecked(false);
    _panel_layout->addWidget(cb_raw_pose);
    connect(cb_raw_pose, SIGNAL(clicked(bool)), this, SLOT(slot_cb_raw_pose(bool)));
    _k_raw_pose->setNodeMask(false);

    QCheckBox *cb_filtered_pose = new QCheckBox("Filtered Pose");
    cb_filtered_pose->setChecked(false);
    _panel_layout->addWidget(cb_filtered_pose);
    connect(cb_filtered_pose, SIGNAL(clicked(bool)), this, SLOT(slot_cb_filtered_pose(bool)));
    _k_filtered_pose->setNodeMask(false);

    QCheckBox *cb_ego_frame = new QCheckBox("Ego Frame");
    cb_ego_frame->setChecked(false);
    _panel_layout->addWidget(cb_ego_frame);
    connect(cb_ego_frame, SIGNAL(clicked(bool)), this, SLOT(slot_cb_ego_frame(bool)));
    _flow_image->set_render_in_ego_frame(false);
    _flow_tracklets->set_render_in_ego_frame(false);

    QCheckBox *cb_tracklets = new QCheckBox("Tracklets");
    cb_tracklets->setChecked(false);
    _panel_layout->addWidget(cb_tracklets);
    connect(cb_tracklets, SIGNAL(clicked(bool)), this, SLOT(slot_cb_tracklets(bool)));
    _tracklets->setNodeMask(false);

    QLabel *label_image = new QLabel("Image Options");
    _panel_layout->addWidget(label_image);

    QCheckBox *cb_im1 = new QCheckBox("Image 1");
    cb_im1->setChecked(true);
    _panel_layout->addWidget(cb_im1);
    connect(cb_im1, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_1(bool)));

    QCheckBox *cb_im2 = new QCheckBox("Image 2");
    cb_im2->setChecked(true);
    _panel_layout->addWidget(cb_im2);
    connect(cb_im2, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_2(bool)));


    QCheckBox *cb_im_vel = new QCheckBox("Image Velodyne");
    cb_im_vel->setChecked(false);
    _panel_layout->addWidget(cb_im_vel);
    connect(cb_im_vel, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_vel(bool)));

    QCheckBox *cb_im_dm = new QCheckBox("Image Depth Map");
    cb_im_dm->setChecked(false);
    _panel_layout->addWidget(cb_im_dm);
    connect(cb_im_dm, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_depth_map(bool)));

    QCheckBox *cb_im_flow = new QCheckBox("Image Flow");
    cb_im_flow->setChecked(false);
    _panel_layout->addWidget(cb_im_flow);
    connect(cb_im_flow, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_flow(bool)));

    QCheckBox *cb_im_roi = new QCheckBox("Image ROI");
    cb_im_roi->setChecked(false);
    _panel_layout->addWidget(cb_im_roi);
    connect(cb_im_roi, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_roi(bool)));

    QCheckBox *cb_im_roi_dm = new QCheckBox("Image ROI Depth Mask");
    cb_im_roi_dm->setChecked(false);
    _panel_layout->addWidget(cb_im_roi_dm);
    connect(cb_im_roi_dm, SIGNAL(clicked(bool)), this, SLOT(slot_cb_image_roi_depth_mask(bool)));

    _panel_widget->setMaximumWidth(200);
    _panel_layout->setAlignment(Qt::AlignTop);

    setCentralWidget(_main_widget);

    setWindowTitle(tr("KITTI Flow"));
    setMinimumSize(760, 1014);

    init(args.getApplicationUsage());

    // set up f_state
    _state.fa = nullptr;
    _state.f_state.status = FlowState::Status::NO_STATE;
    _state.f_state.cmd = FlowState::Command::NONE; // build on start
    _state.f_state.ready = false;

    // set up fa and mutex
    pthread_attr_init(&_attr);
    pthread_attr_setdetachstate(&_attr, PTHREAD_CREATE_JOINABLE);
    pthread_mutex_init(&_state.mutex_fa, nullptr);
    pthread_cond_init(&_state.cond_cmd, nullptr);

    _occ_grid_1->set_color(osg::Vec4(43/255.0, 131/255.0, 186/255.0, 1.0));
    //_occ_grid_2->set_color(osg::Vec4(171/255.0, 221/255.0, 164/255.0, 1.0));
    _occ_grid_2->set_color(osg::Vec4(171/255.0, 221/255.0, 164/255.0, 0.5));
    _occ_grid_proj->set_color(osg::Vec4(255/255.0, 0/255.0, 0/255.0, 0.5));
    _flow_image->set_color(osg::Vec4(253/255.0, 174.0/255.0, 97/255.0, 8.8));
    _flow_tracklets->set_color(osg::Vec4(215/255.0, 25/255.0, 28/255.0, 0.8));

}

FlowViewerWindow::~FlowViewerWindow()
{
    pthread_attr_destroy(&_attr);
    pthread_mutex_destroy(&_state.mutex_fa);
    pthread_cond_destroy(&_state.cond_cmd);

    delete_actions();

    delete _k_sm_pose;
    delete _k_raw_pose;
    delete _k_filtered_pose;
    delete _k_velodyne;
    delete _occ_grid_1;
    delete _occ_grid_2;
    delete _occ_grid_proj;
    delete _flow_image;
    delete _flow_tracklets;
    delete _tracklets;
    //delete _camera_proj_matrix;
    //delete _camera;
    //delete _camera_texture;
}

int FlowViewerWindow::start()
{
    show();

    // Make sure everything is scaled correctly
    //osg::ref_ptr<osgViewer::View> view = _vwidget->get_view();
    //int w = view->getCamera()->getViewport()->width();
    //int h = view->getCamera()->getViewport()->height();
    // Not sure why this isn't working right, hard code something that's close
    int w = 758;
    int h = 1014;
    _layered_image_1->update_window_size(w, h);
    _layered_image_2->update_window_size(w, h);

    // start threads
    std::cout << "Starting fa..." << std::endl;
    int rc = pthread_create(&_fa_thread, &_attr,
                        fa_thread_routine,
                        (void *) &_state);
    if (rc) {
        std::cerr << "Error creating fa_thread: " << rc << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void FlowViewerWindow::slot_prev_scan()
{
    pthread_mutex_lock(&_state.mutex_fa);

    if (_state.f_state.ready) {
        _state.f_state.cmd = FlowState::Command::PREV_SCAN;
        pthread_cond_signal(&_state.cond_cmd);
    } else {
        std::cout << "Not ready!" << std::endl;
    }

    pthread_mutex_unlock(&_state.mutex_fa);
}

void FlowViewerWindow::slot_refresh()
{
    pthread_mutex_lock(&_state.mutex_fa);

    if (_state.f_state.ready) {
        _state.f_state.cmd = FlowState::Command::REFRESH;
        pthread_cond_signal(&_state.cond_cmd);
    } else {
        std::cout << "Not ready!" << std::endl;
    }

    pthread_mutex_unlock(&_state.mutex_fa);
}

void FlowViewerWindow::slot_next_scan()
{
    pthread_mutex_lock(&_state.mutex_fa);

    if (_state.f_state.ready) {
        _state.f_state.cmd = FlowState::Command::NEXT_SCAN;
        pthread_cond_signal(&_state.cond_cmd);
    } else {
        std::cout << "Not ready!" << std::endl;
    }

    pthread_mutex_unlock(&_state.mutex_fa);
}

void FlowViewerWindow::slot_update_params()
{
    pthread_mutex_lock(&_state.mutex_fa);

    if (_state.f_state.ready) {
        _state.f_state.cmd = FlowState::Command::UPDATE_PARAMS;
        pthread_cond_signal(&_state.cond_cmd);
    } else {
        std::cout << "Not ready!" << std::endl;
    }

    pthread_mutex_unlock(&_state.mutex_fa);
}

void FlowViewerWindow::slot_project_flow()
{
    pthread_mutex_lock(&_state.mutex_fa);

    if (_state.f_state.ready) {
        _state.f_state.cmd = FlowState::Command::PROJECT_FLOW;
        pthread_cond_signal(&_state.cond_cmd);
    } else {
        std::cout << "Not ready!" << std::endl;
    }

    pthread_mutex_unlock(&_state.mutex_fa);
}

void FlowViewerWindow::slot_cb_vel(bool state)
{
    _k_velodyne->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_og1(bool state)
{
    _occ_grid_1->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_og2(bool state)
{
    _occ_grid_2->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_og_proj(bool state)
{
    _occ_grid_proj->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_f(bool state)
{
    _flow_image->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_ft(bool state)
{
    _flow_tracklets->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_sm_pose(bool state)
{
    _k_sm_pose->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_raw_pose(bool state)
{
    _k_raw_pose->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_filtered_pose(bool state)
{
    _k_filtered_pose->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_ego_frame(bool state)
{
    _flow_image->set_render_in_ego_frame(state);
    _flow_tracklets->set_render_in_ego_frame(state);
}

void FlowViewerWindow::slot_cb_tracklets(bool state)
{
    _tracklets->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_image_1(bool state)
{
    _layered_image_1->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_image_2(bool state)
{
    _layered_image_2->setNodeMask(state);
}

void FlowViewerWindow::slot_cb_image_vel(bool state)
{
    _layered_image_1->render_point_cloud(state);
    _layered_image_2->render_point_cloud(state);
}

void FlowViewerWindow::slot_cb_image_depth_map(bool state)
{
    _layered_image_1->render_depth_map(state);
    _layered_image_2->render_depth_map(state);
}

void FlowViewerWindow::slot_cb_image_flow(bool state)
{
    _layered_image_1->render_flow(state);
    _layered_image_2->render_flow(state);
}

void FlowViewerWindow::slot_cb_image_roi(bool state)
{
    _layered_image_1->render_roi(state);
    _layered_image_2->render_roi(state);
}

void FlowViewerWindow::slot_cb_image_roi_depth_mask(bool state)
{
    _layered_image_1->render_roi_depth_mask(state);
    _layered_image_2->render_roi_depth_mask(state);
}

void FlowViewerWindow::slot_cb_forward(bool state)
{
    _state.fa->set_forward(state);
}

void FlowViewerWindow::slot_cb_filter(bool state)
{
    _state.fa->set_filter(state);
}

void FlowViewerWindow::slot_cleanup()
{
    pthread_mutex_lock(&_state.mutex_fa);

    printf("cleanup\n");

    //if (_state.f_state.ready) {
    //    _state.f_state.cmd = FlowState::Command::STOP;
    //    pthread_cond_signal(&_state.cond_cmd);
    //} else {
    //    std::cout << "Not ready; quitting early!" << std::endl;
    //}

    pthread_mutex_unlock(&_state.mutex_fa);
}

void FlowViewerWindow::init(osg::ApplicationUsage* au)
{
    osg::ref_ptr<osgViewer::View> view = _vwidget->get_view();

    // set background to black
    // TODO: magic numbers
    view->getCamera()->setClearColor(osg::Vec4d(1, 1, 1, 0));

    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> ksm =
            new osgGA::KeySwitchMatrixManipulator();

    ksm->addMatrixManipulator(
        '1', "TerrainTrackpad", new osgGA::TerrainTrackpadManipulator());
    ksm->addMatrixManipulator(
        '3', "Terrain", new osgGA::TerrainManipulator());

    // set initial camera position (for all manipulators)
    // TODO: magic numbers
    ksm->setHomePosition(osg::Vec3d(0, 0, 100),
                         osg::Vec3d(0, 0, 0),
                         osg::Vec3d(1, 0, 0),
                         false);

    ksm->getUsage(*au);
    view->setCameraManipulator(ksm.get());

    // add the state manipulator
    osg::ref_ptr<osgGA::StateSetManipulator> ssm =
            new osgGA::StateSetManipulator(
                view->getCamera()->getOrCreateStateSet());
    ssm->getUsage(*au);
    view->addEventHandler(ssm);

    // add the stats handler
    osg::ref_ptr<osgViewer::StatsHandler> sh =
            new osgViewer::StatsHandler();
    sh->getUsage(*au);
    view->addEventHandler(sh);

    // add the help handler
    osg::ref_ptr<osgViewer::HelpHandler> hh =
            new osgViewer::HelpHandler(au);
    hh->getUsage(*au);
    view->addEventHandler(hh);

    // add the screen capture handler
    osg::ref_ptr<osgViewer::ScreenCaptureHandler> sch =
            new osgViewer::ScreenCaptureHandler();
    sch->getUsage(*au);
    view->addEventHandler(sch);

    // add the level of detail scale selector
    osg::ref_ptr<osgViewer::LODScaleHandler> lod =
            new osgViewer::LODScaleHandler();
    lod->getUsage(*au);
    view->addEventHandler(lod);

    // add the pick handler
    osg::ref_ptr<PickHandler> ph = new PickHandler(&_state);
    ph->getUsage(*au);
    view->addEventHandler(ph);

    // rotate by x until z down
    // car RH coordinate frame has x forward, z down
    const double k_d2r = UNITS_DEGREE_TO_RADIAN;
    //osg::Matrixd H(osg::Quat(180*k_d2r, osg::Vec3d(1,0,0)));
    osg::Matrixd H(osg::Quat(0, osg::Vec3d(1, 0, 0)));
    osg::ref_ptr<osg::MatrixTransform> xform =
            new osg::MatrixTransform(H);

    // add xyz axes at origin
    //xform->addChild(new osg::Axes());
    //xform->addChild(new osg::Car());

    osg::ref_ptr<osg::MatrixTransform> xform_car = new osg::MatrixTransform();
    osg::Matrixd D(osg::Quat(M_PI, osg::Vec3d(1, 0, 0)));
    D.postMultTranslate(osg::Vec3d(-1, 0, -1.2));
    xform_car->setMatrix(D);
    xform_car->addChild(new osg::Car());
    xform->addChild(xform_car);

    // Setup kitti velodyne
    xform->addChild(_k_sm_pose);
    xform->addChild(_k_raw_pose);
    xform->addChild(_k_filtered_pose);
    xform->addChild(_k_velodyne);
    xform->addChild(_occ_grid_1);
    xform->addChild(_occ_grid_2);
    xform->addChild(_occ_grid_proj);
    xform->addChild(_flow_image);
    xform->addChild(_flow_tracklets);
    xform->addChild(_tracklets);

    // For camera display
    xform->addChild(_layered_image_1);
    xform->addChild(_layered_image_2);
    view->addEventHandler(_layered_image_1); // so image resizes correctly
    view->addEventHandler(_layered_image_2); // so image resizes correctly

    // set scene
    view->setSceneData(xform);
}

void FlowViewerWindow::create_actions()
{
    // actions
    _state.gui_state.action_prev_scan = new QAction(tr("&Prev Scan"), this);
    _state.gui_state.action_prev_scan->setShortcut(QKeySequence(tr("Ctrl+P")));
    _state.gui_state.action_prev_scan->setStatusTip(
        tr("Advance to prev scan"));
    _state.gui_state.action_prev_scan->setToolTip(
        tr("Advance to prev scan"));
    connect(_state.gui_state.action_prev_scan,
            SIGNAL(triggered()), this, SLOT(slot_prev_scan()));

    _state.gui_state.action_refresh = new QAction(tr("&Refresh"), this);
    _state.gui_state.action_refresh->setShortcut(QKeySequence(tr("Ctrl+R")));
    _state.gui_state.action_refresh->setStatusTip(
        tr("Refresh result"));
    _state.gui_state.action_refresh->setToolTip(
        tr("Refresh result"));
    connect(_state.gui_state.action_refresh,
            SIGNAL(triggered()), this, SLOT(slot_refresh()));

    _state.gui_state.action_next_scan = new QAction(tr("&Next Scan"), this);
    _state.gui_state.action_next_scan->setShortcut(QKeySequence(tr("Ctrl+N")));
    _state.gui_state.action_next_scan->setStatusTip(
        tr("Advance to next scan"));
    _state.gui_state.action_next_scan->setToolTip(
        tr("Advance to next scan"));
    connect(_state.gui_state.action_next_scan,
            SIGNAL(triggered()), this, SLOT(slot_next_scan()));

    _state.gui_state.action_update_params = new QAction(tr("&Update Params"), this);
    _state.gui_state.action_update_params->setShortcut(QKeySequence(tr("Ctrl+U")));
    _state.gui_state.action_update_params->setStatusTip(
        tr("Update Params"));
    _state.gui_state.action_update_params->setToolTip(
        tr("Update Params"));
    connect(_state.gui_state.action_update_params,
            SIGNAL(triggered()), this, SLOT(slot_update_params()));

    _state.gui_state.action_project_flow = new QAction(tr("Project &Flow"), this);
    _state.gui_state.action_project_flow->setShortcut(QKeySequence(tr("Ctrl+F")));
    _state.gui_state.action_project_flow->setStatusTip(
        tr("Project Flow"));
    _state.gui_state.action_project_flow->setToolTip(
        tr("Project Flow"));
    connect(_state.gui_state.action_project_flow,
            SIGNAL(triggered()), this, SLOT(slot_project_flow()));

    // disable by default
    _state.gui_state.action_prev_scan->setEnabled(true);
    _state.gui_state.action_refresh->setEnabled(true);
    _state.gui_state.action_next_scan->setEnabled(true);
    _state.gui_state.action_update_params->setEnabled(true);
    _state.gui_state.action_project_flow->setEnabled(true);

    // buttons
    _button_prev_scan = new ActionButton(_state.gui_state.action_prev_scan, this);
    _button_refresh = new ActionButton(_state.gui_state.action_refresh, this);
    _button_next_scan = new ActionButton(_state.gui_state.action_next_scan, this);
    _button_update_params = new ActionButton(_state.gui_state.action_update_params, this);
    _button_project_flow = new ActionButton(_state.gui_state.action_project_flow, this);

    // spin boxes
    _spinbox_em_iterations = new QSpinBox;
    _spinbox_em_iterations->setRange(1, 100);
    _spinbox_em_iterations->setSingleStep(1);
    _spinbox_em_iterations->setValue(FlowApp::default_em_iterations);

    _spinbox_smoothing_weight = new QDoubleSpinBox;
    _spinbox_smoothing_weight->setRange(0, 100000);
    _spinbox_smoothing_weight->setSingleStep(0.01);
    _spinbox_smoothing_weight->setValue(FlowApp::default_smoothing_weight);

    _spinbox_proj_dt = new QDoubleSpinBox;
    _spinbox_proj_dt->setRange(0, 10);
    _spinbox_proj_dt->setSingleStep(0.01);
    _spinbox_proj_dt->setValue(0.10);

    // add status bar
    _state.gui_state.status_bar = statusBar();

    // progress bar
    _state.gui_state.busy_bar = new QProgressBar();
    _state.gui_state.busy_bar->setMinimum(0);
    _state.gui_state.busy_bar->setMaximum(0);
    _state.gui_state.busy_bar->setTextVisible(false);
    _state.gui_state.busy_bar->setVisible(false);
    _state.gui_state.status_bar->addPermanentWidget(_state.gui_state.busy_bar);

    _state.gui_state.fvw = this;
}

void FlowViewerWindow::delete_actions()
{
    delete _state.gui_state.action_prev_scan;
    delete _state.gui_state.action_refresh;
    delete _state.gui_state.action_next_scan;
    delete _state.gui_state.action_update_params;
    delete _state.gui_state.action_project_flow;

    delete _button_prev_scan;
    delete _button_refresh;
    delete _button_next_scan;
    delete _button_update_params;
    delete _button_project_flow;
    delete _spinbox_em_iterations;
    delete _spinbox_smoothing_weight;
    delete _spinbox_proj_dt;
}

void FlowViewerWindow::set_image(osg::ref_ptr<osg::Image> img1, osg::ref_ptr<osg::Image> img2)
{
    _layered_image_1->set_image(img1);
    _layered_image_2->set_image(img2);
}

void FlowViewerWindow::set_frame(int frame)
{
    if (_tracklets)
        _tracklets->set_frame(frame);

    if (_k_sm_pose) _k_sm_pose->set_frame(frame);
    if (_k_raw_pose) _k_raw_pose->set_frame(frame);
    if (_k_filtered_pose) _k_filtered_pose->set_frame(frame);
}

void FlowViewerWindow::set_point_cloud(velodyne_returns_t *vr1, velodyne_returns_t* vr2)
{
    _k_velodyne->set_point_cloud(vr1);
    _layered_image_1->set_point_cloud(vr1);
    _layered_image_2->set_point_cloud(vr2);
}

void FlowViewerWindow::set_depth_map(const depth_buffer_t *dm1, const depth_buffer_t *dm2)
{
    _layered_image_1->set_depth_map(dm1);
    _layered_image_2->set_depth_map(dm2);
}

void FlowViewerWindow::set_image_roi_grid(const image_roi_grid_t *rg1, const image_roi_grid_t *rg2)
{
    _layered_image_1->set_image_roi_grid(rg1);
    _layered_image_2->set_image_roi_grid(rg2);
}

void FlowViewerWindow::set_occ_grids(sparse_occ_grid_t *sog_1, sparse_occ_grid_t *sog_2)
{
    _occ_grid_1->set_sparse_occ_grid(sog_1);
    _occ_grid_1->init = true;

    _occ_grid_2->set_sparse_occ_grid(sog_2);
    _occ_grid_2->init = true;

    _layered_image_1->set_occ_grid(sog_1);
    _layered_image_2->set_occ_grid(sog_1);
}

void FlowViewerWindow::set_occ_grid_proj(sparse_occ_grid_t *sog)
{
    _occ_grid_proj->set_sparse_occ_grid(sog);
    _occ_grid_proj->init = true;
}

void FlowViewerWindow::set_occ_grid1_sel_pos(double x, double y)
{
    if (_occ_grid_1) {
        _occ_grid_1->set_selected_position(x, y);
        _occ_grid_1->rendered = false;
    }

    printf("1: %5.3f, %5.3f\n", x, y);
    _layered_image_1->set_selected_position(x, y);
}

void FlowViewerWindow::set_occ_grid2_sel_pos(double x, double y)
{
    if (_occ_grid_2) {
        _occ_grid_2->set_selected_position(x, y);
        _occ_grid_2->rendered = false;
    }

    printf("2: %5.3f, %5.3f\n", x, y);
    _layered_image_2->set_selected_position(x, y);
}

void FlowViewerWindow::set_occ_grid_proj_sel_pos(double x, double y)
{
    if (_occ_grid_proj) {
        _occ_grid_proj->set_selected_position(x, y);
        _occ_grid_proj->rendered = false;
    }

    printf("proj: %5.3f, %5.3f\n", x, y);
}

void FlowViewerWindow::set_flow_image(sparse_occ_grid_t *sog, flow_image_t *f, double x_12[6])
{
    _flow_image->set_flow_image(sog, f);
    _flow_image->set_rel_pose(x_12);
    _flow_image->init = true;

    _layered_image_1->set_flow_image(sog, f);
    _layered_image_2->set_flow_image(sog, f);
}

void FlowViewerWindow::set_flow_image_sel(double x, double y)
{
    if (_flow_image) {
        _flow_image->set_selected_position(x, y);
    }
}

void FlowViewerWindow::set_image_sel_pos(double x, double y)
{
    _layered_image_1->set_selected_position(x, y);
    _layered_image_2->set_selected_position(x, y);
}

void FlowViewerWindow::set_image_calib(Eigen::MatrixXd P_rect, Eigen::MatrixXd R_rect, Eigen::MatrixXd T_cv)
{
    _layered_image_1->set_image_calib(P_rect, R_rect, T_cv);
    _layered_image_2->set_image_calib(P_rect, R_rect, T_cv);
}

// from osgpick example
bool PickHandler::handle(const osgGA::GUIEventAdapter& ea,
                         osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType()) {
        case(osgGA::GUIEventAdapter::PUSH):
        {
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (view) { pick(view, ea); };
            return false;
        }
        default:
            return false;
    }
}

void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    int mod = ea.getModKeyMask();

    bool ctrl = false;
    if (mod && osgGA::GUIEventAdapter::ModKeyMask::MODKEY_CTRL)
        ctrl = true;

    osgUtil::LineSegmentIntersector::Intersections intersections;

    if (view->computeIntersections(ea, intersections)) {
        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr =
                    intersections.begin();
            hitr != intersections.end();
            ++hitr) {

            osg::Vec3 p = hitr->getWorldIntersectPoint();
            //printf("Intersection: %5.3f %5.3f %5.3f\n", p[0], p[1], p[2]);
            if (_state->fa && ctrl)
                _state->fa->set_selected_position(p[0], p[1]);

            // is it valid?
            //if (!hitr->drawable.valid()) { continue; }

            // is it a Tetrahedron?
            //std::string drawable_name(hitr->drawable->className());
            //printf("Drawable name: %s\n", drawable_name.c_str());

            // does it have a parent?
            //osg::Group* group = hitr->drawable->getParent(0);
            //if (!group) { printf("no ground\n"); continue; }

            // is that parent an occ grid?
            //std::string parent_name = group->className();
            //printf("parent name: %s\n", parent_name.c_str());
            //if (parent_name != osg::FactorGraph::Node::get_class_name())
            //{ continue; }

            // cast to node
            //osg::FactorGraph::Node* n = (osg::FactorGraph::Node*) group;

            //if (!n->get_point_cloud_shown()) {
            //    int64_t utime = n->get_node()->get_node_utime();
            //    velodyne_returns_t* vr = _state->fa->get_point_cloud(utime);
            //    n->show_point_cloud(vr);
            //    velodyne_returns_destroy(vr);
            //} else {
            //    n->hide_point_cloud();
            //}

            break; // only do first one
        }
    }
}

FlowViewer::FlowViewer(osg::ArgumentParser& args) :
        _app(new QApplication(args.argc(), args.argv())),
        _gvwindow(new FlowViewerWindow(args, 0, Qt::Widget))
{
    // add cleanup slot
    _gvwindow->connect(_app, SIGNAL(aboutToQuit()),
                       _gvwindow, SLOT(slot_cleanup()));
}

void
FlowViewer::set_flow_app(FlowApp* fa)
{
    _gvwindow->set_flow_app(fa);
    fa->set_viewer_window(_gvwindow);
}

FlowViewer::~FlowViewer()
{
    delete _gvwindow;
    delete _app;
}
