#ifndef _FLOW_STATE_FILTER_H_
#define _FLOW_STATE_FILTER_H_

#include <Eigen/Core>

#include "library/kitti/pose.hpp"

class StateFilter
{

  public:

    typedef Eigen::Matrix<double, 5, 1> State;
    typedef Eigen::Matrix<double, 5, 5> StateCov;
    typedef Eigen::Matrix<double, 2, 1> FlowObs;

    StateFilter();
    StateFilter(const StateFilter &sf);
    ~StateFilter();

    void init(Pose p);
    void init(double x, double y, double x_flow, double y_flow);

    void run_process_model(double dt);
    double flow_observation(FlowObs z);
    double flow_observation_pos(FlowObs z);
    void pose_observation(Pose p);

    State get_mu() {return _mu;}
    StateCov get_sigma() {return _sigma;}

    Pose get_pose();

    double minimize_angle(double t);

    int get_age() {return _age;}

    void change_coordinate_frames(Pose p1, Pose p2);

  private:

    static State transform(State mu, Pose p1, Pose p2);

    State    _mu;
    StateCov _sigma;

    double _x0 = 0;
    double _y0 = 0;

    int _age = 0;

};

#endif
