#include <Eigen/Dense>

#include <iostream>
#include <math.h>

#include "state_filter.hpp"

#include "perls-math/ssc.h"

StateFilter::StateFilter()
{
}

StateFilter::StateFilter(const StateFilter &sf) :
    _mu(sf._mu),
    _sigma(sf._sigma),
    _x0(sf._x0),
    _y0(sf._y0),
    _age(sf._age)
{
}

StateFilter::~StateFilter()
{
}

void StateFilter::init(Pose p)
{

    _x0 = p.x;
    _y0 = p.y;

    _mu(0, 0) = 0;
    _mu(1, 0) = 0;
    _mu(2, 0) = p.h;
    _mu(3, 0) = p.v_f;
    _mu(4, 0) = p.v_h ;

    double x_std = 0.1;
    double y_std = 0.1;
    double t_std = 0.1;
    double v_std = 0.01;
    double t_dot_std = 0.001;

    _sigma.setZero();
    _sigma(0, 0) = x_std * x_std;
    _sigma(1, 1) = y_std * y_std;
    _sigma(2, 2) = t_std * t_std;
    _sigma(3, 3) = v_std * v_std;
    _sigma(4, 4) = t_dot_std * t_dot_std;
}

void
StateFilter::init(double x, double y, double x_flow, double y_flow)
{
    _mu(0, 0) = x;
    _mu(1, 0) = y;

    double v = sqrt(x_flow*x_flow + y_flow*y_flow);
    double t = atan2(y_flow, x_flow);

    _mu(2, 0) = t;
    _mu(3, 0) = v;
    _mu(4, 0) = 0;

    double x_std = 0.1;
    double y_std = 0.1;
    double t_std = 0.2;
    double v_std = 1.5;
    double t_dot_std = 0.2;

    _sigma.setZero();
    _sigma(0, 0) = x_std * x_std;
    _sigma(1, 1) = y_std * y_std;
    _sigma(2, 2) = t_std * t_std;
    _sigma(3, 3) = v_std * v_std;
    _sigma(4, 4) = t_dot_std * t_dot_std;
}

void
StateFilter::run_process_model(double dt)
{
    State mu_bar;

    double x = _mu(0, 0);
    double y = _mu(1, 0);
    double t = _mu(2, 0);
    double v = _mu(3, 0);
    double t_dot = _mu(4, 0);

    double st = sin(t);
    double ct = cos(t);

    mu_bar(0, 0) = x + v * ct * dt;
    mu_bar(1, 0) = y + v * st * dt;
    mu_bar(2, 0) = t + t_dot * dt;
    mu_bar(3, 0) = v;
    mu_bar(4, 0) = t_dot;

    Eigen::Matrix<double, 5, 5> G;
    G.setIdentity();

    G(0, 2) = -v*st*dt;
    G(0, 3) = ct*dt;

    G(1, 2) = v*ct*dt;
    G(1, 3) = st*dt;

    G(2, 4) = dt;

    StateCov sigma_bar = G * _sigma * G.transpose();

    // Add process model noise
    StateCov Q;
    double x_std = 0.0*dt;
    double y_std = 0.0*dt;
    double t_std = 0.0*dt;
    double v_std = 0.5*dt;
    double t_dot_std = 0.3*dt;

    Q.setZero();
    Q(0, 0) = x_std * x_std;
    Q(1, 1) = y_std * y_std;
    Q(2, 2) = t_std * t_std;
    Q(3, 3) = v_std * v_std;
    Q(4, 4) = t_dot_std * t_dot_std;

    //std::cout << "Q:" << std::endl << Q << std::endl;

    //Q.setIdentity();

    sigma_bar += Q;

    _mu = mu_bar;
    _sigma = sigma_bar;

    _mu(2, 0) = minimize_angle(_mu(2, 0));
}

double
StateFilter::minimize_angle(double t)
{
    while (t < -M_PI) t += 2*M_PI;
    while (t >  M_PI) t -= 2*M_PI;

    return t;
}

double
StateFilter::flow_observation(StateFilter::FlowObs z)
{
    double t = _mu(2, 0);
    double v = _mu(3, 0);

    double ct = cos(t);
    double st = sin(t);

    // Observation model for flow
    Eigen::Matrix<double, 2, 5> H;
    H.setZero();

    H(0, 2) = -v * st;
    H(0, 3) = ct;

    H(1, 2) = v * ct;
    H(1, 3) = st;

    // observation noise
    Eigen::Matrix<double, 2, 2> Q;
    Q.setZero();

    double flow_std = 0.3/0.1; // cell res time dt

    Q(0, 0) = flow_std * flow_std;
    Q(1, 1) = flow_std * flow_std;

    // Get Kalman gain
    Eigen::Matrix<double, 2, 2> S = H * _sigma * H.transpose() + Q;
    Eigen::Matrix<double, 5, 2> K = _sigma * H.transpose() * S.inverse();

    //Eigen::Matrix<double, 2, 1> h_mu_bar = H * _mu;
    Eigen::Matrix<double, 2, 1> h_mu_bar;
    h_mu_bar(0, 0) = v*ct;
    h_mu_bar(1, 0) = v*st;

    //std::cout << " Expected obs: " << h_mu_bar.transpose() << std::endl;

    Eigen::Matrix<double, 2, 1> innov = (z - h_mu_bar);

    double mahal = sqrt(innov.transpose() * S.inverse() * innov);
    //printf("Mahal: %5.3f\n", mahal);

    _mu = _mu + K*innov;
    _mu(2, 0) = minimize_angle(_mu(2, 0));

    Eigen::Matrix<double, 5, 5> I;
    I.setIdentity();
    _sigma = (I - K*H) * _sigma * (I - K*H).transpose() + K*Q*K.transpose();

    _age++;

    return mahal;
}

double
StateFilter::flow_observation_pos(StateFilter::FlowObs z)
{
    // Observation model for flow
    Eigen::Matrix<double, 2, 5> H;
    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    // observation noise
    Eigen::Matrix<double, 2, 2> Q;
    Q.setZero();

    double pos_std = 0.15;

    Q(0, 0) = pos_std * pos_std;
    Q(1, 1) = pos_std * pos_std;

    // Get Kalman gain
    Eigen::Matrix<double, 2, 2> S = H * _sigma * H.transpose() + Q;
    Eigen::Matrix<double, 5, 2> K = _sigma * H.transpose() * S.inverse();

    Eigen::Matrix<double, 2, 1> h_mu_bar = H * _mu;

    //std::cout << " Observation : " << z.transpose() << std::endl;
    //std::cout << " Expected obs: " << h_mu_bar.transpose() << std::endl;

    Eigen::Matrix<double, 2, 1> innov = (z - h_mu_bar);

    double mahal = sqrt(innov.transpose() * S.inverse() * innov);
    //printf("Mahal: %5.3f\n", mahal);

    _mu = _mu + K*innov;
    _mu(2, 0) = minimize_angle(_mu(2, 0));

    Eigen::Matrix<double, 5, 5> I;
    I.setIdentity();
    _sigma = (I - K*H) * _sigma * (I - K*H).transpose() + K*Q*K.transpose();

    _age++;

    return mahal;
}

void
StateFilter::pose_observation(Pose p)
{
    Eigen::Matrix<double, 5, 1> z;
    z.setZero();
    z(0, 0) = p.x - _x0;
    z(1, 0) = p.y - _y0;
    z(2, 0) = p.h;
    z(3, 0) = p.v_f;
    z(4, 0) = p.v_h;

    //std::cout << "  Observation: " << z.transpose() << std::endl;

    // Observation model for pose
    Eigen::Matrix<double, 5, 5> H;
    H.setIdentity();

    // observation noise
    Eigen::Matrix<double, 5, 5> Q;
    double x_std = 1.0;
    double y_std = 1.0;
    double h_std = 0.1;
    double v_std = 0.01;
    double t_dot_std = 0.001;

    Q.setZero();
    Q(0, 0) = x_std * x_std;
    Q(1, 1) = y_std * y_std;
    Q(2, 2) = h_std * h_std;
    Q(3, 3) = v_std * v_std;
    Q(4, 4) = t_dot_std * t_dot_std;

    // Get Kalman gain
    Eigen::Matrix<double, 5, 5> S = H * _sigma * H.transpose() + Q;
    Eigen::Matrix<double, 5, 5> K = _sigma * H.transpose() * S.inverse();

    Eigen::Matrix<double, 5, 1> h_mu_bar = H * _mu;

    //std::cout << " Obsveration : " << z.transpose() << std::endl;
    //std::cout << " Expected obs: " << h_mu_bar.transpose() << std::endl;

    Eigen::Matrix<double, 5, 1> innov = (z - h_mu_bar);
    innov(2, 0) = minimize_angle(innov(2, 0));

    _mu = _mu + K*innov;
    _mu(2, 0) = minimize_angle(_mu(2, 0));

    Eigen::Matrix<double, 5, 5> I;
    I.setIdentity();
    _sigma = (I - K*H) * _sigma * (I - K*H).transpose() + K*Q*K.transpose();

    _age++;
}

Pose
StateFilter::get_pose()
{
    Pose p;
    p.x = _mu(0, 0) + _x0;
    p.y = _mu(1, 0) + _y0;
    p.z = 0;
    p.r = 0;
    p.p = 0;
    p.h = _mu(2, 0);

    return p;
}

StateFilter::State
StateFilter::transform(State mu, Pose p1, Pose p2)
{
    double x = mu(0, 0);
    double y = mu(1, 0);
    double t = mu(2, 0);

    // Project (x, y) in frame 1 to frame 2
    double x_1[6] = {x, y, 0, 0, 0, t};

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

    //printf("\t %5.3f %5.3f %5.3f\n", x_1[0], x_1[1], x_1[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_1w[0], x_1w[1], x_1w[2]);
    //printf("\t %5.3f %5.3f %5.3f\n", x_2[0], x_2[1], x_2[2]);

    StateFilter::State res;
    res(0, 0) = x_2[0];
    res(1, 0) = x_2[1];
    res(2, 0) = x_2[5];
    res(3, 0) = mu(3, 0);
    res(4, 0) = mu(4, 0);

    return res;
}

void
StateFilter::change_coordinate_frames(Pose p1, Pose p2)
{
    StateFilter::State res = transform(_mu, p1, p2);

    // Compute Jacobian
    Eigen::Matrix<double, 5, 5> J;
    double delta = 0.001;
    for (int i=0; i<5; i++) {
        State dx;
        dx.setZero();
        dx(i, 0) = delta;

        State mu_dx = _mu + dx;
        State res_dx = transform(mu_dx, p1, p2);

        for (int j=0; j<5; j++) {
            J(i, j) = (res_dx(j) - res(j))/delta;
        }
    }

    // Update state and covariance
    //std::cout << "going from " << _mu.transpose() << " to " << res.transpose() << std::endl;
    //std::cout << "going from " << _sigma << " to " << J.transpose()*_sigma*J << std::endl;
    _mu = res;
    _sigma = J.transpose() * _sigma * J;
}
