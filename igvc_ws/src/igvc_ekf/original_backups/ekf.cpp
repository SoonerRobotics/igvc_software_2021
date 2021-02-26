#include "igvc_ekf/ekf.h"


EKF::EKF()
{
    // set the covariance matrix to the identity matrix
    this->P_k.setIdentity(11, 11);
    this->P_k *= 1.25;

    // Define the process noise
    this->Q_k.setIdentity(11, 11);
    this->Q_k *= 1.3;

    // Define the measurement noise
    this->R_k.setIdentity(11, 11);
    this->R_k(0, 0) = 3.395864;
    this->R_k(1, 1) = 4.571665;
    this->R_k(3, 3) = 2;
    this->R_k(4, 4) = 2;
    this->R_k(5, 5) = 2;

    // Define the measurement model
    this->H_k.setIdentity(11, 11);

    // Initialize the kalman gain
    this->K_k.setZero(11, 9);

    // Setup identity matrix
    this->I.setIdentity(11, 11);
}


void EKF::init(Eigen::VectorXd x0)
{
    // set the initial state
    this->x_k = x0;

    // init the timer
    this->last_time = ros::Time::now().toNSec() / NSEC_TO_SEC;
}


Eigen::VectorXd EKF::run_filter(Eigen::VectorXd sensors, Eigen::VectorXd u_k)
{
    // local vars
    double cur_time;
    double dt;

    // find time delta
    cur_time = ros::Time::now().toNSec() / NSEC_TO_SEC;
    dt = cur_time - last_time;
    last_time = cur_time;

    // Run prediction
    this->predict(u_k, dt);

    // Update from prediction
    this->update(sensors);

    // Return the new state estimate
    return this->x_k;
}


double EKF::get_convergence()
{
    return this->convergence;
}




void EKF::calculate_dynamics(Eigen::VectorXd u_k, double dt)
{
    // Velocity calculations
    double velocity = 0.5 * WHEEL_RADIUS * (x_k(8) + x_k(9));
    double x_dot    = velocity * cos(x_k(5));
    double y_dot    = velocity * sin(x_k(5));
    double psi_dot  = (WHEEL_RADIUS / WHEELBASE_LEN) * (x_k(8) - x_k(9));

    // Position Calculations
    double x     = x_k(3) + x_dot * dt;
    double y     = x_k(4) + y_dot * dt;
    double psi   = x_k(5) - psi_dot * dt;
    double theta = x_k(2) - psi_dot * dt;

    // GPS calculations
    double lat = x_k(0) + (x_k(6) * dt) * cos(x_k(2)) / EARTH_RADIUS;
    double lon = x_k(1) + (x_k(6) * dt) * sin(x_k(2)) / (EARTH_RADIUS * cos(x_k(0)));

    // GPS Orientation
    x_k(0) = lat;
    x_k(1) = lon;
    x_k(2) = theta;

    // Local Orientation
    x_k(3) = x;
    x_k(4) = y;
    x_k(5) = psi;

    // Velocities
    x_k(6) = velocity;
    x_k(7) = psi_dot;

    // Wheel velocities
    x_k(8) = u_k(0);
    x_k(9) = u_k(1);
}


void EKF::linear_dynamics(Eigen::VectorXd u_k, double dt)
{
    double velocity = x_k(6);
    double left_vel = x_k(8);
    double right_vel = x_k(9);

    double sin_phi = cos(x_k(0));
    double cos_phi = cos(x_k(0));

    double sin_psi = sin(x_k(5));
    double cos_psi = cos(x_k(5));

    double sin_theta = sin(x_k(2));
    double cos_theta = cos(x_k(2));


    this->F_k.setIdentity(11, 11);

    // latitude
    F_k(0, 2) = -dt * velocity * sin_theta / EARTH_RADIUS;
    F_k(0, 6) = dt * cos_theta / EARTH_RADIUS;

    // longitude
    F_k(1, 0) = dt * velocity * sin_phi * sin_theta / (EARTH_RADIUS * pow(cos_phi, 2));
    F_k(1, 2) = dt * velocity * cos_theta / (EARTH_RADIUS * cos_phi);
    F_k(1, 6) = dt * sin_theta / (EARTH_RADIUS * cos_phi);

    // global heading
    F_k(2, 8) = -(WHEEL_RADIUS / WHEELBASE_LEN) * dt;
    F_k(2, 9) = (WHEEL_RADIUS / WHEELBASE_LEN) * dt;

    // X
    F_k(3, 5) = -dt * (0.5 * WHEEL_RADIUS * (left_vel + right_vel)) * sin_psi;
    F_k(3, 8) = 0.5 * WHEEL_RADIUS * dt * cos_psi;
    F_k(3, 9) = 0.5 * WHEEL_RADIUS * dt * cos_psi;

    // Y
    F_k(4, 5) = dt * (0.5 * WHEEL_RADIUS * (left_vel + right_vel)) * cos_psi;
    F_k(4, 8) = 0.5 * WHEEL_RADIUS * dt * sin_psi;
    F_k(4, 9) = 0.5 * WHEEL_RADIUS * dt * sin_psi;

    // local heading
    F_k(5, 8) = -(WHEEL_RADIUS / WHEELBASE_LEN) * dt;
    F_k(5, 9) = (WHEEL_RADIUS / WHEELBASE_LEN) * dt;

    // velocity
    F_k(6, 8) = 0.5 * WHEEL_RADIUS;
    F_k(6, 9) = 0.5 * WHEEL_RADIUS;

    // angular velocity
    F_k(7, 8) = (WHEEL_RADIUS / WHEELBASE_LEN);
    F_k(7, 9) = -(WHEEL_RADIUS / WHEELBASE_LEN);
}


void EKF::predict(Eigen::VectorXd u_k, double dt)
{
    // Predict current state from past state and control signal
    this->calculate_dynamics(u_k, dt);

    // Linearize the dynamics using a jacobian
    this->linear_dynamics(u_k, dt);

    // Update the covariance matrix
    this->P_k = (F_k * P_k * F_k.transpose()) + this->Q_k;
}




Eigen::VectorXd EKF::get_measurement_model()
{
    // We compare the measurements, z_k, to the current state as a one-to-one mapping
    return this->x_k;
}



void EKF::update(Eigen::VectorXd z_k)
{
    // Calculate the innovation (the difference between predicted measurements and the actual measurements)
    this->yk = z_k - get_measurement_model();

    // Find the covariance of the innovation
    this->Sk = (this->H_k * this->P_k * this->H_k.transpose()) + this->R_k;

    // Convergence calculation
    this->convergence = this->yk.transpose() * this->Sk.inverse() * this->yk;

    // Compute Kalman gain
    this->K_k = this->P_k * H_k.transpose() * Sk.inverse();

    // Use the kalman gain to update the state estimate
    this->x_k = this->x_k + this->K_k * yk;

    // Likewise, update the covariance for the state estimate
    this->P_k = (this->I - this->K_k * this->H_k) * this->P_k;
}
