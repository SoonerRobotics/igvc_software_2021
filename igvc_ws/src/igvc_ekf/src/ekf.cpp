#include "igvc_ekf/ekf.h"


EKF::EKF()
{
    // Set the covariance matrix to the identity matrix.
    // The covariance begins as A only, so we need a 6x6 matrix (6 state vars).
    this->P_k.setIdentity(6, 6);
    this->P_k *= 1.25; // multiply by some constant to start out

    // Define the process noise. Similarly, it is 6x6.
    this->Q_k.setIdentity(6, 6);
    this->Q_k *= 1.3; // multiply by some constant

    // Define the measurement noise.
    this->R_k.setIdentity(6, 6);
    // TODO Set specific values on the diagonal relating to our measurement uncertainties.
    this->R_k(0, 0) = 2;
    this->R_k(1, 1) = 2;
    this->R_k(3, 3) = 2;
    this->R_k(4, 4) = 2;
    this->R_k(5, 5) = 2;

    // Define the measurement model
    this->H_k.setIdentity(6, 6);

    // Initialize the kalman gain.
    // It is [# of vars] tall by [# of measurement sources] wide
    this->K_k.setZero(6, 6);

    // Define identity matrix
    this->I.setIdentity(6, 6);
}


void EKF::init(Eigen::VectorXd x0)
{
    // set the initial state
    this->x_k = x0;

    // init the timer
    this->last_time = ros::Time::now().toNSec() / NSEC_TO_SEC;
}


Eigen::VectorXd EKF::run_filter(Eigen::VectorXd z_k, Eigen::VectorXd u_k)
{
    // local vars
    double cur_time;
    double dt;

    // find time delta
    cur_time = ros::Time::now().toNSec() / NSEC_TO_SEC;
    dt = cur_time - last_time;
    last_time = cur_time;

    // Run prediction
    this->predict(u_k, z_k, dt);

    // Update from prediction
    this->update(z_k);

    // Return the new state estimate
    return this->x_k;
}


double EKF::get_convergence()
{
    return this->convergence;
}


void EKF::calculate_dynamics(Eigen::VectorXd z_k, double dt)
{
    // My state will be (x, x_dot, y, y_dot, yaw, yaw_dot, v_l, v_r) = x_k
    // The measurements are (v_l, v_r, yaw) = z_k

    // Velocity calculations
    double v_l = z_k(0);
    double v_r = z_k(1);
    double velocity = 0.5 * WHEEL_RADIUS * (v_l + v_r);
    double x_dot    = velocity * cos(z_k(2));
    double y_dot    = velocity * sin(z_k(2));
    double yaw_rate = (WHEEL_RADIUS / WHEELBASE_LEN) * (v_l - v_r);

    // Position Calculations (const. velocity model)
    double x     = x_k(0) + (x_dot * dt);
    double y     = x_k(2) + (y_dot * dt);
    double yaw   = x_k(4) - yaw_rate * dt; //local yaw

    // Update the state
    x_k(0) = x;
    x_k(1) = x_dot;
    x_k(2) = y;
    x_k(3) = y_dot;
    x_k(4) = yaw;
    x_k(5) = yaw_rate;
    x_k(6) = v_l;
    x_k(7) = v_r;

}

// TODO this whole function
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

void EKF::predict(Eigen::VectorXd u_k, Eigen::VectorXd z_k, double dt)
{
    // Predict current state from past state and measurements
    this->calculate_dynamics(z_k, dt);

    // TODO Linearize the dynamics using a jacobian
    this->linear_dynamics(u_k, dt);

    // Update the covariance matrix
    this->P_k = (F_k * P_k * F_k.transpose()) + this->Q_k;
}



// TODO make matrix H s.t. z_mapping = H * X
Eigen::VectorXd EKF::get_measurement_model()
{
    // We compare the measurements, z_k, to the current state as a one-to-one mapping
    Eigen::VectorXd z_mapping(3);
    z_mapping.resize(3);
    z_mapping.setZero(3);
    //double velocity = this->x_k(2) / cos(this->x_k(4)); //v = x_dot / cos(yaw)
    z_mapping(0) = this->x_k(6); //v_l
    z_mapping(1) = this->x_k(7); //v_r
    z_mapping(2) = this->x_k(4); //yaw
    return z_mapping;
}



void EKF::update(Eigen::VectorXd z_k)
{
    // Calculate the innovation (the difference between predicted measurements and the actual measurements)
    this->y_k = z_k - get_measurement_model();

    // Find the covariance of the innovation
    this->S_k = (this->H_k * this->P_k * this->H_k.transpose()) + this->R_k;

    // Convergence calculation
    this->convergence = this->y_k.transpose() * this->S_k.inverse() * this->y_k;

    // Compute Kalman gain
    this->K_k = this->P_k * H_k.transpose() * S_k.inverse();

    // Use the kalman gain to update the state estimate
    this->x_k = this->x_k + this->K_k * this->y_k;

    // Likewise, update the covariance for the state estimate
    this->P_k = (this->I - this->K_k * this->H_k) * this->P_k;
}
