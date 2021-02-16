#ifndef IGVC_EKF_H
#define IGVC_EKF_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

// Macros for unit conversion
#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)

// Global Constants
#define EARTH_RADIUS (double)(6378137)  // meters
#define LAT_RAD_TO_M (double)(degreesToRadians(111111))   //meters
#define LON_RAD_TO_M (double)(degreesToRadians(78710))    //meters
#define NSEC_TO_SEC (double)(1000.0 * 1000.0 * 1000.0)

// Robot Properties
#define WHEELBASE_LEN (double)(0.84455) // 33.25 inches (in meters)
#define WHEEL_RADIUS (double)(0.127)    // 5 inches (in meters)



class EKF
{
    public:
        EKF();

        void init(Eigen::VectorXd x0);
        Eigen::VectorXd run_filter(Eigen::VectorXd sensor_data, Eigen::VectorXd u_k);

        double get_convergence();

    private:
        // Filter variables
        Eigen::VectorXd x_k;
        Eigen::MatrixXd P_k;
        double last_time;

        // Prediction functions
        void calculate_dynamics(Eigen::VectorXd u_k, double dt);
        void linear_dynamics(Eigen::VectorXd u_k, double dt);
        void predict(Eigen::VectorXd u_k, double dt);

        // Prediction vars
        Eigen::MatrixXd F_k;        // Jacobian of transfer function
        Eigen::MatrixXd Q_k;        // process noise


        // Update functions
        Eigen::VectorXd get_measurement_model();
        void update(Eigen::VectorXd z_k);

        // Update vars
        Eigen::MatrixXd H_k;    // Sensor model
        Eigen::MatrixXd K_k;    // Kalman Gain
        Eigen::MatrixXd R_k;    // Measurement noise
        Eigen::MatrixXd Sk;     // Innovation Covariance
        Eigen::VectorXd yk;     // Innovation

        // Convergence
        double convergence;

        // Identity matrix
        Eigen::MatrixXd I;

};

#endif