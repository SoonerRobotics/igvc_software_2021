#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <fstream>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include "igvc_ekf/ekf.h"
#include "igvc_msgs/EKFState.h"
#include "igvc_msgs/EKFConvergence.h"
#include "igvc_msgs/gps.h"
#include "igvc_msgs/velocity.h"
#include "igvc_msgs/EKFService.h"
#include "igvc_msgs/motors.h"

// Init the EKF
EKF ekf;

// State and control vectors
Eigen::VectorXd x(11), u(2);

// Measurement vector
Eigen::VectorXd z(11);
double last_heading = -1010;
double x_coord, y_coord;

ros::Publisher output_pub;
ros::Publisher convergence_pub;

uint16_t data_init = 0;

// Logging
std::ofstream gps_file;
std::ofstream vel_file;
std::ofstream accel_file;
std::ofstream hdg_file;


igvc_msgs::EKFState encodeEKFState(Eigen::VectorXd x)
{
    igvc_msgs::EKFState ekf_state;

    ekf_state.latitude = x(0);
    ekf_state.longitude = x(1);
    ekf_state.global_heading = x(2);
    ekf_state.x = x(3);
    ekf_state.y = x(4);
    ekf_state.yaw = x(5);
    ekf_state.velocity = x(6);
    ekf_state.yaw_rate = x(7);
    ekf_state.left_angular_vel = x(8);
    ekf_state.right_angular_vel = x(9);
    ekf_state.acceleration = x(10);

    return ekf_state;
}


void updateEKF(const ros::TimerEvent& time_event)
{
    // If all sensor data has been received once
    if(data_init & 0xF)
    {
        // EKF state message
        igvc_msgs::EKFState state;

        // Update the extended kalman filter
        x = ekf.run_filter(z, u);

        // Publish predicted state
        output_pub.publish(encodeEKFState(x));
    }
}

void updateConvergence(const ros::TimerEvent& timer_event)
{
    // If all the sensor data has been received at least once, start publishing convergence
    if(data_init & 0xF)
    {
        igvc_msgs::EKFConvergence converge;

        converge.data = ekf.get_convergence();

        convergence_pub.publish(converge);
    }
}

bool get_robot_state(igvc_msgs::EKFService::Request  &request,
                     igvc_msgs::EKFService::Response &response)
{
    // Construct the state vector to publish to other modules
    response.state = encodeEKFState(x);

    return true;
}

/******
 * Helper functions
 ******/


float constrainAngle(float x){
    x = std::fmod(x + 180,360);
    if (x < 0)
        x += 360;
    return x - 180;
}

float angleDiff(float a,float b){
    float dif = std::fmod(b - a + 180,360);
    if (dif < 0)
        dif += 360;
    return dif - 180;
}



/******
 * Measurement callbacks
 ******/


void updateGPS(const igvc_msgs::gps::ConstPtr& gps_msg)
{
    std::string data = std::to_string(gps_msg->latitude) + ", " + std::to_string(gps_msg->longitude) + "\n";
    gps_file << data;

    // Update GPS coords
    z(0) = degreesToRadians(gps_msg->latitude);   // Latitude
    z(1) = degreesToRadians(gps_msg->longitude);  // Longitude

    // Show the GPS has been initialized
    data_init |= 1;
}

void updateVelocity(const igvc_msgs::velocity::ConstPtr& vel_msg)
{
    // Update measurement
    z(6) = WHEEL_RADIUS * (vel_msg->leftVel + vel_msg->rightVel) / 2.0;
    z(8) = vel_msg->leftVel;
    z(9) = vel_msg->rightVel;

    // Show that the velocity has been updated
    data_init |= (1 << 1);

    std::string data = std::to_string(z(6)) + ", " + std::to_string(z(8)) + ", " + std::to_string(z(9)) +  "\n";
    vel_file << data;
}

void updateIMU(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    z(10) = imu_msg->linear_acceleration.x;

    // Show that the acceleration and heading have been updated
    data_init |= (1 << 2);

    tf::Quaternion q(imu_msg->orientation.w, imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::cout << roll << " -- " << pitch << " -- " << yaw << std::endl;

    // HACK: For some reason, pitch is the yaw from the IMU in the simulator
    double deg_hdg = radiansToDegrees(pitch);

    if(last_heading < -1000)
    {
        last_heading = deg_hdg;
    }

    z(7) = degreesToRadians(angleDiff(last_heading, deg_hdg)) / 0.02;   // TODO: replace with angular velocity from IMU once coordinates are established better
    z(5) += degreesToRadians(angleDiff(last_heading, deg_hdg)); // Update local heading from change to global heading
    z(2) = degreesToRadians(deg_hdg);                 // Update global heading
    last_heading = deg_hdg;

    // Update the heading file
    std::string data = std::to_string(z(2)) + ", " + std::to_string(z(5)) + "\n";
    hdg_file << data;

    // Update the accel file
    data = std::to_string(z(10)) + "\n";
    accel_file << data;
}

void updateControlSignal(const igvc_msgs::motors::ConstPtr& motors)
{
    u(0) = motors->left;
    u(1) = motors->right;
}


void updatePosition(const ros::TimerEvent& timer_event)
{
    // position initialization flag
    data_init |= (1 << 3);

    // Compute time difference
    double dt = (timer_event.current_real - timer_event.last_real).toSec();

    // X component velocity and acceleration
    double vx = z(6) * cos(z(5));
    double ax = z(10) * cos(z(5));
    x_coord += (vx * dt) + (ax * pow(dt, 2));

    // Y component velocity and acceleration
    double vy = z(6) * sin(z(5));
    double ay = z(10) * sin(z(5));
    y_coord += (vy * dt) + (ay * pow(dt, 2));

    // Update the measurement vector
    z(3) = x_coord;
    z(4) = y_coord;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "igvc_ekf_node");

    ros::NodeHandle ekf_node;

    // Initialize the measurement vector
    z.resize(11);
    z.setZero(11);

    // Initialize the x and y coordinates to 0 (the starting position)
    x_coord = 0;
    y_coord = 0;

    // Initialize the subscribers
    ros::Subscriber imu_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/imu"), 1, &updateIMU);
    ros::Subscriber vel_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/velocity"), 1, &updateVelocity);
    ros::Subscriber gps_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/gps"), 1, &updateGPS);
    ros::Subscriber ctrl_sub = ekf_node.subscribe(ekf_node.resolveName("/igvc/motors_raw"), 1, &updateControlSignal);

    // Initialize logging
    std::string path = ros::package::getPath("igvc_ekf");

    gps_file.open(path + "/data/gps_log.csv");
    vel_file.open(path + "/data/vel_log.csv");
    accel_file.open(path + "/data/accel_log.csv");
    hdg_file.open(path + "/data/hdg_log.csv");

    gps_file << "lat, lon\n";

    // Initialize the state and control vectors
    x.resize(11);
    u.resize(2);
    x << degreesToRadians(35.194881), degreesToRadians(-97.438621), degreesToRadians(0), x_coord, y_coord, degreesToRadians(0), 0, 0, 0, 0, 0;
    u << 0, 0; //6.3, 3.14;

    // Initialize the EKF
    ekf.init(x);

    // Publishers
    output_pub = ekf_node.advertise<igvc_msgs::EKFState>(ekf_node.resolveName("/igvc_ekf/filter_output"), 10);
    convergence_pub = ekf_node.advertise<igvc_msgs::EKFConvergence>(ekf_node.resolveName("/igvc_ekf/EKFConvergence"), 10);

    // Service
    ros::ServiceServer get_state_srv = ekf_node.advertiseService("/igvc_ekf/get_robot_state", &get_robot_state);

    // Timers
    ros::Timer dead_reckon_update = ekf_node.createTimer(ros::Duration(0.01), &updatePosition, false);
    ros::Timer ekf_update = ekf_node.createTimer(ros::Duration(0.02), &updateEKF, false);
    ros::Timer converge_update = ekf_node.createTimer(ros::Duration(0.025), &updateConvergence, false);

    ros::spin();

    return 0;
}