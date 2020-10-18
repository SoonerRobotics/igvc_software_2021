#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <igvc_msgs/EKFState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Robot state information
double x = 0.0;
double y = 0.0;
double hdg = 0.0;
double vx = 0.0;
double vy = 0.0;
double vhdg = 0.0;

// Pose offset
double x_offset = 0.0;
double y_offset = 0.0;
double hdg_offset = 0.0;

void ekf_callback(const igvc_msgs::EKFState::ConstPtr& ekf)
{
    // Positional
    x = ekf->x;
    y = ekf->y;
    hdg = ekf->yaw;

    // Rates
    vx = ekf->velocity * cos(hdg);
    vy = ekf->velocity * sin(hdg);
    vhdg = ekf->yaw_rate;
}

void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& init_pose)
{
    x_offset = init_pose->pose.pose.position.x;
    y_offset = init_pose->pose.pose.position.y;
    hdg_offset = tf::getYaw(init_pose->pose.pose.orientation);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "igvc_odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber ekf_sub = n.subscribe(n.resolveName("/igvc_ekf/filter_output"), 1, &ekf_callback);
  ros::Subscriber init_sub = n.subscribe(n.resolveName("/initialpose"), 1, &init_pose_callback);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(hdg + hdg_offset);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x + x_offset;
    odom_trans.transform.translation.y = y + y_offset;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x + x_offset;
    odom.pose.pose.position.y = y + y_offset;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vhdg;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}