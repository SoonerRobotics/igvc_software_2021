#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher lidar_pub;

void onLidarCallback(const sensor_msgs::LaserScan::ConstPtr& lidar)
{
    // Receive the lidar data and change the frame id
    sensor_msgs::LaserScan scan_data = *lidar;
    scan_data.header.frame_id = "base_laser";

    std::cout << "wtf\n";

    // republish the lidar data
    lidar_pub.publish(scan_data);
}


int main(int argc, char** argv)
{
    // Init the node
    ros::init(argc, argv, "igvc_laser_tf_pub_node");
    ros::NodeHandle lidar_tf_node;

    // Broadcast the transform at 10 Hz
    ros::Rate rate(10);
    tf::TransformBroadcaster broadcaster;

    // Run the broadcast
    while(lidar_tf_node.ok()){
        // TODO: Make this transform based on the ydlidar scan message (i.e. replace base_laser with the correct frame id)
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.15, 0.0, 0.46)),
                ros::Time::now(),"base_link", "base_laser"));

        rate.sleep();

        ros::spin();
    }
}