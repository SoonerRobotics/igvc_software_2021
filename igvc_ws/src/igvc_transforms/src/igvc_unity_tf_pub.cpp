#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
    // Init the node
    ros::init(argc, argv, "igvc_unity_tf_pub_node");
    ros::NodeHandle unity_tf_node;

    // Broadcast the transform at 10 Hz
    ros::Rate rate(10);
    tf::TransformBroadcaster broadcaster;

    // Run the broadcast
    while(unity_tf_node.ok()){
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.15, 0.0, 0.46)),
                ros::Time::now(), "base_link", "Unity"));

        rate.sleep();
    }
}