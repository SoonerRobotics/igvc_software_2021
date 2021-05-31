#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

//CONSTANTS
#define MIN_DISTANCE 0.5  // Min distance (meters)
#define MAX_DISTANCE 10 // Max distance (meters)
#define DIMENSIONS 200  // The grid dimensions
#define RESOLUTION 0.1  // Resolution of map (meters)
#define LIDAR_POS 100   // Row and Col indices for LiDAR


ros::Publisher map_pub;

/**
 * @brief
 *
 * @param msg
 */

void onLidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//Instantiate message to publish to
	nav_msgs::OccupancyGrid map_data;

	// Change the map meta data. Can also choose origin?
	nav_msgs::MapMetaData map_info;
	map_info.resolution = RESOLUTION;
	map_info.width = DIMENSIONS;
	map_info.height = DIMENSIONS;

	geometry_msgs::Pose origin;
	origin.position.x = -DIMENSIONS / 2 * RESOLUTION;
	origin.position.y = -DIMENSIONS / 2 * RESOLUTION;
	map_info.origin = origin;

	// Give map meta data to occupancy grid
	map_data.info = map_info;
	// Set up a vector to handle the lidar data
	std::vector<signed char> lidar_data(DIMENSIONS*DIMENSIONS);
	//Instantiate variables for obstacle detection
	float angle;
	float distance;
	std::vector<float> ranges = msg->ranges;

	for (int i = 0; i < ranges.size(); i++)
	{
		// If something is detected within range
		if (ranges[i] > MIN_DISTANCE && ranges[i] <= MAX_DISTANCE)
		{
			// Calculate angle
			angle = msg->angle_min + (i * msg->angle_increment);
			// Get cartesian coordinates
			float x = ranges[i] * cos(angle);
			float y = ranges[i] * sin(angle);
			// Convert to number of indices away from LiDAR
			int index = (LIDAR_POS - round(y / RESOLUTION)) * DIMENSIONS + (LIDAR_POS - round(x / RESOLUTION));
			// Save value to save on text
			int cur_val = lidar_data[index];
			// If the indice already has an obstacle, increment "certainty"
			if (cur_val > 0 && cur_val < 100)
			{
				lidar_data[index] += 1;
			}
			else if (cur_val != 100)
			{
				lidar_data[index] = 1;
			}
			//lidar_data[index] = 100;
		}
	}
	// Set anything in the occupancy grid without a value to 0
	for (int i = 0; i < DIMENSIONS * DIMENSIONS; ++i)
	{
		if (lidar_data[i] > 0 && lidar_data[i] <= 100)
			lidar_data[i] = 100;
		if (lidar_data[i] != 100)
			lidar_data[i] = 0;
	}

	map_data.data = lidar_data;
	// Publish message data to the topic
	map_pub.publish(map_data);
}

/**
 * @brief
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char **argv)
{
	//Initialize the node
	ros::init(argc, argv, "lidar_map_node");
	//Set up node
	ros::NodeHandle lidar_map_node;
	//Create the publisher for the map
	map_pub = lidar_map_node.advertise<nav_msgs::OccupancyGrid>(lidar_map_node.resolveName("/igvc_vision/map"), 10);
	//Create the subscriber for the LiDAR
	ros::Subscriber lidar = lidar_map_node.subscribe(lidar_map_node.resolveName("/scan"), 10, onLidarCallback);
	//Automatically handles callbacks
	ros::spin();

	return 0;
}
