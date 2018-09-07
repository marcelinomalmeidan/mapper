

// Standard includes
#include <ros/ros.h>
#include <mapper/mapper_class.h>

int main(int argc, char **argv) {

	ROS_INFO("[mapper_node]: Starting...");

	ros::init(argc, argv, "mapper_node");
	ros::NodeHandle node("~");
	
	mapper::MapperClass octomapper;
	octomapper.Initialize(&node);

	ros::spin();

	return 0;
}