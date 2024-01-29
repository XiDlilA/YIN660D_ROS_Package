#include "test_imu/yesense_sdk.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "yesense_imu_node");
    ros::NodeHandle private_node_handle("~");
	ros::NodeHandle nh("imu");
    serial::Serial ser;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data", 100);
	ros::Publisher imu_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/imu_pose", 100);
    ros::Rate r(200);
    std::string input;
	std::string read;
    while(ros::ok()) {
        Yesense_SDK::serial_port.run(imu_pub);
        ros::spinOnce();
		r.sleep();
    }
    return 0;
}