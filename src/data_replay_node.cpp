#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "data_replay.h"

ros::Publisher pub_imu, pub_odom, pub_gps;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "replay_node");

	ros::NodeHandle nh("~");
	
    pub_imu = nh.advertise<sensor_msgs::Imu>("/data_replay/imu", 100);
    pub_odom = nh.advertise<nav_msgs::Odometry>("/data_replay/odom", 100);
	pub_gps = nh.advertise<nmea_msgs::Gpgga>("/data_replay/gps", 100);

	std::shared_ptr<DataReplay> pDR(std::make_shared<DataReplay>());

	ros::param::get("~rate", pDR->mnRate);
	ros::param::get("~log_file_path_prefix", pDR->mLogFilePathPredix);
	ros::param::get("~imu_path", pDR->msImuPath);
	ros::param::get("~imu2_path", pDR->msImu2Path);
	ros::param::get("~odom_path", pDR->msOdomPath);
	ros::param::get("~gps_path", pDR->msGpsPath);
	ros::param::get("~localization_path", pDR->msLocalizationPath);
	ros::param::get("~imu_replay", pDR->mbImuReplay);
	ros::param::get("~imu2_replay", pDR->mbImu2Replay);
	ros::param::get("~odom_replay", pDR->mbOdomReplay);
	ros::param::get("~gps_replay", pDR->mbGpsReplay);
	ros::param::get("~localization_replay", pDR->mbLocalizationReplay);
	ros::param::get("~all_replay", pDR->mbAllReplay);
	

	pDR->init();
	pDR->MainRun();

	ros::spin();
	return 0;
}