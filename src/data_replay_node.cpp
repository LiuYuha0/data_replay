#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "data_replay.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "replay_node");

	ros::NodeHandle nh("~");
	
    ros::Publisher pub_imu = nh.advertise<sensor_msgs::Imu>("/imu_display", 100);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_display", 100);

	int rate;
	bool imu_replay, imu2_replay, odom_replay, localization_replay;
	string imu_path, imu2_path, odom_path, localization_path;
	
	ros::param::get("~rate", rate);
	ros::param::get("~imu_path", imu_path);
	ros::param::get("~imu2_path", imu2_path);
	ros::param::get("~odom_path", odom_path);
	ros::param::get("~localization_path", localization_path);
	ros::param::get("~imu_replay", imu_replay);
	ros::param::get("~imu2_replay", imu2_replay);
	ros::param::get("~odom_replay", odom_replay);
	ros::param::get("~localization_replay", localization_replay);

	ros::Rate loop_rate(rate);

	std::shared_ptr<DataReplay> DR(std::make_shared<DataReplay>());

	if(imu_replay)  DR->mfImu.open(imu_path);
	if(imu2_replay)  DR->mfImu.open(imu2_path);
	if(odom_replay)  DR->mfOdom.open(localization_path);
	if(localization_replay)  DR->mfOdom.open(localization_path);

	while(ros::ok())
	{
		if(imu_replay == true || imu2_replay == true)
		{
			while(!DR->mfImu.eof())
			{
				DR->LoadData(DR->mfImu, DR->msImuData);
				DR->SetImuMsg(DR->mImuMsg, DR->msImuData);
				DR->PubMsg(DR->mImuMsg, pub_imu);
				loop_rate.sleep();
			}
		}

		if(odom_replay == true || localization_replay == true)
		{
			while(!DR->mfOdom.eof())
			{
				DR->LoadData(DR->mfOdom, DR->msOdomData);
				DR->SetOdomMsg(DR->mImuMsg, DR->msOdomData);
				DR->PubMsg(DR->mImuMsg, pub_imu);
				loop_rate.sleep();
			}			
		}
	}

	ros::spin();
	return 0;
}
