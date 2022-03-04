#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "data_replay.h"


using namespace std;


template <typename Type>  
Type DataReplay::stringToNum(const string &str)  
{
    istringstream iss(str);  
    Type num;  
    iss >> num;  
    return num;      
}  

vector<string> DataReplay::splitData(string &s)
{
    string strs = s;
    vector<string> vTmp;
    const string blank = " ";
    size_t pos = strs.find(blank);
    size_t size = strs.size();

    while(pos != string::npos)
    {
        string x = strs.substr(0, pos);
        vTmp.push_back(x);
        strs = strs.substr(pos + 1, size);
        pos = strs.find(blank);
    }
    return vTmp;
}

void DataReplay::LoadData(ifstream& fTimes, string& fileData)
{
	getline(fTimes, fileData);
	fileData = fileData + " ";
}


void DataReplay::SetImuMsg(sensor_msgs::Imu &msg, string &imu_data)
{
	if(!imu_data.empty())
	{
		vector<string> vImuTmp;
		vImuTmp = splitData(imu_data);
		vector<double> vImuCvt;
		for(int i = 0; i < vImuTmp.size(); i++)
		{
			vImuCvt.push_back(stringToNum<double>(vImuTmp[i]));
		}

		msg.header.frame_id = "imu";
		msg.header.stamp.sec = vImuCvt[0];
		msg.header.stamp.nsec = vImuCvt[1];
		msg.linear_acceleration.x = vImuCvt[2];
		msg.linear_acceleration.y = vImuCvt[3];
		msg.linear_acceleration.z = vImuCvt[4];
		msg.angular_velocity.x = vImuCvt[5];
		msg.angular_velocity.y = vImuCvt[6];
		msg.angular_velocity.z = vImuCvt[7];
		msg.orientation.x = vImuCvt[8];
		msg.orientation.y = vImuCvt[9];
		msg.orientation.z = vImuCvt[10];
		msg.orientation.w = vImuCvt[11];

		Eigen::Quaterniond Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
		ROS_INFO("time:%9d.%9d Quat roll(x) pitch(y) yaw(z) = %f %f %f", msg.header.stamp.sec, 
			msg.header.stamp.nsec, Quat.matrix().eulerAngles(0, 1, 2).transpose()(0) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(1) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(2) / M_PI * 180 );
	}
}

void DataReplay::SetOdomMsg(sensor_msgs::Imu &msg, string &odom_data)
{
	if(!odom_data.empty())
	{
		vector<string> vOdomTmp;
		vOdomTmp = splitData(odom_data);
		vector<double> vOdomCvt;
		for(int i = 0; i < vOdomTmp.size(); i++)
		{
			vOdomCvt.push_back(stringToNum<double>(vOdomTmp[i]));
		}

		msg.header.frame_id = "imu";
		msg.header.stamp.sec = vOdomCvt[0];
		msg.header.stamp.nsec = vOdomCvt[1];
		msg.orientation.x = vOdomCvt[5];
		msg.orientation.y = vOdomCvt[6];
		msg.orientation.z = vOdomCvt[7];
		msg.orientation.w = vOdomCvt[8];

		Eigen::Quaterniond Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
		ROS_INFO("time:%9d.%9d Quat roll(x) pitch(y) yaw(z) = %f %f %f", msg.header.stamp.sec, 
			msg.header.stamp.nsec, Quat.matrix().eulerAngles(0, 1, 2).transpose()(0) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(1) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(2) / M_PI * 180 );
	}
}

void DataReplay::PubMsg(sensor_msgs::Imu &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

void DataReplay::MainRun()
{

}

// void SetRecordImuPrefix(const std::string& logFile)
// {
//     mLogOfstream.open(logFile.c_str(), std::ios::app);
//     mLogOfstream
//         << "time_stamp_sec time_stamp_nsec"
//         << std::endl;
//     mLogOfstream.close();
// }

// void RecordLog(sensor_msgs::Imu &msg, const std::string& logFile){
// 	mLogOfstream.open(logFile.c_str(), std::ios::app);
//   	mLogOfstream << msg.header.stamp.sec << " "
//                  << msg.header.stamp.nsec << " "
//                  << msg.linear_acceleration.x << " "
//                  << msg.linear_acceleration.y << " "
//                  << msg.linear_acceleration.z << " "
//                  << msg.angular_velocity.x << " "
//                  << msg.angular_velocity.y << " "
//                  << msg.angular_velocity.z << " "
//                  << msg.orientation.x << " "
//                  << msg.orientation.y << " "
//                  << msg.orientation.z << " "
//                  << msg.orientation.w << " "
// 				 << std::endl;
//   	mLogOfstream.close();
// }