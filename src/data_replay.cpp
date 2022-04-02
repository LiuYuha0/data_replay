#include "data_replay.h"

using namespace std;

DataReplay::DataReplay() 
{

}

void DataReplay::init()
{
	if(mbImuReplay || mbAllReplay)  mfImu.open(msImuPath);
	if(mbImu2Replay)  mfImu.open(msImu2Path);
	if(mbOdomReplay || mbAllReplay)  mfOdom.open(msOdomPath);
	if(mbGpsReplay || mbAllReplay)  mfGps.open(msGpsPath);
	if(mbLocalizationReplay)  mfLclz.open(msLocalizationPath);
}

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

void DataReplay::toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}

void DataReplay::LoadData(ifstream& fTimes, string& fileData)
{
	getline(fTimes, fileData);
	if(fileData.empty()) {
		ROS_ERROR("no data");
		return;
	}
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

		maImuNextTimeStamp[0] = vImuCvt[0];
		maImuNextTimeStamp[1] = vImuCvt[1];

		if(mbShowRPY){
		Eigen::Quaterniond Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
		ROS_INFO("time:%9d.%9d Quat roll(x) pitch(y) yaw(z) = %f %f %f", msg.header.stamp.sec, 
			msg.header.stamp.nsec, Quat.matrix().eulerAngles(0, 1, 2).transpose()(0) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(1) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(2) / M_PI * 180 );
		}
	}
}

void DataReplay::SetOdomLclzAsImuMsg(sensor_msgs::Imu &msg, string &odom_data)
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

		if(mbShowRPY){
		Eigen::Quaterniond Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
		ROS_INFO("time:%9d.%9d Quat roll(x) pitch(y) yaw(z) = %f %f %f", msg.header.stamp.sec, 
			msg.header.stamp.nsec, Quat.matrix().eulerAngles(0, 1, 2).transpose()(0) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(1) / M_PI * 180,
			Quat.matrix().eulerAngles(0, 1, 2).transpose()(2) / M_PI * 180 );
		}
	}
}

void DataReplay::SetOdomMsg(nav_msgs::Odometry &msg, string &odom_data)
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

		msg.header.frame_id = "odom";
		msg.header.stamp.sec = vOdomCvt[0];
		msg.header.stamp.nsec = vOdomCvt[1];
        msg.pose.pose.position.x = vOdomCvt[2];
        msg.pose.pose.position.y = vOdomCvt[3];
        msg.pose.pose.position.z = vOdomCvt[4];
        msg.pose.pose.orientation.x = vOdomCvt[5];
        msg.pose.pose.orientation.y = vOdomCvt[6];
        msg.pose.pose.orientation.z = vOdomCvt[7];
        msg.pose.pose.orientation.w = vOdomCvt[8];

		maOdomTimeStamp[0] = vOdomCvt[0];
		maOdomTimeStamp[1] = vOdomCvt[1];
	}
}

void DataReplay::SetGpsmMsg(nmea_msgs::Gpgga &msg, string &gps_data)
{
	if(!gps_data.empty())
	{
		vector<string> vGpsDataSplit;
		vGpsDataSplit = splitData(gps_data);
		// msg.header.frame_id = "gps";
        msg.header.stamp.sec = stringToNum<double>(vGpsDataSplit[0]);
        msg.header.stamp.nsec = stringToNum<double>(vGpsDataSplit[1]);
        msg.header.frame_id = vGpsDataSplit[2].empty() ? "0" : vGpsDataSplit[2];
        msg.utc_seconds = vGpsDataSplit[3].empty()? 0.: stringToNum<float>(vGpsDataSplit[3]);
        msg.lat = vGpsDataSplit[4].empty()? 0. :  stringToNum<double>(vGpsDataSplit[4]);
        msg.lat_dir = vGpsDataSplit[5].empty() ? "0": vGpsDataSplit[5];
        msg.lon = vGpsDataSplit[6].empty()? 0. : stringToNum<double>(vGpsDataSplit[6]);
        msg.lon_dir = vGpsDataSplit[7].empty() ? "0" : vGpsDataSplit[7];
        msg.gps_qual = vGpsDataSplit[8].empty() ? 0 : stringToNum<int>(vGpsDataSplit[8]);
        msg.num_sats = vGpsDataSplit[9].empty() ? 0 : stringToNum<int>(vGpsDataSplit[9]);
        msg.hdop = vGpsDataSplit[10].empty()? 0. : stringToNum<float>(vGpsDataSplit[10]);
        msg.alt = vGpsDataSplit[11].empty()? 0. : stringToNum<float>(vGpsDataSplit[11]);
        msg.altitude_units = vGpsDataSplit[12].empty() ? "0" : vGpsDataSplit[12];
        msg.undulation = vGpsDataSplit[13].empty()? 0. : stringToNum<float>(vGpsDataSplit[13]);
        msg.undulation_units = vGpsDataSplit[14].empty() ? "0" : vGpsDataSplit[14];
        msg.diff_age = vGpsDataSplit[15].empty() ? 0 : stringToNum<int>(vGpsDataSplit[15]);

		maGpsTimeStamp[0] = stringToNum<double>(vGpsDataSplit[0]);
		maGpsTimeStamp[1] = stringToNum<double>(vGpsDataSplit[1]);
	}
}

void DataReplay::setLclzAsPathMsg(nav_msgs::Path &msg, string &path_data)
{
	if(!path_data.empty()){
		vector<string> vPathDataSplit = splitData(path_data);

		msg.header.frame_id = "world";
		msg.header.stamp.sec = stringToNum<double>(vPathDataSplit[0]);
		msg.header.stamp.nsec = stringToNum<double>(vPathDataSplit[1]);

		geometry_msgs::PoseStamped this_pose_stamped;
		this_pose_stamped.header.stamp.sec = stringToNum<double>(vPathDataSplit[0]);
		this_pose_stamped.header.stamp.nsec = stringToNum<double>(vPathDataSplit[1]);
		this_pose_stamped.pose.position.x = stringToNum<double>(vPathDataSplit[6]);
		this_pose_stamped.pose.position.y = stringToNum<double>(vPathDataSplit[7]);
		this_pose_stamped.pose.position.z = 0;
		msg.poses.push_back(this_pose_stamped);
	}

}

void DataReplay::PubMsg(sensor_msgs::Imu &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

void DataReplay::PubMsg(nav_msgs::Odometry &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

void DataReplay::PubMsg(nmea_msgs::Gpgga &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

void DataReplay::PubMsg(nav_msgs::Path &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

void DataReplay::ImuReplayRun()
{
	ros::Rate loop_rate(mnRate);
	while(!mfImu.eof())
	{
		LoadData(mfImu, msImuData);
		SetImuMsg(mImuMsg, msImuData);
		PubMsg(mImuMsg, pub_imu);
		loop_rate.sleep();
	}
}

void DataReplay::OdomReplayRun()
{
	ros::Rate loop_rate(mnRate);
	while(!mfOdom.eof())
	{
		LoadData(mfOdom, msOdomData);
		SetOdomLclzAsImuMsg(mImuMsg, msOdomData);
		PubMsg(mImuMsg, pub_imu);
		loop_rate.sleep();
	}
}

void DataReplay::LclzPathReplayRun()
{
	ros::Rate loop_rate(mnRate);
	while(!mfLclz.eof())
	{
		LoadData(mfLclz, msLclzData);
		setLclzAsPathMsg(mPathMsg, msLclzData);
		PubMsg(mPathMsg, pub_path);
		loop_rate.sleep();
	}
}

void DataReplay::AllReplayRun()
{
	clock_t startTime, endTime;
	while(!mfImu.eof())
	{
		startTime = clock();
		if (mbFirstFlag){
			// get first data
			LoadData(mfImu, msImuData);
			SetImuMsg(mImuMsg, msImuData);
			maImuCurTimeStamp[0] = maImuNextTimeStamp[0];
			maImuCurTimeStamp[1] = maImuNextTimeStamp[1];
			// ROS_INFO("time:%9d.%9d", maImuCurTimeStamp[0], maImuCurTimeStamp[1]);
			LoadData(mfOdom, msOdomData);
			SetOdomMsg(mOdomMsg, msOdomData);
			LoadData(mfGps, msGpsData);
			SetGpsmMsg(mGpsMsg, msGpsData);
			mbFirstFlag = false;
		}
		else{
			maImuCurTimeStamp[0] = maImuNextTimeStamp[0];
			maImuCurTimeStamp[1] = maImuNextTimeStamp[1];
		}
		
		PubMsg(mImuMsg, pub_imu);

		long long int odom_diff = 
			(maImuCurTimeStamp[0] - maOdomTimeStamp[0]) * 1e9 +
          	(maImuCurTimeStamp[1] - maOdomTimeStamp[1]);
		if(odom_diff > 0){
			PubMsg(mOdomMsg, pub_odom);
			LoadData(mfOdom, msOdomData);
			SetOdomMsg(mOdomMsg, msOdomData);
			// ROS_INFO("odom_time:%9d.%9d", maOdomTimeStamp[0], maOdomTimeStamp[1]);
		}

		long long int gps_diff = 
			(maImuCurTimeStamp[0] - maGpsTimeStamp[0]) * 1e9 +
          	(maImuCurTimeStamp[1] - maGpsTimeStamp[1]);
		if(gps_diff > 0){
			PubMsg(mGpsMsg, pub_gps);
			LoadData(mfGps, msGpsData);
			SetGpsmMsg(mGpsMsg, msGpsData);
			// ROS_INFO("gps_time:%9d.%9d", maGpsTimeStamp[0], maGpsTimeStamp[1]);
		}
		LoadData(mfImu, msImuData);
		SetImuMsg(mImuMsg, msImuData);
		// ROS_INFO("imu_time:%9d.%9d", maImuNextTimeStamp[0], maImuNextTimeStamp[1]);
		long long int NanoSecDiff = maImuNextTimeStamp[1] - maImuCurTimeStamp[1];
		long long int SecDiff = maImuNextTimeStamp[0] - maImuCurTimeStamp[0];
		mnSleepTime = SecDiff * 1e9 + NanoSecDiff;

		endTime = clock();
		long long int RunTimeNanoSec = 1e9 * (endTime - startTime) / CLOCKS_PER_SEC;
		this_thread::sleep_for(chrono::nanoseconds(mnSleepTime - RunTimeNanoSec));
	}
}

void DataReplay::MainRun()
{
	// if(mbImuReplay == true || mbImu2Replay == true) ImuReplayRun();
	// if(mbOdomReplay) OdomReplayRun();
	if(mbLocalizationReplay) LclzPathReplayRun();
	// if(mbAllReplay) AllReplayRun();
}

void DataReplay::SetLog(std::string &log)
{
	char buff[80];
	struct tm* info;
	time_t now;
	time(&now);
	info = (localtime(&now));
	strftime(buff, 30, "_%Y_%m_%d_%H_%M_%S.txt", info);
	log = mLogFilePathPredix + "/" + log + std::string(buff);
}

void DataReplay::RecordData(const std::string& logFile, sensor_msgs::Imu &msg)
{
	mImuLogOfstream.open(logFile.c_str(), std::ios::app);
	mImuLogOfstream.setf(ios::fixed);
	mImuLogOfstream.precision(9);
	Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
	double r, p, y;
	toEulerAngle(q, r, p, y);
  	mImuLogOfstream << msg.header.stamp.sec << " "
                 << msg.header.stamp.nsec << " "
                 << msg.linear_acceleration.x << " "
                 << msg.linear_acceleration.y << " "
                 << msg.linear_acceleration.z << " "
                 << msg.angular_velocity.x << " "
                 << msg.angular_velocity.y << " "
                 << msg.angular_velocity.z << " "
                 << msg.orientation.x << " "
                 << msg.orientation.y << " "
                 << msg.orientation.z << " "
                 << msg.orientation.w << " "
				 << r / M_PI * 180 << " "
				 << p / M_PI * 180 << " "
				 << y / M_PI * 180
				 << std::endl;
  	mImuLogOfstream.close();
}

void DataReplay::RecordData(const std::string& logFile, nav_msgs::Odometry &msg)
{
	mOdomLogOfstream.open(logFile.c_str(), std::ios::app);
	mOdomLogOfstream.setf(ios::fixed);
	mOdomLogOfstream.precision(9);
	mOdomLogOfstream
		<< msg.header.stamp.sec << " "
		<< msg.header.stamp.nsec << " "
		<< msg.pose.pose.position.x << " "
		<< msg.pose.pose.position.y << " "
		<< msg.pose.pose.position.z << " "
		<< msg.pose.pose.orientation.x << " "
		<< msg.pose.pose.orientation.y << " "
		<< msg.pose.pose.orientation.z << " "
		<< msg.pose.pose.orientation.w << " "
		<< std::endl;
	mOdomLogOfstream.close();
}

void DataReplay::RecordData(const std::string& logFile, nmea_msgs::Gpgga &msg)
{
	mGpsLogOfstream.open(logFile.c_str(), std::ios::app);
	mGpsLogOfstream.setf(ios::fixed);
	mGpsLogOfstream.precision(9);
	mGpsLogOfstream
		<< msg.header.stamp.sec << " "
		<< msg.header.stamp.nsec << " "
		<< msg.header.frame_id << " "
		<< msg.utc_seconds << " "
		<< msg.lat << " "
		<< msg.lat_dir << " "
		<< msg.lon << " "
		<< msg.lon_dir << " "
		<< msg.gps_qual << " "
		<< msg.num_sats << " "
		<< msg.hdop << " "
		<< msg.alt << " "
		<< msg.altitude_units << " "
		<< msg.undulation << " "
		<< msg.undulation_units << " "
		<< msg.diff_age 
		<< std::endl;
	mGpsLogOfstream.close();
}
