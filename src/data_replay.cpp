#include "data_replay.h"

using namespace std;

DataReplay::DataReplay() {}

void DataReplay::init()
{
	// if(mbImuReplay || mbAllReplay)  mfImu.open(msImuPath);
	// if(mbImu2Replay)  mfImu.open(msImu2Path);
	// if(mbOdomReplay || mbAllReplay)  mfOdom.open(msOdomPath);
	// if(mbGpsReplay || mbAllReplay)  mfGps.open(msGpsPath);
	// if(mbLocalizationReplay)  mfLocalization.open(msLocalizationPath);
	if(mbAllReplay){
		mfImu.open(msImu2Path);
		mfOdom.open(msOdomPath);
		mfGps.open(msGpsPath);
		ROS_INFO("imu_path = %s, odom_path = %s, gps_path = %s", msImu2Path.c_str(), msOdomPath.c_str(), msGpsPath.c_str());
	}
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


void DataReplay::SetImuMsg(sensor_msgs::Imu &msg, ImuData &imu_data_pub)
{
	msg.header.frame_id = "imu_link";
	msg.linear_acceleration.x = imu_data_pub.linear_acceleration.x;
	msg.linear_acceleration.y = imu_data_pub.linear_acceleration.y;
	msg.linear_acceleration.z = imu_data_pub.linear_acceleration.z;
	msg.angular_velocity.x = imu_data_pub.angular_velocity.x;
	msg.angular_velocity.y = imu_data_pub.angular_velocity.y;
	msg.angular_velocity.z = imu_data_pub.angular_velocity.z;
	msg.orientation.x = imu_data_pub.quaternion.x;
	msg.orientation.y = imu_data_pub.quaternion.y;
	msg.orientation.z = imu_data_pub.quaternion.z;
	msg.orientation.w = imu_data_pub.quaternion.w;
	msg.orientation_covariance[0] = 0.0012250000000000002;
	msg.orientation_covariance[4] = 0.0012250000000000002;
	msg.orientation_covariance[8] = 0.0012250000000000002;
	msg.angular_velocity_covariance[0] = 0.0004;
	msg.angular_velocity_covariance[4] = 0.0004;
	msg.angular_velocity_covariance[8] = 0.0004;
	msg.linear_acceleration_covariance[0] = 0.009604000000000001;
	msg.linear_acceleration_covariance[4] = 0.009604000000000001;
	msg.linear_acceleration_covariance[8] = 0.009604000000000001;
	msg.header.stamp.sec = imu_data_pub.time_stamp.sec;
	msg.header.stamp.nsec = imu_data_pub.time_stamp.nanosec;
//          LOG(INFO) << time_now << " " << imu_data_pub.time_stamp.sec << " "
//                    << imu_data_pub.time_stamp.nanosec << " " << imu_idx + 1
//                    << " imu pub";

	if(mbShowRPY){
	Eigen::Quaterniond Quat(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
	ROS_INFO("time:%9d.%9d Quat roll(x) pitch(y) yaw(z) = %f %f %f", msg.header.stamp.sec, 
		msg.header.stamp.nsec, Quat.matrix().eulerAngles(0, 1, 2).transpose()(0) / M_PI * 180,
		Quat.matrix().eulerAngles(0, 1, 2).transpose()(1) / M_PI * 180,
		Quat.matrix().eulerAngles(0, 1, 2).transpose()(2) / M_PI * 180 );
	}
}

void DataReplay::SetOdomMsg(nav_msgs::Odometry &msg, OdomData odom_data_pub)
{
	msg.header.frame_id = "odom";
	msg.child_frame_id = "base_link";
	msg.header.stamp.sec = odom_data_pub.time_stamp.sec;
	msg.header.stamp.nsec = odom_data_pub.time_stamp.nanosec;
	msg.pose.pose.position.x = odom_data_pub.position.x;
	msg.pose.pose.position.y = odom_data_pub.position.y;
	msg.pose.pose.position.z = odom_data_pub.position.z;
	msg.pose.pose.orientation.x = odom_data_pub.quaternion.x;
	msg.pose.pose.orientation.y = odom_data_pub.quaternion.y;
	msg.pose.pose.orientation.z = odom_data_pub.quaternion.z;
	msg.pose.pose.orientation.w = odom_data_pub.quaternion.w;
	msg.pose.covariance[0] = 0.001;
	msg.pose.covariance[7] = 0.001;
	msg.pose.covariance[14] = 0.001;
	msg.pose.covariance[21] = 0.001;
	msg.pose.covariance[28] = 0.001;
	msg.pose.covariance[35] = 0.03;
	msg.twist.covariance[0] = 0.001;
	msg.twist.covariance[7] = 0.001;
	msg.twist.covariance[14] = 0.001;
	msg.twist.covariance[21] = 0.001;
	msg.twist.covariance[28] = 0.001;
	msg.twist.covariance[35] = 0.03;
//          LOG(INFO) << time_now << " " << odom_data_pub.time_stamp.sec << " "
//                    << odom_data_pub.time_stamp.nanosec << " " << odom_idx + 1
//                    << " odom pub";
}

void DataReplay::SetGpsMsg(data_replay::Bestpos &msg, GpsData gps_data_pub)
{
	msg.header.stamp.sec = gps_data_pub.time_stamp.sec;
	msg.header.stamp.nsec = gps_data_pub.time_stamp.nanosec;
	msg.header.frame_id = gps_data_pub.frame_id;
	msg.message_id = gps_data_pub.message_id;
	msg.sol_status = gps_data_pub.sol_status;
	msg.pos_type = gps_data_pub.pos_type;
	msg.lat = gps_data_pub.lat;
	msg.lon = gps_data_pub.lon;
	msg.hgt = gps_data_pub.hgt;
	msg.undulation = gps_data_pub.undulation;
	msg.datum_id = gps_data_pub.datum_id;
	msg.lat_sigma = gps_data_pub.lat_sigma;
	msg.lon_sigma = gps_data_pub.lon_sigma;
	msg.hgt_sigma = gps_data_pub.hgt_sigma;
	msg.stn_id = gps_data_pub.stn_id;
	msg.diff_age = gps_data_pub.diff_age;
	msg.sol_age = gps_data_pub.sol_age;
	msg.svs = gps_data_pub.svs;
	msg.soln_svs = gps_data_pub.soln_svs;
	msg.reserved_1 = gps_data_pub.reserved_1;
	msg.reserved_2 = gps_data_pub.reserved_2;
	msg.reserved_3 = gps_data_pub.reserved_3;
	msg.ext_sol_stat = gps_data_pub.ext_sol_stat;
	msg.galileo_sig_mask = gps_data_pub.galileo_sig_mask;
	msg.gps_glonass_and_bds_sig_mask = gps_data_pub.gps_glonass_and_bds_sig_mask;
//          LOG(INFO) << time_now << " " << gps_data_pub.time_stamp.sec << " "
//                    << gps_data_pub.time_stamp.nanosec << " " << gps_idx + 1
//                    << " gps pub";
}

void DataReplay::SetGpsMsg(nmea_msgs::Gpgga &msg, GpsGpggaData gps_data_pub)
{
	msg.header.stamp.sec = gps_data_pub.time_stamp.sec;
	msg.header.stamp.nsec = gps_data_pub.time_stamp.nanosec;
	msg.header.frame_id = gps_data_pub.frame_id;
	msg.utc_seconds = gps_data_pub.utc_seconds;
	msg.lat = gps_data_pub.lat;
	msg.lat_dir = gps_data_pub.lat_dir;
	msg.lon = gps_data_pub.lon;
	msg.lon_dir = gps_data_pub.lon_dir;
	msg.gps_qual = gps_data_pub.gps_qual;
	msg.num_sats = gps_data_pub.num_sats;
	msg.hdop = gps_data_pub.hdop;
	msg.alt = gps_data_pub.alt;
	msg.altitude_units = gps_data_pub.altitude_units;
	msg.undulation = gps_data_pub.undulation;
	msg.undulation_units = gps_data_pub.undulation_units;
	msg.diff_age = gps_data_pub.diff_age;
}

void DataReplay::setLocalizationAsPathMsg(nav_msgs::Path &msg, string &path_data)
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

void DataReplay::PubMsg(data_replay::Bestpos &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

void DataReplay::PubMsg(nav_msgs::Path &msg, ros::Publisher& pub_msg)
{
	pub_msg.publish(msg);
}

/*
void DataReplay::ImuReplayRun()
{
	ros::Rate loop_rate(mnRate);
	while(!mfImu.eof())
	{
		LoadData(mfImu, msImuFileData);
		sensor_msgs::Imu imu_message;
		SetImuMsg(imu_message, msImuFileData);
		PubMsg(imu_message, pub_imu);
		loop_rate.sleep();
	}
}

void DataReplay::OdomReplayRun()
{
	ros::Rate loop_rate(mnRate);
	while(!mfOdom.eof())
	{
		LoadData(mfOdom, msOdomFileData);
		sensor_msgs::Imu imu_message;
		SetOdomLclzAsImuMsg(imu_message, msOdomFileData);
		PubMsg(imu_message, pub_imu);
		loop_rate.sleep();
	}
}

void DataReplay::LocalizationPathReplayRun()
{
	ros::Rate loop_rate(mnRate);
	while(!mfLocalization.eof())
	{
		LoadData(mfLocalization, msLocalizationFileData);
		nav_msgs::Path path_message;
		setLocalizationAsPathMsg(path_message, msLocalizationFileData);
		PubMsg(path_message, pub_path);
		loop_rate.sleep();
	}
}
*/

void DataReplay::MainRun()
{
	// if(mbImuReplay == true || mbImu2Replay == true) ImuReplayRun();
	// if(mbOdomReplay) OdomReplayRun();
	// if(mbLocalizationReplay) LocalizationPathReplayRun();
	if(mbAllReplay) AllReplayRun();
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
	mGpsLogOfstream << msg.header.stamp.sec << " "
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

void DataReplay::RecordData(nav_msgs::Odometry &msg) {
  mOdomLogOfstream.open(mOdomLogFileName.c_str(), std::ios::app);
  mOdomLogOfstream.setf(ios::fixed);
  mOdomLogOfstream.precision(9);
  mOdomLogOfstream << msg.header.stamp.sec << "." << setw(9)
                   << setfill('0') << right
                   << msg.header.stamp.nsec << " "
                   << msg.pose.pose.position.x << " "
                   << msg.pose.pose.position.y << " "
                   << msg.pose.pose.position.z << " "
                   << msg.pose.pose.orientation.x << " "
                   << msg.pose.pose.orientation.y << " "
                   << msg.pose.pose.orientation.z << " "
                   << msg.pose.pose.orientation.w << std::endl;
  mOdomLogOfstream.close();
}

void DataReplay::LoadData(ifstream& fTimes, string& fileData, int& fileLine, std::deque<string>& dataQue)
{
	while (getline(fTimes, fileData, '\n')) {
      fileLine++;
      dataQue.push_back(fileData);
      fileData.clear();
    }
}

void DataReplay::GetOdomMsg(deque<string>& odom_data_que, deque<OdomData>& odom_msg)
{
	for (int i = 0; i < odom_data_que.size(); ++i) {
	  string odom_i = odom_data_que[i] + " ";
	  vector<string> odom_string_split = splitData(odom_i);
	  OdomData odom_data;
	  int idx = 0;
	  odom_data.time_stamp.sec = stringToNum<long long int>(odom_string_split[idx++]);
	  odom_data.time_stamp.nanosec = stringToNum<long long int>(odom_string_split[idx++]);
	  odom_data.position.x = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.position.y = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.position.z = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.quaternion.x = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.quaternion.y = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.quaternion.z = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.quaternion.w = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.eulers.roll = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.eulers.pitch = stringToNum<double>(odom_string_split[idx++]);
	  odom_data.eulers.yaw = stringToNum<double>(odom_string_split[idx++]);
	  odom_msg.push_back(odom_data);
	  //      LOG(INFO) << std::setiosflags(std::ios::fixed) << setprecision(9)
	  //                << odom_data.time_stamp.sec +
	  //                       1e-9 * odom_data.time_stamp.nsec
	  //                << " " << odom_data.position.x << " " <<
	  //                odom_data.position.y
	  //                << " " << odom_data.position.z << " " <<
	  //                odom_data.quaternion.x
	  //                << " " << odom_data.quaternion.y << " "
	  //                << odom_data.quaternion.z << " " <<
	  //                odom_data.quaternion.w
	  //                << " " << odom_data.eulers.roll << " " <<
	  //                odom_data.eulers.pitch
	  //                << " " << odom_data.eulers.yaw;
	}
}

void DataReplay::GetImuMsg(deque<string>& imu_data_que, deque<ImuData>& imu_msg)
{
	for (int i = 0; i < imu_data_que.size(); ++i) {
	  string imu_i = imu_data_que[i] + " ";
	  vector<string> imu_string_split = splitData(imu_i);
	  ImuData imu_data;
	  int idx = 0;
	  imu_data.time_stamp.sec = stringToNum<long long int>(imu_string_split[idx++]);
	  imu_data.time_stamp.nanosec = stringToNum<long long int>(imu_string_split[idx++]);
	  imu_data.linear_acceleration.x = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.linear_acceleration.y = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.linear_acceleration.z = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.angular_velocity.x = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.angular_velocity.y = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.angular_velocity.z = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.quaternion.x = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.quaternion.y = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.quaternion.z = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.quaternion.w = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.eulers.roll = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.eulers.pitch = stringToNum<double>(imu_string_split[idx++]);
	  imu_data.eulers.yaw = stringToNum<double>(imu_string_split[idx++]);
	  imu_msg.push_back(imu_data);
	  //      LOG(INFO) << std::setiosflags(std::ios::fixed)
	  //                << imu_data.time_stamp << " "
	  //                << imu_data.linear_acceleration.x << " "
	  //                << imu_data.linear_acceleration.y << " "
	  //                << imu_data.linear_acceleration.z << " "
	  //                << imu_data.angular_velocity.x << " "
	  //                << imu_data.angular_velocity.y << " "
	  //                << imu_data.angular_velocity.z << " "
	  //                << imu_data.quaternion.x << " "
	  //                << imu_data.quaternion.y << " "
	  //                << imu_data.quaternion.z << " "
	  //                << imu_data.quaternion.w << " "
	  //                << imu_data.eulers.roll << " "
	  //                << imu_data.eulers.pitch << " "
	  //                << imu_data.eulers.yaw;
	}
}

void DataReplay::GetGpsMsg(deque<string>& gps_data_que, deque<GpsData>& gps_msg)
{
	for (int i = 0; i < gps_data_que.size(); ++i) {
	  string gps_i = gps_data_que[i] + " ";
	  vector<string> gps_string_split = splitData(gps_i);
	  GpsData gps_data;
	  int idx = 0;
	  gps_data.time_stamp.sec = stringToNum<long long int>(gps_string_split[idx++]);
	  gps_data.time_stamp.nanosec = stringToNum<long long int>(gps_string_split[idx++]);
	  gps_data.frame_id = gps_string_split[idx++];
	  gps_data.message_id = gps_string_split[idx++];
	  gps_data.sol_status = gps_string_split[idx++];
	  gps_data.pos_type = gps_string_split[idx++];
	  gps_data.lat = stringToNum<double>(gps_string_split[idx++]);
	  gps_data.lon = stringToNum<double>(gps_string_split[idx++]);
	  gps_data.hgt = stringToNum<double>(gps_string_split[idx++]);
	  gps_data.undulation = stringToNum<float>(gps_string_split[idx++]);
	  gps_data.datum_id = gps_string_split[idx++];
	  gps_data.lat_sigma = stringToNum<float>(gps_string_split[idx++]);
	  gps_data.lon_sigma = stringToNum<float>(gps_string_split[idx++]);
	  gps_data.hgt_sigma = stringToNum<float>(gps_string_split[idx++]);
	  gps_data.stn_id = gps_string_split[idx++];
	  gps_data.diff_age = stringToNum<float>(gps_string_split[idx++]);
	  gps_data.sol_age = stringToNum<float>(gps_string_split[idx++]);
	  gps_data.svs = stringToNum<uint32_t>(gps_string_split[idx++]);
	  gps_data.soln_svs = stringToNum<uint32_t>(gps_string_split[idx++]);
	  gps_data.reserved_1 = stringToNum<uint32_t>(gps_string_split[idx++]);
	  gps_data.reserved_2 = stringToNum<uint32_t>(gps_string_split[idx++]);
	  gps_data.reserved_3 = stringToNum<uint32_t>(gps_string_split[idx++]);
	  gps_data.ext_sol_stat = stringToNum<int>(gps_string_split[idx++]);
	  gps_data.galileo_sig_mask = stringToNum<int>(gps_string_split[idx++]);
	  gps_data.gps_glonass_and_bds_sig_mask = stringToNum<int>(gps_string_split[idx++]);
	  gps_msg.push_back(gps_data);
	    //    std::cout << std::setiosflags(std::ios::fixed)
	    //              << setprecision(9)
	        //   std::cout       << gps_data.time_stamp << " "
	        //          << gps_data.frame_id << " "
	        //          << gps_data.message_id << " "
	        //          << gps_data.sol_status << " "
	        //          << gps_data.pos_type << " "
	        //          << gps_data.lat << " "
	        //          << gps_data.lon << " "
	        //          << gps_data.hgt << " "
	        //          << gps_data.undulation << " "
	        //          << gps_data.datum_id << " "
	        //          << gps_data.lat_sigma << " "
	        //          << gps_data.lon_sigma << " "
	        //          << gps_data.hgt_sigma << " "
	        //          << gps_data.stn_id << " "
	        //          << gps_data.diff_age << " "
	        //          << gps_data.sol_age << " "
	        //          << gps_data.svs << " "
	        //          << gps_data.soln_svs << " "
	        //          << gps_data.reserved_1 << " "
	        //          << gps_data.reserved_2 << " "
	        //          << gps_data.reserved_3 << " "
	        //          << gps_data.ext_sol_stat << " "
	        //          << gps_data.galileo_sig_mask << " "
	        //          << gps_data.gps_glonass_and_bds_sig_mask << std::endl;
	}
}

void DataReplay::GetGpsGpggaMsg(deque<string>& gps_data_que, deque<GpsGpggaData>& gps_msg)
{
	for (int i = 0; i < gps_data_que.size(); ++i) {
		string gps_i = gps_data_que[i] + " ";
		vector<string> gps_string_split = splitData(gps_i);
		GpsGpggaData gps_data;
		int idx = 0;
		gps_data.time_stamp.sec = stringToNum<long long int>(gps_string_split[idx++]);
		gps_data.time_stamp.nanosec = stringToNum<long long int>(gps_string_split[idx++]);
		gps_data.frame_id = gps_string_split[idx++];
		gps_data.utc_seconds = stringToNum<float>(gps_string_split[idx++]);
		gps_data.lat = stringToNum<double>(gps_string_split[idx++]);
		gps_data.lat_dir = gps_string_split[idx++];
		gps_data.lon = stringToNum<double>(gps_string_split[idx++]);
		gps_data.lon_dir = gps_string_split[idx++];
		gps_data.gps_qual = stringToNum<int>(gps_string_split[idx++]);
		gps_data.num_sats = stringToNum<int>(gps_string_split[idx++]);
		gps_data.hdop = stringToNum<float>(gps_string_split[idx++]);
		gps_data.alt = stringToNum<float>(gps_string_split[idx++]);
		gps_data.altitude_units = gps_string_split[idx++];
		gps_data.undulation = stringToNum<float>(gps_string_split[idx++]);
		gps_data.undulation_units = gps_string_split[idx++];
		gps_data.diff_age = stringToNum<int>(gps_string_split[idx++]);
		gps_msg.push_back(gps_data);
	}
}

void DataReplay::AllReplayRun()
{
	
	LoadData(mfImu, msImuFileData, mnImuFileLines, mqImuFileData);
	LoadData(mfOdom, msOdomFileData, mnOdomFileLines, mqOdomFileData);
	LoadData(mfGps, msGpsFileData, mnGpsFileLines, mqGpsFileData);
	if(mnOdomFileLines == 0 || mnImuFileLines == 0 || mnGpsFileLines == 0){
		ROS_ERROR("odom_lines = %d, imu_lines = %d, gps_lines = %d, please check it!", mnOdomFileLines, mnImuFileLines, mnGpsFileLines);
		return;
	}else{
		ROS_INFO("odom_lines = %d, imu_lines = %d, gps_lines = %d", mnOdomFileLines, mnImuFileLines, mnGpsFileLines);
	}

	GetImuMsg(mqImuFileData, mqImuMsg);
	GetOdomMsg(mqOdomFileData, mqOdomMsg);
	GetGpsMsg(mqGpsFileData, mqGpsMsg);

	auto less_than = [](const auto& e1, const auto& e2) {
      return (e1.sec * 1e9 + e1.nanosec <= e2.sec * 1e9 + e2.nanosec);
    };
    std::vector<TimeStamp> min_time_stamp;
    min_time_stamp.push_back(mqOdomMsg.front().time_stamp);
    min_time_stamp.push_back(mqImuMsg.front().time_stamp);
    min_time_stamp.push_back(mqGpsMsg.front().time_stamp);

    // odom cmd vel pub

	auto min_ele = std::min_element(std::begin(min_time_stamp), std::end(min_time_stamp), less_than);
	TimeStamp time_pass;
    auto t_start = std::chrono::steady_clock::now();
    auto t_epoch_start = t_start.time_since_epoch();
    auto start_pass_sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(t_epoch_start).count());
    auto start_pass_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_epoch_start - std::chrono::seconds(start_pass_sec)).count();
    int odom_idx = 0, imu_idx = 0, gps_idx = 0;
    bool odom_update = false, imu_update = false, gps_update = false;
    long long int odom_start_time = mqOdomMsg.front().time_stamp.sec * 1e9 +
                                    mqOdomMsg.front().time_stamp.nanosec;
    long long int odom_end_time = mqOdomMsg.back().time_stamp.sec * 1e9 +
                                  mqOdomMsg.back().time_stamp.nanosec;
    long long int imu_start_time = mqImuMsg.front().time_stamp.sec * 1e9 +
                                   mqImuMsg.front().time_stamp.nanosec;
    long long int imu_end_time = mqImuMsg.back().time_stamp.sec * 1e9 +
                                 mqImuMsg.back().time_stamp.nanosec;
    long long int gps_start_time = mqGpsMsg.front().time_stamp.sec * 1e9 +
                                   mqGpsMsg.front().time_stamp.nanosec;
    long long int gps_end_time = mqGpsMsg.back().time_stamp.sec * 1e9 +
                                 mqGpsMsg.back().time_stamp.nanosec;

    bool odom_start = false, imu_start = false, gps_start = false;

	while(ros::ok())
	{
		auto t = std::chrono::steady_clock::now();
      auto t_epoch = t.time_since_epoch();
      auto start_sleep_sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(t_epoch).count());
      auto start_sleep_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_epoch - std::chrono::seconds(start_sleep_sec)).count();
//      LOG(INFO) << "sec: " << start_sleep_sec
//                << " nanosec: " << start_sleep_nanosec;

      auto t_end = std::chrono::steady_clock::now();
      auto t_epoch_end = t_end.time_since_epoch();
      auto end_pass_sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(t_epoch_end).count());
      auto end_pass_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(t_epoch_end - std::chrono::seconds(end_pass_sec)).count();
      double secs = end_pass_sec * 1e9 + end_pass_nanosec - start_pass_sec * 1e9 - start_pass_nanosec;
      if (mbIsAcc) {
        secs *= mAccelerate;
      }
      time_pass.sec = floor(secs / 1e9);
      time_pass.nanosec = secs - time_pass.sec * 1e9;

//      LOG(INFO) << "time_pass.sec: " << time_pass.sec
//                << " time_pass.nanosec: " << time_pass.nanosec;

      long long int time_now = min_ele->sec * 1e9 + min_ele->nanosec + time_pass.sec * 1e9 + time_pass.nanosec;

	   if (odom_end_time >= time_now || imu_end_time >= time_now || gps_end_time >= time_now) {
		} else {
		// break ros::ok()
//        LOG(INFO) << "time_now.sec: " << time_now << " odom_idx: " << odom_idx
//                  << " imu_idx: " << imu_idx << " gps_idx: " << gps_idx;
        break;
      }

	  if (odom_idx < mqOdomMsg.size() && !odom_update &&
          mqOdomMsg[odom_idx].time_stamp.sec * 1e9 + mqOdomMsg[odom_idx].time_stamp.nanosec < time_now) {
        if (!odom_start) {
          odom_start = true;
        } else
          odom_idx++;
        odom_update = true;
        //      LOG(INFO) << "odom_update";
      }

      if (imu_idx < mqImuMsg.size() && !imu_update &&
          mqImuMsg[imu_idx].time_stamp.sec * 1e9 + mqImuMsg[imu_idx].time_stamp.nanosec < time_now) {
        if (!imu_start)
          imu_start = true;
        else
          imu_idx++;
        imu_update = true;
        //          LOG(INFO) << "imu_update";
      }

      if (gps_idx < mqGpsMsg.size() && !gps_update &&
          mqGpsMsg[gps_idx].time_stamp.sec * 1e9 + mqGpsMsg[gps_idx].time_stamp.nanosec < time_now) {
        if (!gps_start)
          gps_start = true;
        else
          gps_idx++;
        gps_update = true;
        //          LOG(INFO) << "gps_update";
      }

	  if (odom_update){
		if (odom_idx < mqOdomMsg.size()) {
			OdomData odom_data_pub = mqOdomMsg[odom_idx];
			nav_msgs::Odometry odom_message;
			SetOdomMsg(odom_message, odom_data_pub);
			PubMsg(odom_message, pub_odom);
			odom_update = false;
		}
	  }

	  if (imu_update){
		if (imu_idx < mqImuMsg.size()) {
			ImuData imu_data_pub = mqImuMsg[imu_idx];
			sensor_msgs::Imu imu_message;
			SetImuMsg(imu_message, imu_data_pub);
			PubMsg(imu_message, pub_imu);
			imu_update = false;
		}
	  }

	  if (gps_update){
		if (gps_idx < mqGpsMsg.size()) {
			GpsData gps_data_pub = mqGpsMsg[gps_idx];
			data_replay::Bestpos gps_message;
			SetGpsMsg(gps_message, gps_data_pub);
			// GpsGpggaData gps_data_pub = mqGpsMsg[gps_idx];
			// nmea_msgs::Gpgga gps_message;
			// SetGpsMsg(gps_message, gps_data_pub);
			PubMsg(gps_message, pub_gps_bestpos);
			gps_update = false;
		}
	  }
	  auto t1 = std::chrono::steady_clock::now();
      auto t1_epoch = t1.time_since_epoch();
      auto end_sleep_sec = static_cast<int32_t>(std::chrono::duration_cast<std::chrono::seconds>(t1_epoch).count());
      auto end_sleep_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(t1_epoch - std::chrono::seconds(end_sleep_sec)).count();

	if (!mbIsAcc) {
        long long int secs_sleep = (end_sleep_sec * 1e9 + end_sleep_nanosec - start_sleep_sec * 1e9 - start_sleep_nanosec);
        this_thread::sleep_for(chrono::nanoseconds(secs_sleep));
      } else {
        long long int secs_sleep = (end_sleep_sec * 1e9 + end_sleep_nanosec - start_sleep_sec * 1e9 - start_sleep_nanosec) / mAccelerate;
        this_thread::sleep_for(chrono::nanoseconds(secs_sleep));
      }
	}

	ROS_INFO("All data published");
	while(ros::ok()){ }
}