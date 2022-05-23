#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Gpgga.h>
#include <data_replay/Bestpos.h>
#include "type_def.h"

#include <string>
#include <vector>
#include <sys/time.h>
#include <chrono>
#include <fstream>
#include <thread>
#include <deque>

using namespace std;

extern ros::Publisher pub_imu, pub_odom, pub_gps_bestpos, pub_gps_gpgga, pub_path;

class DataReplay
{

public:
	DataReplay();
	~DataReplay() {}
	void init();
	void MainRun();

	int mnRate;
	bool mbImuReplay = false, mbImu2Replay = false, mbOdomReplay = false, mbGpsReplay = false, mbLocalizationReplay = false, mbAllReplay = false;
	bool mbShowRPY = false;
	std::string msImuPath, msImu2Path, msOdomPath, msGpsPath, msLocalizationPath, mLogFilePathPredix;
	//acc 
	bool mbIsAcc = false;
	int mAccelerate;

private:
	//select
	// void ImuReplayRun();
	// void OdomReplayRun();
	// void LocalizationPathReplayRun();
	void AllReplayRun();

	//Load data
	void LoadData(ifstream& fTimes, string& data);
	void LoadData(ifstream& fTimes, string& fileData, int& fileLine, std::deque<string>& dataQue);

	//Get msg
	void GetOdomMsg(deque<string>& odom_data_que, deque<OdomData>& odom_msg);
	void GetImuMsg(deque<string>& imu_data_que, deque<ImuData>& imu_msg);
	void GetGpsMsg(deque<string>& gps_data_que, deque<GpsData>& gps_msg);
	void GetGpsGpggaMsg(deque<string>& gps_data_que, deque<GpsGpggaData>& gps_msg);

	//Set msg
	void SetImuMsg(sensor_msgs::Imu &msg, ImuData &imu_data_pub);
	void SetOdomMsg(nav_msgs::Odometry &msg, OdomData odom_data_pub);
	void SetGpsMsg(nmea_msgs::Gpgga &msg, GpsGpggaData gps_data_pub);
	void SetGpsMsg(data_replay::Bestpos &msg, GpsData gps_data_pub);
	void setLocalizationAsPathMsg(nav_msgs::Path &msg, string &path_data);

	//Pub msg
	void PubMsg(sensor_msgs::Imu &msg, ros::Publisher& pub_msg);
	void PubMsg(nav_msgs::Odometry &msg, ros::Publisher& pub_msg);
	void PubMsg(nmea_msgs::Gpgga &msg, ros::Publisher& pub_msg);
	void PubMsg(data_replay::Bestpos &msg, ros::Publisher& pub_msg);
	void PubMsg(nav_msgs::Path &msg, ros::Publisher& pub_msg);

	//Log
	void SetLog(std::string &log);
	void RecordData(const std::string& logFile, sensor_msgs::Imu &msg);
	void RecordData(const std::string& logFile, nav_msgs::Odometry &msg);
	void RecordData(const std::string& logFile, nmea_msgs::Gpgga &msg);
	void RecordData(nav_msgs::Odometry &msg);

	void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
	std::vector<string> splitData(string& input);
	template <typename Type>  
	Type stringToNum(const string &str);

	bool mbFirstFlag = true;

	//imu
	ifstream mfImu;
	std::string msImuFileData;
	int mnImuFileLines = 0;
	std::deque<string> mqImuFileData;
	std::deque<ImuData> mqImuMsg;
	std::string	mImuLogFileName = "imu";
	std::ofstream mImuLogOfstream;

	//odom
	ifstream mfOdom;
	std::string msOdomFileData;
	int mnOdomFileLines = 0;
	std::deque<string> mqOdomFileData;
	std::deque<OdomData> mqOdomMsg;
	std::string	mOdomLogFileName = "odom";
	std::ofstream mOdomLogOfstream;

	//gps
	ifstream mfGps;
	std::string msGpsFileData;
	int mnGpsFileLines = 0;
	std::deque<string> mqGpsFileData;
	std::deque<GpsData> mqGpsMsg;
	std::deque<GpsGpggaData> mqGpsGpggaMsg;
	std::string	mGpsLogFileName = "gps";
	std::ofstream mGpsLogOfstream;
	
	//path
	ifstream mfLocalization;
	std::string msLocalizationFileData;
	
    ros::NodeHandle* nh_;
};



