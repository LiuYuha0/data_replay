#include <string>
#include <vector>
#include <sstream>
#include <sys/time.h>
#include <fstream>
#include <thread>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Gpgga.h>

using namespace std;

extern ros::Publisher pub_imu, pub_odom, pub_gps;

template<typename T>
static inline T norm_value(T x, T lb, T ub)
{
  T a = fmod(x - lb, ub - lb);
  if (a < 0) {
    a += (ub - lb);
  }
  return a + lb;
}

#define norm_angle(x) norm_value(double(x), -M_PI, M_PI)


class DataReplay
{

public:
	DataReplay();

	~DataReplay() {}
	
	void init();

	void MainRun();

	int mnRate;
	bool mbImuReplay, mbImu2Replay, mbOdomReplay, mbGpsReplay, mbLocalizationReplay, mbAllReplay= false;
	bool mbShowRPY = false;
	string msImuPath, msImu2Path, msOdomPath, msGpsPath, msLocalizationPath;


private:
	void LoadData(ifstream& fTimes, string& data);

	void SetImuMsg(sensor_msgs::Imu &msg, string &imu_data);

	void SetOdomMsg(nav_msgs::Odometry &msg, string &odom_data);

	void SetGpsmMsg(nmea_msgs::Gpgga &msg, string &gps_data);

	void SetOdomLclzAsImuMsg(sensor_msgs::Imu &msg, string &odom_data);

	void PubMsg(sensor_msgs::Imu &msg, ros::Publisher& pub_msg);

	void PubMsg(nav_msgs::Odometry &msg, ros::Publisher& pub_msg);

	void PubMsg(nmea_msgs::Gpgga &msg, ros::Publisher& pub_msg);

	void ImuReplayRun();

	void OdomReplayRun();

	void AllReplayRun();

	void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

	vector<string> splitData(string& input);

	template <typename Type>  
	Type stringToNum(const string &str);

	sensor_msgs::Imu mImuMsg;
	nav_msgs::Odometry mOdomMsg;
	nmea_msgs::Gpgga mGpsMsg;

	bool mbFirstFlag = true;

	ifstream mfImu, mfOdom, mfGps, mfLclz;
	string msImuData, msOdomData, msGpsData;

	long long int maImuCurTimeStamp[2], maImuNextTimeStamp[2], maOdomTimeStamp[2], maGpsTimeStamp[2];
	long long int mnImuTimeStamp, mnOdomTimeStamp, mnGpsTimeStamp, mnSleepTime;
	vector<double> mvImuCurData, mvImuNextData;
	
    ros::NodeHandle* nh_;
};



