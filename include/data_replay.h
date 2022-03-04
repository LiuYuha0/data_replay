#include <string>
#include <vector>
#include <sstream>
#include <ros/ros.h>


using namespace std;

class DataReplay
{

public:
	DataReplay() {}

	~DataReplay() {}
	
	void SetImuMsg(sensor_msgs::Imu &msg, string &imu_data);

	void SetOdomMsg(sensor_msgs::Imu &msg, string &odom_data);

	void PubMsg(sensor_msgs::Imu &msg, ros::Publisher& pub_msg);

	void LoadData(ifstream& fTimes, string& data);

	void MainRun();

	string msOdomData, msImuData;

	ifstream mfImu, mfOdom, mfLclz;

	sensor_msgs::Imu mImuMsg;
	nav_msgs::Odometry mOdomMsg;

private:
	vector<string> splitData(string& input);

	template <typename Type>  
	Type stringToNum(const string &str);

    ros::NodeHandle* nh_;
};
