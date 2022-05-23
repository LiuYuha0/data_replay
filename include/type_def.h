#include <string>
#include <vector>

using namespace std;

struct Position {
  double x;
  double y;
  double z;
};

struct Quaternion {
  double x;
  double y;
  double z;
  double w;
};

struct Eulers {
  double roll;
  double pitch;
  double yaw;
};

struct TimeStamp {
  long long int sec;
  long long int nanosec;
};

struct OdomData {
  TimeStamp time_stamp;
  Position position;
  Quaternion quaternion;
  Eulers eulers;
};

struct LinearAcceleration {
  double x;
  double y;
  double z;
};

struct AngularVelocity {
  double x;
  double y;
  double z;
};

struct ImuData {
  TimeStamp time_stamp;
  LinearAcceleration linear_acceleration;
  AngularVelocity angular_velocity;
  Quaternion quaternion;
  Eulers eulers;
};

struct GpsData {
  TimeStamp time_stamp;
  std::string frame_id;
  std::string message_id;
  std::string sol_status;
  std::string pos_type;
  double lat;
  double lon;
  double hgt;
  float undulation;
  std::string datum_id;
  float lat_sigma;
  float lon_sigma;
  float hgt_sigma;
  std::string stn_id;
  float diff_age;
  float sol_age;
  uint32_t svs;
  uint32_t soln_svs;
  uint32_t reserved_1;
  uint32_t reserved_2;
  uint32_t reserved_3;
  int ext_sol_stat;
  int galileo_sig_mask;
  int gps_glonass_and_bds_sig_mask;
};

struct GpsGpggaData {
  TimeStamp time_stamp;
  std::string frame_id;
  float utc_seconds;
  double lat;
  std::string lat_dir;
  double lon;
  std::string lon_dir;
  int gps_qual;
  int num_sats;
  float hdop;
  float alt;
  std::string altitude_units;
  float undulation;
  std::string undulation_units;
  int diff_age;
};

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
