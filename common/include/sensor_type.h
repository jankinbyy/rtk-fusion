#ifndef SENSOR_TYPE_H
#define SENSOR_TYPE_H
#include "common.h"
namespace sensor_msgs_z {
class IMUData {
public:
    IMUData(double time_s, double ax, double ay, double az,
            double gx, double gy, double gz) :
        timestamp(time_s) {
        acc.x() = ax;
        acc.y() = ay;
        acc.z() = az;
        gyro.x() = gx;
        gyro.y() = gy;
        gyro.z() = gz;
    };
    IMUData(){};
    double timestamp;     // In second.
    Eigen::Vector3d acc;  // Acceleration in m/s^2
    Eigen::Vector3d gyro; // Angular velocity in radian/s.
    Eigen::Vector3d rpy;
    Eigen::Quaterniond orientation;
    bool efficient = false;

public:
    static bool SyncData(std::deque<IMUData> &UnsyncedData, std::deque<IMUData> &SyncedData, double sync_time) {
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().timestamp > sync_time) {
                return false;
            }
            //            允许两帧需要同步的数据都在雷达之前
            if (UnsyncedData.at(1).timestamp < sync_time && UnsyncedData.size() > 2) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().timestamp > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            if (UnsyncedData.at(1).timestamp - sync_time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            break;
        }
        if (UnsyncedData.size() < 2) {
            return false;
        }

        IMUData front_data = UnsyncedData.at(0);
        IMUData back_data = UnsyncedData.at(1);
        IMUData synced_data;

        double front_scale = (back_data.timestamp - sync_time) / (back_data.timestamp - front_data.timestamp);
        double back_scale = (sync_time - front_data.timestamp) / (back_data.timestamp - front_data.timestamp);
        synced_data.timestamp = sync_time;
        synced_data.acc = front_data.acc * front_scale + back_data.acc * back_scale;
        synced_data.gyro = front_data.gyro * front_scale + back_data.gyro * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x() = front_scale * front_data.orientation.x() + back_scale * back_data.orientation.x();
        synced_data.orientation.y() = front_scale * front_data.orientation.y() + back_scale * back_data.orientation.y();
        synced_data.orientation.z() = front_scale * front_data.orientation.z() + back_scale * back_data.orientation.z();
        synced_data.orientation.w() = front_scale * front_data.orientation.w() + back_scale * back_data.orientation.w();
        // 线性插值之后要归一化
        synced_data.orientation.normalized();

        SyncedData.push_back(synced_data);

        return true;
    };

    template <typename T>
    static IMUData transform(const IMUData &imu_in, const Eigen::Matrix<T, 4, 4> &imu_to_other) {
        IMUData imu_out = imu_in;
        Eigen::Matrix<T, 4, 4> extRot = imu_to_other.block(0, 0, 3, 3);
        // rotate acceleration
        Eigen::Matrix<T, 3, 1> acc(imu_in.acc.x(), imu_in.acc.y(),
                                   imu_in.acc.z());
        acc = extRot * acc;
        imu_out.acc.x() = acc.x();
        imu_out.acc.y() = acc.y();
        imu_out.acc.z() = acc.z();
        // rotate gyroscope
        Eigen::Matrix<T, 3, 1> gyr(imu_in.gyro.x(), imu_in.gyro.y(), imu_in.gyro.z());
        gyr = extRot * gyr;
        imu_out.gyro.x = gyr.x();
        imu_out.gyro.y = gyr.y();
        imu_out.gyro.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaternion<T> q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                    imu_in.orientation.z);
        Eigen::Quaternion<T> extQRPY(extRot);
        Eigen::Quaternion<T> q_final(extRot * q_from.toRotationMatrix() * extRot.transpose());

        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();
        return imu_out;
    };
};
using IMUDataPtr = std::shared_ptr<IMUData>;
class VelocityData {
public:
    VelocityData() {
        vel.x() = 0.0;
        vel.y() = 0.0;
        vel.z() = 0.0;
        gyro.x() = 0.0;
        gyro.y() = 0.0;
        gyro.z() = 0.0;
    }
    VelocityData(double time_s, double vel_linear,
                 double vel_angular) :
        timestamp(time_s) {
        vel.x() = vel_linear;
        vel.y() = 0.0;
        vel.z() = 0.0;
        gyro.x() = 0.0;
        gyro.y() = 0.0;
        gyro.z() = vel_angular;
    }
    double timestamp;    // In second.
    Eigen::Vector3d vel; //
    Eigen::Vector3d gyro;

public:
    static bool SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData,
                         double sync_time) {
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        bool diff_large = false;
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().timestamp > sync_time) return false;
            //            允许两帧需要同步的数据都在雷达之前
            if (UnsyncedData.at(1).timestamp < sync_time && UnsyncedData.size() > 2) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().timestamp > 0.5) {
                UnsyncedData.pop_front();
                diff_large = true;
                break;
            }
            if (UnsyncedData.at(1).timestamp - sync_time > 0.5) {
                UnsyncedData.pop_front();
                diff_large = true;
                break;
            }
            break;
        }

        VelocityData synced_data;
        if (UnsyncedData.size() < 2) {
            //            因为轮速计代码设计，长时间不返数，默认轮速为0
            if (diff_large) {
                synced_data.timestamp = sync_time;
                SyncedData.push_back(synced_data);
                return true;
            }
            return false;
        }

        VelocityData front_data = UnsyncedData.at(0);
        VelocityData back_data = UnsyncedData.at(1);
        if (back_data.timestamp - front_data.timestamp == 0.0) {
            UnsyncedData.pop_front();
            return false;
        }

        double front_scale = (back_data.timestamp - sync_time) / (back_data.timestamp - front_data.timestamp);
        double back_scale = (sync_time - front_data.timestamp) / (back_data.timestamp - front_data.timestamp);
        synced_data.timestamp = sync_time;
        synced_data.vel = front_data.vel * front_scale + back_data.vel * back_scale;
        synced_data.gyro = front_data.gyro * front_scale + back_data.gyro * back_scale;
        SyncedData.push_back(synced_data);

        return true;
    };

    void TransformCoordinate(Eigen::Matrix4d transform_matrix) {
        Eigen::Matrix4d matrix = transform_matrix.cast<double>();
        Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d w = gyro;
        Eigen::Vector3d v = vel;
        w = t_R * w;
        v = t_R * v;
        Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
        Eigen::Vector3d delta_v;
        delta_v(0) = w(1) * r(2) - w(2) * r(1);
        delta_v(1) = w(2) * r(0) - w(0) * r(2);
        delta_v(2) = w(0) * r(1) - w(1) * r(0);
        v = v - delta_v;
        gyro = w;
        vel = v;
    };

    VelocityData operator+(const VelocityData &v) {
        VelocityData vel;
        vel.timestamp = this->timestamp + v.timestamp;
        vel.vel = this->vel + v.vel;
        vel.gyro = this->gyro + v.gyro;
        return vel;
    }

    template <typename T>
    VelocityData operator/(const T &cnt) {
        VelocityData vel;
        vel.vel = this->vel / cnt;
        vel.gyro = this->gyro / cnt;
        return vel;
    }

    template <typename T>
    VelocityData operator*(const T &cnt) {
        VelocityData vel;
        vel.vel = this->vel * cnt;
        vel.gyro = this->gyro * cnt;
        return vel;
    }
};
using VelocityDataPtr = std::shared_ptr<VelocityData>;
struct GnssData {
public:
    double timestamp = 0;
    Eigen::Vector3d LonLatAlt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpy = {0.0, 0.0, -1000.0};
    int status = -1;
    int service = -1;
    Eigen::Matrix3d cov;
    GnssData(){};
    GnssData &operator=(const GnssData &b) {
        this->timestamp = b.timestamp;
        this->LonLatAlt = b.LonLatAlt;
        this->xyz = b.xyz;
        this->rpy = b.rpy;
        this->status = b.status;
        this->service = b.service;
        return *this;
    }
    GnssData(double _t, double _lon, double _lat, double _alt, int _stat, int _ser) :
        timestamp(_t),
        status(_stat),
        service(_ser) {
        LonLatAlt << _lon, _lat, _alt;
    }
    GnssData(double _t, double _lon, double _lat, double _alt, double _x, double _y, double _z, int _stat, int _ser) :
        timestamp(_t),
        status(_stat),
        service(_ser) {
        xyz << _x, _y, _z;
        LonLatAlt << _lon, _lat, _alt;
    }
};
using GnssDataPtr = std::shared_ptr<GnssData>;

class OdometryData {
public:
    double time = 0.;
    std::string topic = "";
    Eigen::Vector3d position = {0, 0, 0};
    Eigen::Quaterniond rotation;
    Eigen::Vector3d twist;
    Eigen::Vector3d gyro;
    Eigen::Vector3d acc;
    Eigen::Vector3d rpy = {0.0, 0.0, 0.0}; //rol pitch yaw
    Eigen::Matrix3d R_Mat;
    double covariance[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //x y z,roll,pitch,yaw
    OdometryData(){};
    OdometryData(double _t, Eigen::Vector3d _p, Eigen::Quaterniond _q) :
        time(_t),
        position(_p), rotation(_q) {
    }
    OdometryData(double _t, std::string _topic, Eigen::Vector3d _p,
                 Eigen::Quaterniond _q) :
        time(_t),
        topic(_topic), position(_p), rotation(_q) {
        rpy = VecToEulerAngles(rotation);
        R_Mat = rotation.toRotationMatrix();
    }
    OdometryData(double _t, Eigen::Vector3d _p, Eigen::Quaterniond _q,
                 Eigen::Vector3d _v) :
        time(_t),
        position(_p), rotation(_q), twist(_v) {
    }
    OdometryData(double _t, std::string _topic, Eigen::Vector3d _p,
                 Eigen::Quaterniond _q, Eigen::Vector3d _v) :
        time(_t),
        topic(_topic),
        position(_p),
        rotation(_q),
        twist(_v) {
    }
    OdometryData &operator=(const OdometryData &b) {
        this->time = b.time;
        this->topic = b.topic;
        this->position = b.position;
        this->rpy = b.rpy;
        this->rotation = b.rotation;
        this->twist = b.twist;
        this->gyro = b.gyro;
        this->acc = b.acc;
        this->R_Mat = b.R_Mat;
        return *this;
    }
};
using OdometryDataPtr = std::shared_ptr<OdometryData>;

struct State {
    double timestamp;

    Eigen::Vector3d lla; // WGS84 position.
    Eigen::Vector3d
        G_p_I;             // The original point of the IMU frame in the Global frame.
    Eigen::Vector3d G_v_I; // The velocity original point of the IMU frame in the
                           // Global frame.
    Eigen::Matrix3d
        G_R_I;                 // The rotation from the IMU frame to the Global frame.
    Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.
    // Covariance.
    Eigen::Matrix<double, 15, 15> cov;
};

struct Gnss_With_DrOdom {
    int status = -1; // 0:not align to odom axis  1:angle not filter 2:angle filtered
    double time;
    GnssData gnss_data;
    OdometryData ref_drOdom;
    Gnss_With_DrOdom() {
        time = 0.;
    }
};

enum SLAMMODE {
    IDLE = 0,
    MAPPING = 1,
    EDGE_LEARNING = 2,
    LOCALIZATION = 3,
    ISLANDS = 4,
    CHANNELS = 5,
    MULTI_AREA = 6, // 地图续建(通道+续建)
};

enum CONTOURTYPE {
    UNKNOW = 0,
    CUTAREA = 1,
    CHANNEL = 2,
    ISOLATED_ISLAND = 3,
};
struct ContourInfo {
    CONTOURTYPE contour_type;
    int id;
    std::vector<Eigen::Vector3d> contour_points;
    ContourInfo() {
        contour_type = UNKNOW;
        id = -1;
        contour_points.clear();
    }
};

typedef struct WORKAREA {
    int id;
    std::vector<ContourInfo> contour_infos;
    std::vector<int> connected_channels;
    WORKAREA() {
        id = -1;
        contour_infos.clear();
        connected_channels.clear();
    }
} WorkArea;
} // namespace sensor_msgs_z

#endif
