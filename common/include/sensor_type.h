#ifndef SENSOR_TYPE_H
#define SENSOR_TYPE_H
#include "common.h"
namespace sensor_msgs_z {
class IMUData {
public:
    struct LinearAcceleration {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct RPY {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    class Orientation {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;

    public:
        void Normlize() {
            double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }

        RPY getRPY() {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d rx = q.toRotationMatrix();
            Eigen::Vector3d ea = rx.eulerAngles(2, 1, 0);
            double roll, pitch, yaw;
            toEulerAngle(q, roll, pitch, yaw);
            RPY rpy;
            rpy.roll = roll;
            rpy.pitch = pitch;
            rpy.yaw = yaw;
            return rpy;
        }

        void Construct(Eigen::Quaterniond &quat) {
            x = quat.x();
            y = quat.y();
            z = quat.z();
            w = quat.w();
            Normlize();
        }

        Eigen::Matrix3d getRoation() {
            Eigen::Quaterniond q(w, x, y, z);
            Eigen::Matrix3d rx = q.toRotationMatrix();
            return rx;
        }
    };
    IMUData(double time_s, double ax, double ay, double az,
            double gx, double gy, double gz) :
        time(time_s) {
        linear_acceleration.x = ax;
        linear_acceleration.y = ay;
        linear_acceleration.z = az;
        angular_velocity.x = gx;
        angular_velocity.y = gy;
        angular_velocity.z = gz;
    };
    IMUData(){};
    double timestamp;     // In second.
    Eigen::Vector3d acc;  // Acceleration in m/s^2
    Eigen::Vector3d gyro; // Angular velocity in radian/s.
    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    //        geometry_msgs::Quaternion_ <std::allocator<void>> orientation;
    Orientation orientation;

public:
    // 把四元数转换成旋转矩阵送出去
    Eigen::Matrix3f GetOrientationMatrix() {
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f matrix = q.matrix().cast<float>();
        return matrix;
    };

    static bool SyncData(std::deque<IMUData> &UnsyncedData, std::deque<IMUData> &SyncedData, double sync_time) {
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().time > sync_time) {
                return false;
            }
            //            允许两帧需要同步的数据都在雷达之前
            if (UnsyncedData.at(1).time < sync_time && UnsyncedData.size() > 2) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().time > 0.2) {
                UnsyncedData.pop_front();
                break;
            }
            if (UnsyncedData.at(1).time - sync_time > 0.2) {
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

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
        synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
        synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
        // 四元数插值有线性插值和球面插值，球面插值更准确，但是两个四元数差别不大是，二者精度相当
        // 由于是对相邻两时刻姿态插值，姿态差比较小，所以可以用线性插值
        synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
        synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
        synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
        synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;
        // 线性插值之后要归一化
        synced_data.orientation.Normlize();

        SyncedData.push_back(synced_data);

        return true;
    };

    template <typename T>
    static IMUData transform(const IMUData &imu_in, const Eigen::Matrix<T, 4, 4> &imu_to_other) {
        IMUData imu_out = imu_in;
        Eigen::Matrix<T, 4, 4> extRot = imu_to_other.block(0, 0, 3, 3);
        // rotate acceleration
        Eigen::Matrix<T, 3, 1> acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y,
                                   imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Matrix<T, 3, 1> gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
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
        linear_velocity.x = 0.0;
        linear_velocity.y = 0.0;
        linear_velocity.z = 0.0;
        angular_velocity.x = 0.0;
        angular_velocity.y = 0.0;
        angular_velocity.z = 0.0;
    }
    VelocityData(double time_s, double vel_linear,
                 double vel_angular) :
        time(time_s) {
        linear_velocity.x = vel_linear;
        linear_velocity.y = 0.0;
        linear_velocity.z = 0.0;
        angular_velocity.x = 0.0;
        angular_velocity.y = 0.0;
        angular_velocity.z = vel_angular;
    }

    struct LinearVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
    double timestamp;    // In second.
    Eigen::Vector3d vel; //
    Eigen::Vector3d gyr;

public:
    static bool SyncData(std::deque<VelocityData> &UnsyncedData, std::deque<VelocityData> &SyncedData,
                         double sync_time) {
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        bool diff_large = false;
        while (UnsyncedData.size() >= 2) {
            if (UnsyncedData.front().time > sync_time) return false;
            //            允许两帧需要同步的数据都在雷达之前
            if (UnsyncedData.at(1).time < sync_time && UnsyncedData.size() > 2) {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().time > 0.5) {
                UnsyncedData.pop_front();
                diff_large = true;
                break;
            }
            if (UnsyncedData.at(1).time - sync_time > 0.5) {
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
                synced_data.time = sync_time;
                SyncedData.push_back(synced_data);
                return true;
            }
            return false;
        }

        VelocityData front_data = UnsyncedData.at(0);
        VelocityData back_data = UnsyncedData.at(1);
        if (back_data.time - front_data.time == 0.0) {
            UnsyncedData.pop_front();
            return false;
        }

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.linear_velocity.x = front_data.linear_velocity.x * front_scale + back_data.linear_velocity.x * back_scale;
        synced_data.linear_velocity.y = front_data.linear_velocity.y * front_scale + back_data.linear_velocity.y * back_scale;
        synced_data.linear_velocity.z = front_data.linear_velocity.z * front_scale + back_data.linear_velocity.z * back_scale;
        synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
        synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
        synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;
        SyncedData.push_back(synced_data);

        return true;
    };

    void TransformCoordinate(Eigen::Matrix4d transform_matrix) {
        Eigen::Matrix4d matrix = transform_matrix.cast<double>();
        Eigen::Matrix3d t_R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d w(angular_velocity.x, angular_velocity.y, angular_velocity.z);
        Eigen::Vector3d v(linear_velocity.x, linear_velocity.y, linear_velocity.z);
        w = t_R * w;
        v = t_R * v;
        Eigen::Vector3d r(matrix(0, 3), matrix(1, 3), matrix(2, 3));
        Eigen::Vector3d delta_v;
        delta_v(0) = w(1) * r(2) - w(2) * r(1);
        delta_v(1) = w(2) * r(0) - w(0) * r(2);
        delta_v(2) = w(0) * r(1) - w(1) * r(0);
        v = v - delta_v;

        angular_velocity.x = w(0);
        angular_velocity.y = w(1);
        angular_velocity.z = w(2);
        linear_velocity.x = v(0);
        linear_velocity.y = v(1);
        linear_velocity.z = v(2);
    };

    VelocityData operator+(const VelocityData &v) {
        VelocityData vel;
        vel.time = this->time + v.time;
        vel.linear_velocity.x = this->linear_velocity.x + v.linear_velocity.x;
        vel.linear_velocity.y = this->linear_velocity.y + v.linear_velocity.y;
        vel.linear_velocity.z = this->linear_velocity.z + v.linear_velocity.z;
        vel.angular_velocity.x = this->angular_velocity.x + v.angular_velocity.x;
        vel.angular_velocity.y = this->angular_velocity.y + v.angular_velocity.y;
        vel.angular_velocity.z = this->angular_velocity.z + v.angular_velocity.z;
        return vel;
    }

    template <typename T>
    VelocityData operator/(const T &cnt) {
        VelocityData vel;
        vel.time = this->time / cnt;
        vel.linear_velocity.x = this->linear_velocity.x / cnt;
        vel.linear_velocity.y = this->linear_velocity.y / cnt;
        vel.linear_velocity.z = this->linear_velocity.z / cnt;
        vel.angular_velocity.x = this->angular_velocity.x / cnt;
        vel.angular_velocity.y = this->angular_velocity.y / cnt;
        vel.angular_velocity.z = this->angular_velocity.z / cnt;
        return vel;
    }

    template <typename T>
    VelocityData operator*(const T &cnt) {
        VelocityData vel;
        vel.time = this->time * cnt;
        vel.linear_velocity.x = this->linear_velocity.x * cnt;
        vel.linear_velocity.y = this->linear_velocity.y * cnt;
        vel.linear_velocity.z = this->linear_velocity.z * cnt;
        vel.angular_velocity.x = this->angular_velocity.x * cnt;
        vel.angular_velocity.y = this->angular_velocity.y * cnt;
        vel.angular_velocity.z = this->angular_velocity.z * cnt;
        return vel;
    }

    Eigen::Vector3d as_vector() {
        return Eigen::Vector3d(linear_velocity.x, linear_velocity.y, linear_velocity.z);
    }
};
using VelocityDataPtr = std::shared_ptr<VelocityData>;
class PubPos {
public:
    double time;
    double x;
    double y;
    double z;
    Eigen::Quaterniond q;
};
struct GnssPosData {
    double time; //s
    double x, y, z;
    double rol;
    double pitch;
    double yaw;
    GnssPosData() {
        time = 0.;
    }
};

struct GnssData {
public:
    double time = 0;
    double longitude;
    double latitude;
    double altitude;
    double x = 0, y = 0, z = 0;
    double rol;
    double pitch;
    double yaw = -1000;
    int status;
    int service;
    Eigen::Matrix3d cov;
    GnssData(){};
    GnssData &operator=(const GnssData &b) {
        this->time = b.time;
        this->altitude = b.altitude;
        this->longitude = b.longitude;
        this->latitude = b.latitude;
        this->x = b.x;
        this->y = b.y;
        this->z = b.z;
        this->yaw = b.yaw;
        this->status = b.status;
        this->service = b.service;
        return *this;
    }
    GnssData(double _t, double _lon, double _lat, double _alt, int _stat, int _ser) :
        time(_t),
        longitude(_lon),
        latitude(_lat),
        altitude(_alt),
        status(_stat),
        service(_ser) {
    }
    GnssData(double _t, double _lon, double _lat, double _alt, double _x, double _y, double _z, int _stat, int _ser) :
        time(_t),
        longitude(_lon),
        latitude(_lat),
        altitude(_alt),
        x(_x),
        y(_y),
        z(_z),
        status(_stat),
        service(_ser) {
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
    Eigen::Vector3d angle_xyz = {0.0, 0.0, 0.0}; //rol pitch yaw
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
        angle_xyz = VecToEulerAngles(rotation);
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

    // The imu data.
    IMUDataPtr imu_data_ptr;
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
