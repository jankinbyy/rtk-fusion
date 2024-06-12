
#ifndef SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_
#define SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_
#include <yaml-cpp/yaml.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>
#include "Estimator.h"
class DrOdoFlow {
public:
    DrOdoFlow(Eigen::Matrix4d imu_t_wheel) :
        IMU_T_WHEEL_(imu_t_wheel) {
        std::cout << "IMU_T_WHEEL:" << IMU_T_WHEEL_ << std::endl;
        const double gyro_noise = 1e-6;      // default  1e-6
        const double gyro_bias_noise = 1e-8; // default 1e-8
        const double acc_noise = 1.0;        // default 1e-2
        orientation_estimator = std::make_shared<Estimator>(gyro_noise, gyro_bias_noise, acc_noise);
        is_start = true;
        reset_flag_dro = 0;
    }
    void Optimization();
    void setImu(double time_s, double ax, double ay, double az,
                double gx, double gy, double gz);
    void setWheel(double time_s, double vel_linear,
                  double vel_angular);
    void ReadData(IMUData &input_imu, VelocityData &in_wheel);
    void ReadData(IMUData &input_imu);
    void ReadData(VelocityData &in_wheel);
    void updateDrOdo();

    Eigen::Matrix4d Fusing_IMU_VelOdom(IMUData imu_front, IMUData imu_back);

    void WheelVel2IMUVel(VelocityData &vel_data, const Eigen::Vector3d &ang_vel);
    Eigen::Matrix4d GetPos();
    Eigen::Matrix4d GetDeltaPos();
    void getPose(OdometryData &pose);
    void Start();
    void Stop();

    void clear_state_dro();

    Eigen::Vector3d updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time,
                            const Eigen::Vector3d &ang_vel);
    Eigen::Vector3d updateU(std::deque<VelocityData> &unsynced_velocity, double front_time, double back_time);
    std::shared_ptr<Estimator> orientation_estimator;
    Eigen::Matrix4d Twi = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d IMU_T_WHEEL_;

private:
    // Info Container
    std::deque<IMUData> raw_imu_;
    std::deque<IMUData> unsynced_imu_;
    std::deque<VelocityData> unsynced_velocity_;
    VelocityData nonZeroLast;
    // Visualization
    bool is_start; // 定义是否开启程序
    int reset_flag_dro;
    std::mutex pose_mtx_;
    Eigen::Matrix4d T_delta_;
    bool run_alg_ = true;
    OdometryData pub_odomtry_;
};

#endif // SRC_POSE_SOURCE_DR_ODO_FLOW_HPP_