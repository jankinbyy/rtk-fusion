/**
****************************************************************************************

 * @CopyRight: 2020-2030, Positec Tech. CO.,LTD. All Rights Reserved.
 * @FilePath: dr_odo_flow.hpp
 * @Author: Taojing Zhang/张陶晶 (Positec CN) Taojing.Zhang@positecgroup.com
 * @Date: 2024-03-01 10:00:34
 * @Version: 0.1
 * @LastEditTime: 2024-03-01 10:00:35
 * @LastEditors: Taojing Zhang/张陶晶 (Positec CN)
Taojing.Zhang@positecgroup.com
 * @Description:

****************************************************************************************
*/
#ifndef SRC_POSE_SOURCE_DR_ODO2_FLOW_HPP_
#define SRC_POSE_SOURCE_DR_ODO2_FLOW_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <shared_mutex>
#include "sensor_type.h"

namespace DROdom2 {
/**
 * @brief fusion imu and odom class
 *
 */
class DrOdoFlow2 {
public:
    DrOdoFlow2(Eigen::Matrix4d IMU_T_WHEEL);

    ~DrOdoFlow2(){};

    void Optimization(); // handle sensor data

    void Reset(); // clear history state for reset
public:
    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);

    // set vehicle to imu extrinsic, can accelerate convergence
    void setRvi(const Eigen::Matrix<double, 3, 3> &Rvi);

    // set acc and gyro
    void setImu(double time_s, double ax, double ay, double az, double gx,
                double gy, double gz);

    void setWheel(double time_s, double vel_linear, double vel_angular);

    // get time and pose:x,y,z,r,p,y
    void getPose(sensor_msgs_z::OdometryData &pose);

private:
    bool is_start = true; // 定义是否开启程序
    int reset_flag_dro = false;
    Eigen::Matrix<double, 4, 4> Twv_p = Eigen::Matrix<double, 4, 4>::Identity();

private:
    sensor_msgs_z::IMUData processImu(double time_s, const Eigen::Matrix<double, 6, 1> imu,
                                      bool bStatic);

    void processWheel();

    void CalcStaticBias(const Eigen::Vector3d &gry);

    Eigen::Vector3d updateIMU(double gx, double gy, double gz, double ax,
                              double ay, double az, double dT);

    static double radiansToDegrees(double radians);

    std::mutex mImuDataMutex;
    std::mutex mWheelDataMutex;

    std::string mSaveLogPath;
    std::ofstream fTestWheelOdom;

    const double mKp = 1.0;
    const double mKi = 0.0001;
    double m_q0 = 1.0, m_q1 = 0.0, m_q2 = 0.0, m_q3 = 0.0;
    double mIntegralFBx = 0.0, mIntegralFBy = 0.0, mIntegralFBz = 0.0;
    Eigen::Matrix<double, 3, 3> mRviExt = Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> mRviEst = Eigen::Matrix<double, 3, 3>::Identity();
    bool mbGetRviExt = false;
    bool mbGetRviEst = false;

    //    std::list<Eigen::Vector3d> imuStaticW;
    Eigen::Vector3d mGyroBiasSum = Eigen::Vector3d(0., 0., 0.);
    int mGyroBiasNum = 0;
    Eigen::Vector3d mImuStaticBiasMean;

    double mLastImuTime = 0.;
    sensor_msgs_z::OdometryData Odom_;
    std::shared_timed_mutex get_pose_mutex_;

    std::deque<std::pair<double, Eigen::Matrix<double, 6, 1>>> mqIMUBuffer;
    std::deque<std::pair<double, Eigen::Vector2d>> mqWheelBuffer;

    double mXOdom = 0.;
    double mYOdom = 0.;
    Eigen::Matrix4d mTvi = Eigen::Matrix4d::Identity();

public:
    double mLastWheelTime = 0.;
    bool mPublishPoseFlag = false;
    Eigen::Matrix4d mTwi =
        Eigen::Matrix4d::Identity(); // imu pose in world coordina
};

#endif // SRC_POSE_SOURCE_DR_ODO2_FLOW_HPP_
}
