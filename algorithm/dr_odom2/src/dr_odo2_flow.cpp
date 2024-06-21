/**
****************************************************************************************

 * @CopyRight: 2020-2030, Positec Tech. CO.,LTD. All Rights Reserved.
 * @FilePath: dr_odo2_flow.cpp
 * @Author: Taojing Zhang/张陶晶 (Positec CN) Taojing.Zhang@positecgroup.com
 * @Date: 2024-03-01 09:52:00
 * @Version: 0.1
 * @LastEditTime: 2024-03-01 09:52:02
 * @LastEditors: Taojing Zhang/张陶晶 (Positec CN)
Taojing.Zhang@positecgroup.com
 * @Description:

****************************************************************************************
*/
#include "dr_odom2_flow.hpp"

namespace DROdom2 {

using namespace std;

DrOdoFlow2::DrOdoFlow2(Eigen::Matrix4d IMU_T_WHEEL) :
    mTwi(Eigen::Matrix4d::Identity()) {
    mTvi = IMU_T_WHEEL;
    imu_t_wheel = IMU_T_WHEEL;
}

void DrOdoFlow2::Optimization() {
    // check if reset current node
    if (reset_flag_dro) {
        Reset(); // clear state
        reset_flag_dro = false;
    } else // if (is_start)
    {
        // handle sensor data
        processWheel();
    }
}

void DrOdoFlow2::Reset() { //  清空 DR_ODO 的所有状态
    mTwi = Eigen::Matrix4d::Identity();

    m_q0 = 1.0, m_q1 = 0.0, m_q2 = 0.0, m_q3 = 0.0;
    mIntegralFBx = 0.0, mIntegralFBy = 0.0, mIntegralFBz = 0.0;
    mRviExt = Eigen::Matrix<double, 3, 3>::Identity();
    mRviEst = Eigen::Matrix<double, 3, 3>::Identity();
    mbGetRviExt = false;
    mbGetRviEst = false;

    //    std::list<Eigen::Vector3d> imuStaticW;
    mGyroBiasSum = Eigen::Vector3d(0., 0., 0.);
    mGyroBiasNum = 0;
    mImuStaticBiasMean = Eigen::Vector3d(0., 0., 0.);

    mLastImuTime = 0.;
    //mRPYLast = Eigen::Vector3d(0., 0., 0.);

    mqIMUBuffer.clear();
    mqWheelBuffer.clear();

    mXOdom = 0.;
    mYOdom = 0.;
    mLastWheelTime = 0.;
    mPublishPoseFlag = false;
}

// set vehicle to imu extrinsic, can accelerate convergence
void DrOdoFlow2::setRvi(const Eigen::Matrix<double, 3, 3> &Rvi) {
    mRviExt = Rvi;
    mbGetRviExt = true;
}

void DrOdoFlow2::WheelVel2IMUVel(Eigen::Vector3d &vel_data, const Eigen::Vector3d &ang_vel) { // ang_vel measured by IMU, and is expressed in IMU coordinate
    // If all expressed in IMU coordinate, v_IMU = v_Wheel + ang_vel × r_Wheel_IMU
    Eigen::Matrix3d R_IMU_Wheel = imu_t_wheel.block<3, 3>(0, 0);
    Eigen::Vector3d r_Wheel_IMU = R_IMU_Wheel * imu_t_wheel.inverse().block<3, 1>(0, 3); // expressed in IMU coordinate ￥imu_to_wheel_
    Eigen::Vector3d v_Wheel = vel_data;                                                  // expressed in Wheel coordinate
    Eigen::Vector3d v_IMU = R_IMU_Wheel * v_Wheel + ang_vel.cross(r_Wheel_IMU);
    vel_data = v_IMU;
}
Eigen::Matrix3d DrOdoFlow2::eulerAnglesToRotationMatrix(
    Eigen::Vector3d &theta) {
    Eigen::Matrix3d R_x; // 计算旋转矩阵的X分量
    R_x << 1, 0, 0, 0, cos(theta[0]), -sin(theta[0]), 0, sin(theta[0]),
        cos(theta[0]);

    Eigen::Matrix3d R_y; // 计算旋转矩阵的Y分量
    R_y << cos(theta[1]), 0, sin(theta[1]), 0, 1, 0, -sin(theta[1]), 0,
        cos(theta[1]);

    Eigen::Matrix3d R_z; // 计算旋转矩阵的Z分量
    R_z << cos(theta[2]), -sin(theta[2]), 0, sin(theta[2]), cos(theta[2]), 0, 0,
        0, 1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

void DrOdoFlow2::getPose(sensor_msgs_z::OdometryData &pose) {
    {
        std::unique_lock<std::shared_timed_mutex> lock(get_pose_mutex_);
        pose = Odom_;
    }
}

void DrOdoFlow2::setImu(double time_s, double ax, double ay, double az,
                        double gx, double gy, double gz) {
    {
        std::unique_lock<std::mutex> lock(mImuDataMutex);
        Eigen::Matrix<double, 6, 1> imu;
        imu << ax, ay, az, gx, gy, gz;
        mqIMUBuffer.emplace_back(time_s, imu);
        while (mqIMUBuffer.size() > 100) {
            mqIMUBuffer.pop_front();
        }
    }
}

void DrOdoFlow2::setWheel(double time_s, double vel_linear,
                          double vel_angular) {
    {
        std::unique_lock<std::mutex> lock(mWheelDataMutex);
        mqWheelBuffer.emplace_back(time_s,
                                   Eigen::Vector2d(vel_linear, vel_angular));
        while (mqWheelBuffer.size() > 100) {
            mqWheelBuffer.pop_front();
        }
    }
}

// roll pitch yaw,anguler speed
sensor_msgs_z::IMUData DrOdoFlow2::processImu(double time_s,
                                              const Eigen::Matrix<double, 6, 1> imu,
                                              bool bStatic) {
    sensor_msgs_z::IMUData tmp_imu_data;
    tmp_imu_data.efficient = false;
    auto t1 = std::chrono::steady_clock::now();
    Eigen::Vector3d acc = imu.head(3);
    Eigen::Vector3d gyro = imu.tail(3); // * ratio,

    static int logCnt = 0;
    logCnt++;
    {
        if (bStatic && (fabs(gyro(0)) < 0.01) && (fabs(gyro(1)) < 0.01) && (fabs(gyro(2)) < 0.01) && (mGyroBiasNum < 20000)) {
            CalcStaticBias(gyro);
        }

        if (bStatic && (fabs(gyro(0)) < 0.01) && (fabs(gyro(1)) < 0.01) && (fabs(gyro(2)) < 0.01)) {
            gyro = Eigen::Vector3d(0., 0., 0.);
        } else if (mGyroBiasNum > 50) {
            gyro = gyro - mImuStaticBiasMean; // mImuStaticBiasMean.wx;
        }
    }

    if (mbGetRviExt) {
        gyro = (mRviExt * gyro).eval();
        acc = (mRviExt * acc).eval();
    } else {
        if (mbGetRviEst) {
            gyro = (mRviEst * gyro).eval();
            acc = (mRviEst * acc).eval();
        }
    }
    tmp_imu_data.gyro = gyro;
    tmp_imu_data.acc = acc;
    if (mLastImuTime > 0.01 && (time_s - mLastImuTime < 0.2)) {
        Eigen::Vector3d mRPYLast = updateIMU(gyro(0), gyro(1), gyro(2), acc(0), acc(1), acc(2),
                                             time_s - mLastImuTime);
        tmp_imu_data.rpy = mRPYLast;
        tmp_imu_data.efficient = true;
    } else if (!mbGetRviEst) {
        Eigen::Quaterniond quat;
        quat.setFromTwoVectors(Eigen::Vector3d(acc(0), acc(1), acc(2)),
                               Eigen::Vector3d(0, 0, 9.8082));
        mRviEst = Eigen::Matrix3d(quat);
        mbGetRviEst = true;
    } else {
        cerr << "big IMU Time diff " << time_s - mLastImuTime << "s" << endl;
    }

    mLastImuTime = time_s;
    return tmp_imu_data;
}

void DrOdoFlow2::processWheel() {
    double wheel_before_time = 0.0;
    double imu_back_time = 0.0;
    double time_s = 0.0, vel_linear = 0.0, vel_angular = 0.0;
    sensor_msgs_z::IMUData imu_result;
    Eigen::Vector3d angule_imu = {0.0, 0.0, 0.0};
    while (!mqWheelBuffer.empty() && !mqIMUBuffer.empty()) {
        {
            std::unique_lock<std::mutex> lock_wheel(mWheelDataMutex);
            wheel_before_time = mqWheelBuffer.front().first;
            time_s = mqWheelBuffer.front().first;
            vel_linear = mqWheelBuffer.front().second(0);
            vel_angular = mqWheelBuffer.front().second(1);
        }
        {
            std::unique_lock<std::mutex> lock_imu(mImuDataMutex);
            imu_back_time = mqIMUBuffer.back().first;
        }
        if (wheel_before_time > imu_back_time + 0.01) break;
        if (mLastWheelTime > 0.1) {
            bool bStatic = false;
            if ((fabs(vel_linear) < 1e-6) && (fabs(vel_angular) < 1e-6)) { // not support for big time lag
                bStatic = true;
            } else {
                bStatic = false;
            }
            {
                std::unique_lock<std::mutex> lock_imu(mImuDataMutex);
                while ((!mqIMUBuffer.empty()) && (mqIMUBuffer.front().first < time_s + 0.001)) {
                    Eigen::Matrix<double, 6, 1> data_imu = mqIMUBuffer.front().second;
                    angule_imu << data_imu(3), data_imu(4), data_imu(5);
                    double data_time = mqIMUBuffer.front().first;
                    lock_imu.unlock();
                    imu_result = processImu(data_time, data_imu, bStatic);
                    if (imu_result.efficient) {
                        std::unique_lock<std::shared_timed_mutex> lock_pose(get_pose_mutex_);
                        Odom_.time = time_s;
                        Odom_.topic = "dr2_result";
                        Odom_.acc = imu_result.acc;
                        Odom_.gyro = imu_result.gyro;
                        Odom_.rpy = imu_result.rpy;
                        Odom_.R_Mat = eulerAnglesToRotationMatrix(Odom_.rpy);
                        Odom_.rotation = euler3d_to_quaternion(Odom_.rpy);
                    }
                    lock_imu.lock();
                    mqIMUBuffer.pop_front();
                }
            }
            if (imu_result.efficient) {
                Eigen::Vector3d inspeed = {vel_linear, 0.0, 0.0};
                WheelVel2IMUVel(inspeed, angule_imu);
                Eigen::Vector3d dDist = inspeed * (time_s - mLastWheelTime);
                Eigen::Matrix3d MatRPY = E3dEulerToMatrix(imu_result.rpy);
                {
                    std::unique_lock<std::shared_timed_mutex> lock(get_pose_mutex_);
                    Odom_.position += MatRPY * dDist;
                    Odom_.twist = inspeed;
                }
                mLastWheelTime = time_s;
            }
            mPublishPoseFlag = true;
            if (time_s > 0)
                fTestWheelOdom << std::fixed << std::setprecision(3) << time_s << " "
                               << mXOdom << " " << mYOdom << " 0 " << imu_result.rpy(0) << " "
                               << imu_result.rpy(1) << " " << imu_result.rpy(2) << endl;
        } else {
            mLastWheelTime = time_s;
        }
        mqWheelBuffer.pop_front();
    }
}

void DrOdoFlow2::CalcStaticBias(const Eigen::Vector3d &gry) {
    mGyroBiasSum += gry;
    mGyroBiasNum++;
    //    imuStaticW.emplace_back(gry);
    ////    cerr <<"gyr "<<gry.transpose()<<endl;
    //
    //    while (imuStaticW.size() > 10000) {
    //        imuStaticW.pop_front();
    //    }
    //
    //    Eigen::Vector3d imuBiasSum(0.,0.,0.);
    //    for (auto const &imu: imuStaticW) {
    //        imuBiasSum += imu;
    //    }

    mImuStaticBiasMean = mGyroBiasSum / double(mGyroBiasNum);

    //    cerr <<"mImuStaticBiasMean "<<mImuStaticBiasMean<<endl;
    //    abort();
}

double DrOdoFlow2::radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

// Mahony互补滤波算法
Eigen::Vector3d DrOdoFlow2::updateIMU(double gx, double gy, double gz,
                                      double ax, double ay, double az,
                                      double dT) {
    double recipNorm = sqrt(ax * ax + ay * ay + az * az);
    ax /= recipNorm;
    ay /= recipNorm;
    az /= recipNorm;
    double halfvx = m_q1 * m_q3 - m_q0 * m_q2;
    double halfvy = m_q0 * m_q1 + m_q2 * m_q3;
    double halfvz = m_q0 * m_q0 - 0.5 + m_q3 * m_q3;
    //差积误差
    double halfex = (ay * halfvz - az * halfvy);
    double halfey = (az * halfvx - ax * halfvz);
    double halfez = (ax * halfvy - ay * halfvx);
    //差积误差积分为角速度
    mIntegralFBx += mKi * halfex * dT;
    mIntegralFBy += mKi * halfey * dT;
    mIntegralFBz += mKi * halfez * dT;
    gx += mIntegralFBx;
    gy += mIntegralFBy;
    gz += mIntegralFBz;
    //角度补偿
    gx += mKp * halfex;
    gy += mKp * halfey;
    gz += mKp * halfez;

    gx *= (0.5 * dT);
    gy *= (0.5 * dT);
    gz *= (0.5 * dT);
    const double qa = m_q0;
    const double qb = m_q1;
    const double qc = m_q2;
    //更新四元数
    m_q0 += (-qb * gx - qc * gy - m_q3 * gz);
    m_q1 += (qa * gx + qc * gz - m_q3 * gy);
    m_q2 += (qa * gy - qb * gz + m_q3 * gx);
    m_q3 += (qa * gz + qb * gy - qc * gx);
    //单位化四元数
    recipNorm = sqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
    m_q0 /= recipNorm;
    m_q1 /= recipNorm;
    m_q2 /= recipNorm;
    m_q3 /= recipNorm;
    //四元数反解欧拉角
    double roll =
        atan2(m_q0 * m_q1 + m_q2 * m_q3, 0.5 - m_q1 * m_q1 - m_q2 * m_q2);
    double pitch = asin(-2.0 * (m_q1 * m_q3 - m_q0 * m_q2));
    double yaw =
        atan2(m_q1 * m_q2 + m_q0 * m_q3, 0.5 - m_q2 * m_q2 - m_q3 * m_q3);
    return {roll, pitch, yaw};
}

} // namespace DROdom2
