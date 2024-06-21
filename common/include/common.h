#pragma once
#include <stdlib.h>
#include <chrono>
#include <fstream>
#include <memory>
#include <iostream>
#include <deque>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/SVD"
#include "eigen3/Eigen/Dense"
#include <limits>
#include "log.hpp"
constexpr double kDeg2Rad = M_PI / 180.; //度转弧度
constexpr double kRad2Deg = 180. / M_PI; //弧度转度
constexpr double kGravity = 9.8;
const double POSITIVE_INFINITY = std::numeric_limits<double>::infinity();
enum class Status { kValid,
                    kInvalid };

/**
 * @brief 限制输入为-M_PI-M_PI
 * 
 * @param angle 
 * @return double 
 */
static double restrict_angle_range(double angle) {
    while (fabs(angle) > M_PI) {
        if (angle > 0)
            angle -= 2 * M_PI;
        else
            angle += 2 * M_PI;
    }
    return angle;
}
/**
 * @brief 欧拉角转四元数
 * 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @return Eigen::Quaterniond 
 */
static Eigen::Quaterniond euler_to_quaternion(double roll, double pitch, double yaw) {
    // 创建欧拉角
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    // 将旋转矩阵转换为四元数
    Eigen::Quaterniond quaternion(rotation_matrix);

    return quaternion;
}
/**
 * @brief 欧拉角转四元数
 * 
 * @param angxyz 
 * @return Eigen::Quaterniond 
 */
static Eigen::Quaterniond euler3d_to_quaternion(Eigen::Vector3d &angxyz) {
    // 创建欧拉角
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(angxyz(0), Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(angxyz(1), Eigen::Vector3d::UnitY())
                      * Eigen::AngleAxisd(angxyz(2), Eigen::Vector3d::UnitZ());

    // 将旋转矩阵转换为四元数
    Eigen::Quaterniond quaternion(rotation_matrix);

    return quaternion;
}

/**
 * @brief 四元数转欧拉角
 * 
 * @param q 
 * @return Eigen::Vector3d 
 */
static Eigen::Vector3d VecToEulerAngles(Eigen::Quaterniond q) {
    Eigen::Vector3d euler_angles;

    // 滚转角 (Roll)
    euler_angles(0) = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

    // 俯仰角 (Pitch)
    euler_angles(1) = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));

    // 偏航角 (Yaw)
    euler_angles(2) = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    // 将弧度转换为度
    // euler_angles = euler_angles * 180.0 / M_PI;

    return euler_angles;
}
/**
 * @brief 四元数转欧拉角
 * 
 * @tparam T 
 * @param q 
 * @param roll 
 * @param pitch 
 * @param yaw 
 */
template <typename T>
void toEulerAngle(const Eigen::Quaternion<T> &q, T &roll, T &pitch, T &yaw) {
    // roll (x-axis rotation)
    T sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    T cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    T sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    T siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    T cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}
/**
 * @brief 斜对称矩阵（或斜矩阵）用于将叉积表示为矩阵乘法
 * 
 * @param v 
 * @return Eigen::Matrix3d 
 */
inline Eigen::Matrix3d SkewMat(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1), v(2), 0., -v(0), -v(1), v(0), 0.;
    return w;
}
/**
 * @brief 斜对称矩阵（或斜矩阵）用于将叉积表示为矩阵乘法
 * 
 * @tparam T 
 * @param axis 
 * @return Eigen::Matrix<T, 3, 3> 
 */
template <typename T>
inline Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> &axis) {
    Eigen::Matrix<T, 3, 3> skew_matrix = Eigen::Matrix<T, 3, 3>::Identity();

    skew_matrix << 0, -axis(2, 0), axis(1, 0), axis(2, 0), 0, -axis(0, 0), -axis(1, 0), axis(0, 0), 0;

    return skew_matrix;
}
/**
 * @brief 4*4位姿矩阵求逆,3*3旋转,最后一列为位置
 * 
 * @tparam T 
 * @param Tcw 
 * @return Eigen::Matrix<T, 4, 4> 
 */
template <typename T>
inline Eigen::Matrix<T, 4, 4> EigenIsoInv(const Eigen::Matrix<T, 4, 4> &Tcw) {
    Eigen::Matrix<T, 3, 3> Rcw = Tcw.block(0, 0, 3, 3);
    Eigen::Matrix<T, 3, 1> tcw = Tcw.block(0, 3, 3, 1);
    Eigen::Matrix<T, 3, 3> Rwc = Rcw.transpose();
    Eigen::Matrix<T, 3, 1> twc = -Rwc * tcw;

    Eigen::Matrix<T, 4, 4> Twc = Eigen::Matrix<T, 4, 4>::Identity();

    Twc.block(0, 0, 3, 3) = Rwc;
    Twc.block(0, 3, 3, 1) = twc;

    return Twc;
}

/**
 * @brief 欧拉角转旋转矩阵
 * 
 * @param roll 
 * @param pitch 
 * @param yaw 
 * @return Eigen::Matrix3d 
 */
static Eigen::Matrix3d EulerAnglesToRotationMatrix(double roll, double pitch, double yaw) {
    // 定义单轴旋转矩阵
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    // 组合旋转矩阵
    Eigen::Matrix3d R = R_z * R_y * R_x;

    return R;
}
/**
 * @brief 旋转矩阵转四元数
 * 
 * @param input 
 * @return Eigen::Quaterniond 
 */
static Eigen::Quaterniond Mat3dToQuat(Eigen::Matrix3d &input) {
    return Eigen::Quaterniond(input);
}
/**
 * @brief 欧拉角转旋转矩阵
 * 
 * @param angxyz 
 * @return Eigen::Matrix3d 
 */
static Eigen::Matrix3d E3dEulerToMatrix(Eigen::Vector3d angxyz) {
    // 定义单轴旋转矩阵
    double roll = angxyz(0);
    double pitch = angxyz(1);
    double yaw = angxyz(2);
    Eigen::Matrix3d R_x;
    R_x << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);

    Eigen::Matrix3d R_y;
    R_y << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);

    Eigen::Matrix3d R_z;
    R_z << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;

    // 组合旋转矩阵
    Eigen::Matrix3d R = R_z * R_y * R_x;

    return R;
}
inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d &v) {
    Eigen::Matrix3d w;
    w << 0., -v(2), v(1),
        v(2), 0., -v(0),
        -v(1), v(0), 0.;

    return w;
}
/**
 * @brief 四元数转旋转矩阵
 * 
 */
static Eigen::Matrix3d QuatToRotMax(Eigen::Quaterniond q) {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = q.toRotationMatrix();
    return rotation_matrix;
}

static Eigen::Matrix3d expSO3(const Eigen::Vector3d &omega) {
    double theta = omega.norm();
    Eigen::Matrix3d Omega = GetSkewMatrix(omega);

    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    if (theta < 1e-10) {
        // When theta is very small, use first-order Taylor expansion
        return I + Omega;
    } else {
        Eigen::Matrix3d Omega2 = Omega * Omega;
        return I + (std::sin(theta) / theta) * Omega + ((1 - std::cos(theta)) / (theta * theta)) * Omega2;
    }
}
