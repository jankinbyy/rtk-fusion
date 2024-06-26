#pragma once
#include "sensor_type.h"
using namespace sensor_msgs_z;
class EKF_S {
public:
    EKF_S() {
        imu_T_wheel << 0, -1, 0, 0.0014,
            1, 0, 0, -0.8164,
            0, 0, 1, -0.491,
            0, 0, 0, 1;
        wheel_T_imu = imu_T_wheel.inverse();
        std::cout << "wheel_t_imu:" << wheel_T_imu << std::endl;
    }
    Eigen::VectorXd state = Eigen::VectorXd::Zero(6); // [position (3), rpy (3)],GNSS_T_P GNSS_T_W
    Eigen::VectorXd state3 = Eigen::VectorXd::Zero(3);
    Eigen::Matrix4d imu_T_wheel;
    Eigen::Matrix4d wheel_T_imu;
    Eigen::Matrix4d ENU_T_IMU = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(6, 6); // Covariance matrix
    Eigen::MatrixXd P3 = Eigen::MatrixXd::Identity(3, 3);

    void Predict(const OdometryData &curr_imu_data) {
        if (last_imu_data.time > 0) {
            // Time delta
            double dt = curr_imu_data.time - last_imu_data.time;
            // Prediction model
            Eigen::Vector3d delta_angule = curr_imu_data.rpy - last_imu_data.rpy;
            Eigen::Vector3d delta_pos = Eigen::Vector3d((curr_imu_data.position - last_imu_data.position).norm(), 0.0, 0.0);
            Eigen::Vector3d cur_enu_rpy = Eigen::Vector3d::Zero();
            cur_enu_rpy.z() = state3(2) + delta_angule.z();
            cur_enu_rpy.z() = restrict_angle_range(cur_enu_rpy.z());
            Eigen::MatrixXd F3 = Eigen::MatrixXd::Identity(3, 3);
            F3 << 1, 0, -sin(cur_enu_rpy.z()) * delta_pos.norm(),
                0, 1, cos(cur_enu_rpy.z()) * delta_pos.norm(),
                0, 0, 1;
            Eigen::MatrixXd G3 = Eigen::MatrixXd(3, 2);
            G3 << cos(cur_enu_rpy.z()), -sin(cur_enu_rpy.z()) * delta_pos.norm(),
                sin(cur_enu_rpy.z()), cos(cur_enu_rpy.z()) * delta_pos.norm(),
                0, 1;
            Eigen::MatrixXd Q3 = Eigen::MatrixXd::Identity(2, 2);
            Q3 << 0.05 * 0.05, 0.0,
                0.0, 1 * kDeg2Rad * 1 * kDeg2Rad;
            P3 = F3 * P3 * F3.transpose() + G3 * Q3 * G3.transpose();
            state3(0) = state3(0) + cos(cur_enu_rpy.z()) * delta_pos.norm();
            state3(1) = state3(1) + sin(cur_enu_rpy.z()) * delta_pos.norm();
            state3(2) = cur_enu_rpy.z();
        }

        last_imu_data = curr_imu_data;
    }

    void Update(const GnssData &curr_gps_data) {
        // Measurement matrix
        Eigen::MatrixXd H3 = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd R3 = Eigen::MatrixXd::Identity(3, 3);
        R3 << 0.03 * 0.03, 0.0, 0.0,
            0.0, 0.03 * 0.03, 0.0,
            0.0, 0.0, 2.5 * kDeg2Rad * 2.5 * kDeg2Rad;
        Eigen::VectorXd z3 = Eigen::VectorXd::Zero(3);
        z3(0) = curr_gps_data.xyz.x();
        z3(1) = curr_gps_data.xyz.y();
        if (abs(curr_gps_data.rpy[2]) < M_PI) {
            z3(2) = curr_gps_data.rpy.z();
        } else {
            z3(2) = state3(2); // Keep the previous orientation
        }
        Eigen::VectorXd y3 = z3 - H3 * state3;
        y3(2) = restrict_angle_range(y3(2));
        //std::cout << "y3:" << y3.norm() << std::endl;
        Eigen::MatrixXd S3 = H3 * P3 * H3.transpose() + R3;
        Eigen::MatrixXd K3 = P3 * H3.transpose() * S3.inverse();
        state3 += K3 * y3;
        if ((K3 * y3).norm() > 0.5)
            std::cout << "update big" << std::endl;
        P3 = (Eigen::MatrixXd::Identity(3, 3) - K3 * H3) * P3;
    }

    void GetPose(OdometryData &out) {
        out.position << state3(0), state3(1), 0.0;
        out.rpy.z() = state3(2);
    }

private:
    OdometryData last_imu_data;
};