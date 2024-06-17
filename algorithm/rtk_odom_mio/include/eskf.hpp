#pragma once
#include "sensor_type.h"
using namespace sensor_msgs_z;
class ESKF {
public:
    ESKF() {
        x = Eigen::VectorXd(3); // [x, y, yaw]
        x << 0, 0, 0;
        P = Eigen::MatrixXd::Identity(3, 3);
        Q = 0.1 * 0.1 * Eigen::MatrixXd::Identity(3, 3);
        R1 << 0.02 * 0.02, 0, 0,
            0, 0.02 * 0.02, 0,
            0, 0, (5.0 * M_PI / 180.0) * (5.0 * M_PI / 180.0);
    };
    State state;
    Eigen::VectorXd x;  // 状态向量 [x, y, yaw]
    Eigen::MatrixXd P;  // 状态协方差矩阵
    Eigen::MatrixXd Q;  // 过程噪声协方差矩阵
    Eigen::Matrix3d R1; // 定义观测噪声协方差矩阵
    const double acc_noise_ = 0.02;
    const double gyro_noise_ = 0.02;
    const double acc_bias_noise_ = 0.02;
    const double gyro_bias_noise_ = 0.02;
    const Eigen::Vector3d I_p_Gps_ = {0.1, 0.1, 0.0};
    const Eigen::Vector3d gravity_ = {0.0, 0.0, -9.7};
    OdometryData last_odom_;
    void predict(const Eigen::Vector3d &delta_x) {
        // 状态转移矩阵 F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3, 3);

        // 状态预测
        x = F * x + delta_x; //变化值
        // 状态协方差预测
        P = F * P * F.transpose() + Q; //Q为观测噪声
    }

    void update(const Eigen::Vector3d &z) {
        // 观测矩阵 H
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3, 3);
        // 创新协方差
        Eigen::MatrixXd S = H * P * H.transpose() + R1;
        // 卡尔曼增益
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        // 状态更新
        x = x + K * (z - H * x);
        // 状态协方差更新
        P = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P;
    }
    void Predict(const OdometryData cur_input) {
        // Time.
        if (last_odom_.time > 0) {
            double delta_t = cur_input.time - last_odom_.time;
            const double delta_t2 = delta_t * delta_t;

            // Set last state.
            State last_state = state;

            // Acc and gyro.
            const Eigen::Vector3d acc_unbias = 0.5 * (last_odom_.acc + cur_input.acc) - last_state.acc_bias;
            const Eigen::Vector3d gyro_unbias = 0.5 * (last_odom_.gyro + cur_input.gyro) - last_state.gyro_bias;

            // Normal state.
            // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
            if (1)
                state.G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
            else
                state.G_p_I = last_state.G_p_I + (cur_input.position - last_odom_.position);
            if (0)
                state.G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;
            else
                state.G_v_I = ((last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t) + cur_input.twist) / 2.0; //速度为轮速与imu积分和
            Eigen::Vector3d delta_angle_axis;
            if (0)
                delta_angle_axis = gyro_unbias * delta_t;
            else
                delta_angle_axis = cur_input.rpy - last_odom_.rpy;
            if (delta_angle_axis.norm() > 1e-12) {
                state.G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
            }
            Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
            Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * delta_t;
            Fx.block<3, 3>(3, 6) = -state.G_R_I * GetSkewMatrix(acc_unbias) * delta_t;
            Fx.block<3, 3>(3, 9) = -state.G_R_I * delta_t;
            if (delta_angle_axis.norm() > 1e-12) {
                Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
            } else {
                Fx.block<3, 3>(6, 6).setIdentity();
            }
            Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * delta_t;

            Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
            Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

            Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
            Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
            Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
            Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
            Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

            state.cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

            // Time and imu.
            state.timestamp = cur_input.time;
        }
        last_odom_ = cur_input;
    }
    void UpdateStateByGpsPosition(const GnssData gps_data_ptr) {
        Eigen::Matrix<double, 3, 15> H;
        Eigen::Vector3d residual;
        double residual_yaw;
        Eigen::Vector3d G_p_Gps = gps_data_ptr.xyz;
        // Compute residual.
        residual = G_p_Gps - (state.G_p_I + state.G_R_I * I_p_Gps_);
        if (abs(gps_data_ptr.rpy(2)) < 2 * M_PI)
            residual_yaw = gps_data_ptr.rpy(2) - state.G_R_I.eulerAngles(0, 1, 2).z();
        // Compute jacobian.
        Eigen::Matrix3d input_angul;
        if (abs(gps_data_ptr.rpy(2)) < 2 * M_PI && 0) {
            input_angul = -E3dEulerToMatrix(gps_data_ptr.rpy);
        } else {
            input_angul = -state.G_R_I * GetSkewMatrix(I_p_Gps_);
        }
        H.setZero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 6) = input_angul;
        Eigen::Matrix3d V = gps_data_ptr.cov; //观测噪声
        V << 0.002, 0.0, 0.0,
            0.0, 0.002, 0.0,
            0.0, 0.0, 0.002;
        // EKF.
        const Eigen::MatrixXd &P = state.cov;
        const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
        const Eigen::VectorXd delta_x = K * residual;
        // Add delta_x to state.
        state.G_p_I += delta_x.block<3, 1>(0, 0);
        state.G_v_I += delta_x.block<3, 1>(3, 0);
        state.acc_bias += delta_x.block<3, 1>(9, 0);
        state.gyro_bias += delta_x.block<3, 1>(12, 0);
        if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
            state.G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
        }
        // Covarance.
        const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
        state.cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
        last_odom_.position = state.G_p_I;
    }
};