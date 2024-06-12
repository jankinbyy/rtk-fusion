#pragma once

#include <iostream>
#include <memory>
#include "sensor_type.h"
using namespace sensor_msgs_z;

class Initializer {
public:
    struct Config {
        size_t acc_buffer_size = 10;
        double max_acc_std = 0.5;
    };

    Initializer() = default;

    bool Initialize(const Eigen::Vector3d &acc, Eigen::Matrix3d *G_R_I);

private:
    const Config config_;

    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_buffer_;
};
class Propagator {
public:
    Propagator(const double &gyro_noise, const double &gyro_bias_noise);

    void PropagateMeanAndCov(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &begin_bg,
                             const Eigen::Matrix<double, 6, 6> &begin_cov, const Eigen::Vector3d &gyro,
                             const double delta_t, Eigen::Matrix3d *end_G_R_I, Eigen::Vector3d *end_bg,
                             Eigen::Matrix<double, 6, 6> *end_cov);

    void PropagateMean(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro, const double delta_t,
                       Eigen::Matrix3d &end_G_R_I);

    void PropagateRK(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro,
                     const Eigen::Vector3d &last_gyro, const double delta_t, Eigen::Matrix3d &end_G_R_I);

    void EstimateByAcc(const Eigen::Vector3d &acc, Eigen::Matrix3d &end_G_R_I);

private:
    double gyro_noise_;
    double gyro_bias_noise_;
};
class Estimator {
public:
    struct Config {
        size_t acc_buffer_size = 1;
    };

    Estimator(const double gyro_noise, const double gyro_bias_noise, const double acc_noise);

    Status Estimate(double timestamp, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc,
                    Eigen::Matrix3d *G_R_I);

    void clear_state_ori();

    // State.
    Eigen::Matrix3d G_R_I_;
    Eigen::Vector3d bg_;
    Eigen::Matrix<double, 6, 6> cov_;

    Status status_;
    double last_timestamp_;
    Eigen::Matrix3d acc_noise_mat_;

    std::unique_ptr<Initializer> initializer_;
    std::unique_ptr<Propagator> propagator_;

    const Config config_;

    std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> acc_buffer_;
};

IMUData preIntegrate(const IMUData &imu_in, const std::shared_ptr<Estimator> &orientation_estimator);
void Update(const Eigen::Matrix3d &prior_G_R_I, const Eigen::Vector3d &prior_bg,
            const Eigen::Matrix<double, 6, 6> &prior_cov, const Eigen::Vector3d &acc, const Eigen::Matrix3d &acc_noise,
            Eigen::Matrix3d *posterior_G_R_I, Eigen::Vector3d *posterior_bg,
            Eigen::Matrix<double, 6, 6> *posterior_cov);