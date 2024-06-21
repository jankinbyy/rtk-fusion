#pragma once
#include "sensor_type.h"
using namespace sensor_msgs_z;
class ESKF {
private:
    static const unsigned int DIM_STATE = 15;
    static const unsigned int DIM_STATE_NOISE = 6;       // 噪声只有6维，陀螺仪和加速度计的bias
    static const unsigned int DIM_MEASUREMENT = 3;       // 观测向量只有6维
    static const unsigned int DIM_MEASUREMENT_NOISE = 3; // 观测噪声

    static const unsigned int INDEX_STATE_POSI = 0;      // 位置
    static const unsigned int INDEX_STATE_VEL = 3;       // 速度
    static const unsigned int INDEX_STATE_ORI = 6;       // 角度
    static const unsigned int INDEX_STATE_GYRO_BIAS = 9; // 陀螺仪bias
    static const unsigned int INDEX_STATE_ACC_BIAS = 12; // 加速度计bias
    static const unsigned int INDEX_MEASUREMENT_POSI = 0;

    typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;               // 状态向量
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;         // 观测向量 GPS的位置
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;       //15*15
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB; //15*6
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, 1> TypeMatrixW;         //6*1 陀螺仪和加速度计噪声
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixC;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixH;
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

    TypeVectorX X_; //估计的状态变化量,P_enu,V_enu,,R_enu_t_imu,R_enu_bg,R_enu_ba
    TypeVectorY Y_; //rtk观测误差
    TypeMatrixF F_; //雅可比矩阵
    TypeMatrixB B_; //噪声转移矩阵
    TypeMatrixW W_; //过程噪声
    TypeMatrixQ Q_; //过程噪声协方差矩阵
    TypeMatrixP P_; //状态估计的协方差矩阵，表示对当前状态估计的不确定性。随着EKF迭代过程的进行，这个矩阵被更新以反映估计的精确度
    TypeMatrixK K_; //卡尔曼增益
    TypeMatrixC C_; //测量噪声协方差矩阵
    TypeMatrixH H_; //观测协方差矩阵
    TypeMatrixC R_; //测量噪声 rtk

    TypeMatrixF Ft_;

    Eigen::Vector3d init_velocity_ = Eigen::Vector3d::Zero();
    State state_;
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

    Eigen::Vector3d g_; //重力加速度
    Eigen::Vector3d w_; //地球自传角速度

    GnssData curr_gps_data_;

    double L_ = 0.0; //纬度

    std::deque<OdometryData> imu_data_buff_; // 只保存两个IMU数据
    bool init_ = false;
    Eigen::Matrix4d imu_t_wheel_;

public:
    ESKF(Eigen::Matrix4d _imu_t_wheel) :
        imu_t_wheel_(_imu_t_wheel) {
        double gravity = 9.7;
        double earth_rotation_speed = 7.2921151467e-05;
        L_ = 32.0;
        double cov_prior_posi = 0.001;
        double cov_prior_vel = 0.001;
        double cov_prior_ori = 0.001;
        double cov_prior_epsilon = 0.001;
        double cov_prior_delta = 0.01;
        double cov_measurement_posi = 0.0001;
        double cov_process_gyro = 0.001;
        double cov_process_accel = 0.01;
        double cov_w_gyro = 0.001; // IMU数据的噪声，用来组成W矩阵
        double cov_w_accel = 0.01;
        g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
        w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(L_ * kDeg2Rad),
                             earth_rotation_speed * sin(L_ * kDeg2Rad)); // w_ie_n

        SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
                       cov_prior_epsilon, cov_prior_delta);
        SetCovarianceR(cov_measurement_posi);
        SetCovarianceQ(cov_process_gyro, cov_process_accel);
        SetCovarianceW(cov_w_gyro, cov_w_accel);

        X_.setZero();     // 初始化为零矩阵
        F_.setZero();     // 初始化为零矩阵
        C_.setIdentity(); // 单位矩阵
        H_.block<3, 3>(INDEX_MEASUREMENT_POSI, INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();

        F_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity(); // ?矩阵
        F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = GetSkewMatrix(-w_);
    };

    /*!
     * 用于ESKF滤波器的初始化，设置初始位姿，初始速度
     * @param curr_gps_data 与imu时间同步的gps数据
     * @param curr_imu_data 与gps时间同步的imu数据
     * @return
     */
    bool Init(const GnssData &curr_gps_data, const OdometryData &curr_imu_data) {
        if (!init_) {
            init_velocity_ = curr_imu_data.twist; // 用真实速度初始化
            state_.G_v_I = init_velocity_;
            // imu为右前上 rtk为东北天,绕z轴旋转
            double angle = curr_gps_data.rpy.z(); // 60度 = π/3 弧度

            // 使用 AngleAxis 定义绕z轴的旋转
            Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitZ());

            // 计算旋转矩阵
            Eigen::Matrix3d R_rotated = rotation.toRotationMatrix() * Eigen::Matrix3d::Identity();
            //Eigen::Quaterniond Q = Eigen::AngleAxisd(curr_gps_data.rpy.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0 * kDeg2Rad, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0 * kDeg2Rad, Eigen::Vector3d::UnitX());
            //前右地 rtk北东地
            //Eigen::Quaterniond Q = Eigen::AngleAxisd(90 * kDeg2Rad, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0 * kDeg2Rad, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(180 * kDeg2Rad, Eigen::Vector3d::UnitX());
            //init_pose_.block<3, 3>(0, 0) = E3dEulerToMatrix(curr_gps_data.rpy);
            init_pose_.block<3, 1>(0, 3) = curr_gps_data.xyz;
            init_pose_.block<3, 3>(0, 0) = R_rotated;
            std::cout << "orin:" << init_pose_.block<3, 3>(0, 0) << std::endl;
            state_.G_p_I = init_pose_.block<3, 1>(0, 3);
            state_.G_R_I = init_pose_.block<3, 3>(0, 0);
            X_.block<3, 1>(INDEX_STATE_ORI, 0) = (state_.G_R_I * (E3dEulerToMatrix(curr_imu_data.rpy).inverse())).eulerAngles(0, 1, 2);
            imu_data_buff_.clear(); // 这时ESKF中的imu数据
            imu_data_buff_.push_back(curr_imu_data);

            curr_gps_data_ = curr_gps_data;
            init_ = true;
        }
        return init_;
    };
    bool GetInitState() {
        if (init_)
            return true;
        else
            return false;
    }

    bool UpdateErrorState(double t, const Eigen::Vector3d &accel) {
        Eigen::Matrix3d F_23 = GetSkewMatrix(accel);
        // 没有更新F33,因为它是常数，由地球自转和纬度决定
        F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
        F_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = state_.G_R_I;
        F_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -state_.G_R_I;
        B_.setZero();
        B_.block<3, 3>(INDEX_STATE_VEL, 3) = state_.G_R_I;
        B_.block<3, 3>(INDEX_STATE_ORI, 0) = -state_.G_R_I;

        TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * t;
        TypeMatrixB Bk = B_ * t;

        Ft_ = F_ * t;

        X_ = Fk * X_ + Bk * W_;
        P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();

        return true;
    }
    /*!
     * 滤波器的预测，对应卡尔曼滤波器的前两个公式
     * @param curr_imu_data
     * @return
     */
    bool Predict(const OdometryData &curr_imu_data) {
        imu_data_buff_.push_back(curr_imu_data);
        //std::cout << "imu.acc:" << curr_imu_data.acc.transpose() << "gyro:" << curr_imu_data.gyro.transpose() << std::endl;
        if (1) {
            UpdateOdomEstimation(); // 更新 角度  速度 位置  PVQ

            double delta_t = curr_imu_data.time - imu_data_buff_.front().time; // dt

            Eigen::Vector3d curr_accel = state_.G_R_I * curr_imu_data.acc; // 导航坐标系下的加速度

            UpdateErrorState(delta_t, curr_accel); // 更新误差状态，只与R和accel相关
        }
        imu_data_buff_.pop_front();
        return true;
    };

    /*!
     * 滤波器的矫正，对应卡尔曼滤波器的后三个公式
     * @param curr_gps_data
     * @return
     */
    bool Correct(const GnssData &curr_gps_data) {
        curr_gps_data_ = curr_gps_data;

        Y_ = state_.G_p_I - curr_gps_data.xyz; //! Y_cal-Y_measure

        K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + C_ * R_ * C_.transpose()).inverse(); // kalman增益

        P_ = (TypeMatrixP::Identity() - K_ * H_) * P_;
        X_ = X_ + K_ * (Y_ - H_ * X_);

        EliminateError();

        ResetState();

        return true;
    }

    State GetPose() const {
        State out_pos;
        out_pos.G_p_I = state_.G_p_I;
        out_pos.G_R_I = imu_t_wheel_.block<3, 3>(0, 0).inverse() * state_.G_R_I;
        return out_pos;
    }

    Eigen::Vector3d GetVelocity() {
        return state_.G_v_I;
    }

private:
    void SetCovarianceW(double gyro_noise, double accel_noise) {
        W_.setZero();
        W_.block<3, 1>(0, 0) = Eigen::Vector3d(accel_noise, accel_noise, accel_noise);
        W_.block<3, 1>(3, 0) = Eigen::Vector3d(gyro_noise, gyro_noise, gyro_noise);
    }

    void SetCovarianceQ(double gyro_noise, double accel_noise) {
        Q_.setZero();
        Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise; // 平方
        Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
    }

    void SetCovarianceR(double posi_noise) {
        R_.setZero();
        R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
    }

    // 设置P矩阵
    void SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                        double gyro_noise, double accel_noise) {
        P_.setZero();
        P_.block<3, 3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
        P_.block<3, 3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
        P_.block<3, 3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise;
        P_.block<3, 3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
        P_.block<3, 3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
    }

    /*!
     * 通过IMU计算位姿和速度
     * @return
     */
    bool UpdateOdomEstimation() {
        Eigen::Vector3d angular_delta;
        ComputeAngularDelta(angular_delta); // 平均角速度求转动过的角度，以此求delta_R

        Eigen::Matrix3d R_nm_nm_1 = Eigen::Matrix3d::Identity(); // i系到n系
        if (0)                                                   //不考虑地球自传
            ComputeEarthTranform(R_nm_nm_1);                     // 考虑地球自传

        Eigen::Matrix3d curr_R, last_R;
        ComputeOrientation(angular_delta, R_nm_nm_1, curr_R, last_R);

        Eigen::Vector3d curr_vel, last_vel;
        ComputeVelocity(curr_vel, last_vel, curr_R, last_R);

        ComputePosition(curr_vel, last_vel);

        return true;
    }

    bool ComputeAngularDelta(Eigen::Vector3d &angular_delta) {
        OdometryData curr_imu_data = imu_data_buff_.at(1);
        OdometryData last_imu_data = imu_data_buff_.at(0);

        double delta_t = curr_imu_data.time - last_imu_data.time;

        if (delta_t <= 0) {
            return false;
        }
        if (1) {
            Eigen::Vector3d curr_angular_vel = curr_imu_data.gyro;
            Eigen::Vector3d last_angular_vel = last_imu_data.gyro;
            // 直接使用last_R来对gyro_bias进行旋转
            Eigen::Matrix3d last_R = state_.G_R_I;
            Eigen::Vector3d curr_unbias_angular_vel = curr_angular_vel;
            Eigen::Vector3d last_unbias_angular_vel = last_angular_vel;
            angular_delta = 0.5 * (curr_unbias_angular_vel + last_unbias_angular_vel) * delta_t; // 中值
        } else {
            angular_delta = curr_imu_data.rpy - last_imu_data.rpy;
        }

        return true;
    }

    /*!
     * 计算地球转动给导航系带来的变换
     * @param R_nm_nm_1
     * @return
     */
    bool ComputeEarthTranform(Eigen::Matrix3d &R_nm_nm_1) {
        OdometryData curr_imu_data = imu_data_buff_.at(1);
        OdometryData last_imu_data = imu_data_buff_.at(0);

        double delta_t = curr_imu_data.time - last_imu_data.time;

        constexpr double rm = 6353346.18315;
        constexpr double rn = 6384140.52699;
        Eigen::Vector3d w_en_n(-state_.G_v_I[1] / (rm + curr_gps_data_.LonLatAlt[2]),
                               state_.G_v_I[0] / (rn + curr_gps_data_.LonLatAlt[2]),
                               state_.G_v_I[0] / (rn + curr_gps_data_.LonLatAlt[2])
                                   * std::tan(curr_gps_data_.LonLatAlt[0] * kDeg2Rad));
        // 实际导航坐标系中，不动系(i系)是地心惯性系，我们需要的导航结果是相对于导航系(n系)的
        // 两个坐标系中有一个相对旋转，旋转角速度为w_in_n 。
        Eigen::Vector3d w_in_n = w_en_n + w_; // 导航系(n系)相对于惯性系(i系)的旋转，包含导航系相对于地球的旋转和地球自转
        auto angular = delta_t * w_in_n;
        Eigen::AngleAxisd angle_axisd(angular.norm(), angular.normalized());
        R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose(); // 取转置，得到i系相对于n系的转换
        return true;
    }

    bool ComputeOrientation(const Eigen::Vector3d &angular_delta,
                            const Eigen::Matrix3d R_nm_nm_1,
                            Eigen::Matrix3d &curr_R,
                            Eigen::Matrix3d &last_R) {
        Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized()); // 轴角公式，前一个为转动角度，后一个为向量。角度转旋转矩阵
        last_R = state_.G_R_I;

        curr_R = R_nm_nm_1 * state_.G_R_I * angle_axisd.toRotationMatrix(); // R*delta_R

        state_.G_R_I = curr_R;

        return true;
    }

    // 使用去除重力影响和加速度bias的平均加速度计算速度
    bool ComputeVelocity(Eigen::Vector3d &curr_vel, Eigen::Vector3d &last_vel,
                         const Eigen::Matrix3d &curr_R,
                         const Eigen::Matrix3d last_R) {
        OdometryData curr_imu_data = imu_data_buff_.at(1);
        OdometryData last_imu_data = imu_data_buff_.at(0);
        double delta_t = curr_imu_data.time - last_imu_data.time;
        if (delta_t <= 0) {
            return false;
        }

        Eigen::Vector3d curr_accel = curr_imu_data.acc;
        Eigen::Vector3d curr_unbias_accel = GetUnbiasAccel(curr_R * curr_accel);
        //std::cout << "curr_acc:" << curr_accel.transpose() << " curr_unbias_acc:" << curr_unbias_accel.transpose() << "  R:" << curr_R.eulerAngles(0, 1, 2).transpose() << std::endl;
        Eigen::Vector3d last_accel = last_imu_data.acc;
        Eigen::Vector3d last_unbias_accel = GetUnbiasAccel(last_R * last_accel); // 减去重力影响
        //std::cout << "last_acc:" << last_accel.transpose() << " last_unbias_acc:" << last_accel.transpose() << "  last_R:" << last_R.eulerAngles(0, 1, 2).transpose() << std::endl;
        last_vel = state_.G_v_I;
        state_.G_v_I += delta_t * 0.5 * (curr_unbias_accel + last_unbias_accel);
        if (1) //重新写速度观测
            state_.G_v_I = curr_R * curr_imu_data.twist;
        curr_vel = state_.G_v_I;
        //std::cout << "imu vel:" << curr_vel.transpose() << "imu input vel:" << (curr_R * curr_imu_data.twist).transpose() << std::endl;
        return true;
    }

    Eigen::Vector3d GetUnbiasAccel(const Eigen::Vector3d &accel) {
        return accel - state_.acc_bias + g_; // z方向精度提高很多
                                             // return accel + g_;
    }

    bool ComputePosition(const Eigen::Vector3d &curr_vel, const Eigen::Vector3d &last_vel) {
        double delta_t = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;
        state_.G_p_I += 0.5 * delta_t * (curr_vel + last_vel);

        return true;
    }
    // 只将状态量置零
    void ResetState() {
        X_.setZero();
    }
    // 估计值=估计值-误差量
    void EliminateError() {
        state_.G_p_I = state_.G_p_I - X_.block<3, 1>(INDEX_STATE_POSI, 0);
        state_.G_v_I = state_.G_v_I - X_.block<3, 1>(INDEX_STATE_VEL, 0);
        Eigen::Matrix3d C_nn = expSO3(X_.block<3, 1>(INDEX_STATE_ORI, 0)).matrix();
        state_.G_R_I = C_nn * state_.G_R_I; // 固定坐标系更新，左乘
        if (0) {
            state_.gyro_bias = state_.gyro_bias - X_.block<3, 1>(INDEX_STATE_GYRO_BIAS, 0);
            state_.acc_bias = state_.acc_bias - X_.block<3, 1>(INDEX_STATE_ACC_BIAS, 0);
        } else {
            state_.acc_bias = Eigen::Vector3d::Zero();  //估计存在错误,设置为0
            state_.gyro_bias = Eigen::Vector3d::Zero(); //估计存在错误,设置为0
        }
    }
};