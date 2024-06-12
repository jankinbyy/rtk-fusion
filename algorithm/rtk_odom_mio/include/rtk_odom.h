#ifndef RTK_ODOM_H
#define RTK_ODOM_H
#include <stdlib.h>
#include <iostream>
#include <string>
#include <shared_mutex>
#include <list>
#include <deque>
#include <functional>
#include <fstream>
#include <mutex>
#include <thread>
#include "sensor_type.h"
#include "trans_rtk_enu.h"
#include <utility>
using namespace sensor_msgs_z;
namespace rtk_odom_component {
class EKF {
public:
    Eigen::VectorXd x;  // 状态向量 [x, y, yaw]
    Eigen::MatrixXd P;  // 状态协方差矩阵
    Eigen::MatrixXd Q;  // 过程噪声协方差矩阵
    Eigen::Matrix3d R1; // 定义观测噪声协方差矩阵
    Eigen::Matrix3d R2;

    EKF() {
        x = Eigen::VectorXd(3); // [x, y, yaw]
        x << 0, 0, 0;

        P = Eigen::MatrixXd::Identity(3, 3);

        Q = 0.01 * Eigen::MatrixXd::Identity(3, 3);
        R1 << 0.03 * 0.03, 0, 0,
            0, 0.03 * 0.03, 0,
            0, 0, (3.0 * M_PI / 180.0) * (3.0 * M_PI / 180.0);
        R2 << 0.03 * 0.03, 0, 0,
            0, 0.03 * 0.03, 0,
            0, 0, (2.0 * M_PI / 180.0) * (2.0 * M_PI / 180.0);
    }

    void predict(const Eigen::Vector3d &delta_x) {
        // 状态转移矩阵 F
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(3, 3);

        // 状态预测
        x = F * x + delta_x; //变化值
        // 状态协方差预测
        P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::Vector3d &z, const Eigen::Matrix3d &R) {
        // 观测矩阵 H
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3, 3);
        // 创新
        Eigen::VectorXd y = z - H * x;
        // 创新协方差
        Eigen::MatrixXd S = H * P * H.transpose() + R;
        // 卡尔曼增益
        Eigen::MatrixXd K = P * H.transpose() * S.inverse();
        // 状态更新
        x = x + K * y;
        // 状态协方差更新
        P = (Eigen::MatrixXd::Identity(3, 3) - K * H) * P;
    }
};

class RTKOdom {
public:
    RTKOdom(const std::string &config_file);
    ~RTKOdom();

    //data
    void inputGnssMsg(const GnssData &rtk_msg);
    void inputOdomMsg(const OdometryData &odom_msg);
    Gnss_With_DrOdom getFilteredRtk();
    OdometryData getOdometry();
    OdometryData fusion20ms_with_rtk(
        OdometryData &dr_update_pose, //dr_odom更新数据
        Gnss_With_DrOdom &align_rtk_dr_pose);
    const int RTK_BUF_SIZE = 31;
    EKF rtk_fusion_dr_;

private:
    int calculate_rtk_with_odom(std::list<OdometryData> &odomData, GnssData &rtk_pos);
    int convertLLA_insertToHoldList(std::list<GnssData> &lRtkData, const std::list<GnssData> &rtk_buf);

    int tryCalcTransOdomToRtk(std::list<GnssData> &lRtkData, const std::list<OdometryData> &lOdomData);
    void process_rtk_odom();
    void updateOutputPos(const Gnss_With_DrOdom pub_pos);
    bool EstimateYawInENU(Eigen::Vector3d in_pose, double &out_yaw);
    //save log
    std::ofstream ofLogOut;
    std::ofstream ofDataOut;
    std::ofstream ofRtkOriginOut;
    std::ofstream ofRtkKalman;
    std::ofstream ofRtkTrajectory;
    //data
    std::mutex mutex_rtk_data_;
    std::mutex mutex_odom_data_;
    std::mutex mutex_output_pos_;
    std::list<GnssData> rtk_buf_;
    std::list<OdometryData> odom_buf_;
    std::list<GnssData> RtkDataBuf;
    std::list<OdometryData> OdomDataBuf;
    bool bfirstRtkFlag = true;       // init lla with enu
    bool bRTkOdomInitedFlag = false; // init with odom
    LLAToENUConvertor trans_to_enu_;
    GnssData originRtkPosData;
    GnssData last_rtk_data;
    bool LastForwordFlag = false;
    double LastRtkDiff = 0.0;

    bool getTransFlag_ = false;
    Eigen::Matrix<double, 4, 4> T_rtkToOdom = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 4> T_OdomToRtk = Eigen::Matrix<double, 4, 4>::Identity();
    std::pair<double, double> CalAglDif_ = std::make_pair(0.0, 0.0);
    double error_sum;
    Eigen::Vector3d g_last_rtk = {-1000, -1000, -1000};
    double est_yaw[2] = {-1000, -1000};
    Gnss_With_DrOdom filtered_pos;
    Gnss_With_DrOdom publish_pos; //sendout
    OdometryData pub_pose_;
    OdometryData last_odom_pose_;
    OdometryData last_dr_pose_;
    std::thread thd_process;
    bool start_thd_ = true;
    Gnss_With_DrOdom last_align_rtk_dr_pos_;

    OdometryData last_predict_pose_;
    std::deque<OdometryData> dr_que_;
    std::deque<Gnss_With_DrOdom> algin_gnss_dr_que_;
    float fDeltaTheta_ = 0.;
    float fCosA_ = 1.;
    float fSinA_ = 0.;
    int nCalculateJump_ = 0;

    float mfThJumpIn20ms = 0.04; // unit: meter
    float mfJumpStep = 0.03;
    float mfJumpLen = 0;
    // if mfJumpLen > mfMaxJump; then do not force-filter
    float mfMaxJump = 30; // filter step norm
};

} // namespace rtk_odom_component

#endif //end RTK_ODOM_H
