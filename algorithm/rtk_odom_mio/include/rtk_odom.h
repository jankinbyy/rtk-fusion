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
#include "ekf_sample.hpp"
#include "config.h"
using namespace sensor_msgs_z;
namespace rtk_odom_component {

class RTKOdom {
public:
    RTKOdom(const std::string &config_file);
    ~RTKOdom();
    void Stop();
    //data
    void inputGnssMsg(const GnssData &rtk_msg);
    void inputOdomMsg(const OdometryData &odom_msg);
    Gnss_With_DrOdom getFilteredRtk();
    OdometryData getOdometry();
    void CalRtkYaw(list<GnssData> &_rtk_buf, list<GnssData> &_lRtkData);
    OdometryData sync_dr_rtk(OdometryData &dr_update_pose, Gnss_With_DrOdom &align_rtk_dr_pose);
    const int RTK_BUF_SIZE = 31;
    DebugMode rtk_odom_log_;
    EKF_S ekf_sp_;

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
    Eigen::Matrix4d IMU_T_ENU_ = Eigen::Matrix4d::Identity(); //Imu到enu坐标系的变换矩阵
    double true_dif_ = 0.0;
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
    std::pair<bool, bool> rtk_stt_ = std::make_pair(true, false); //first status,second turn

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
