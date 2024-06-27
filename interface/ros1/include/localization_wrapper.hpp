#pragma once

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <shared_mutex>
#include <thread>
#include <vector>
#include "dr_odom2_flow.hpp"
#include "dr_odo_flow.hpp"
#include "map_command.h"
#include "rtk_odom.h"
#include "sensor_type.h"
#include "config.h"
#include "GeocentricENU.hpp"
#include <nav_msgs/Odometry.h>
#define DR_ODOM1 false
#define DR_ODOM2 true
class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle &nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);
    void WheelCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr);
    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr);
    void Evaluate();
    void EvaluateRTK();
    void ConvertStateToRosTopic(Eigen::Matrix<double, 7, 1> pose_out,
                                nav_msgs::Path &in_path);
    void ConvertStateToRosTopic(
        OdometryData pose_out, nav_msgs::Path &in_path);
    void Function_Ctl_();

private:
    ros::Subscriber imu_sub_;
    ros::Subscriber wheel_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher odom2_pub_;
    ros::Publisher odom1_pub_;
    ros::Publisher rtk_dr_pub_;
    ros::Publisher rtk_true_pub_;
    ros::Publisher odom_pub_;
    std::thread run_thread_;
    std::thread run_thread_rtk_;
    std::condition_variable_any data_condition_;
    std::condition_variable_any rtk_dr_condition_;
    std::ofstream file_fusion_pos_;
    std::ofstream file_gps_;
    std::shared_timed_mutex m_data_mutex_;
    std::shared_timed_mutex rtk_dr_mutex_;
    nav_msgs::Path dr2_path_;
    nav_msgs::Path dr1_path_;
    nav_msgs::Path rtk_path_;
    nav_msgs::Path rtk_true_path_;
    std::deque<IMUData> imu_buf_;
    std::deque<VelocityData> wheelOdom_buf_;
    std::deque<GnssData> rtk_buf_;
    std::deque<OdometryData> dr_buf_;
    std::unique_ptr<DROdom2::DrOdoFlow2> imu_wheel_localizer_ptr_;
    std::unique_ptr<DrOdoFlow> imu_wheel_loc_ptr_;
    std::shared_ptr<rtk_odom_component::RTKOdom> filterangle_odom_rtk_ptr_ =
        nullptr;
    double last_reveive_rtk_time;
    Eigen::Matrix4d matrix_body_to_wheel_ = Eigen::Matrix4d::Identity();
    Gnss_With_DrOdom cur_gwd_pos_;
    bool start_alg_ = true;
    std::unique_ptr<LocalCartesianENU> gps2_enu_ptr_;
    DebugMode ros_wrapper_log_;
};