#include "localization_wrapper.hpp"

#include <glog/logging.h>

#include <iomanip>

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle &nh) {
    // Load configs.
    std::string config_file = "../config/config.yaml";
    std::string log_folder = "../log";
    Config::readConfig(config_file);
    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder + "/gps.csv");

    // Initialization imu gps localizer.
    Eigen::Matrix4d mTvi = Eigen::Matrix4d::Identity();
    mTvi << 0, -1, 0, 0.0000, 1, 0, 0, -0.812, 0, 0, 1, -0.483, 0, 0, 0,
        1; // imu_T_wheel
    matrix_body_to_wheel_ = mTvi;
    imu_wheel_loc_ptr_ = std::make_unique<DrOdoFlow>(mTvi.inverse());
    imu_wheel_localizer_ptr_ =
        std::make_unique<DROdom2::DrOdoFlow2>(mTvi.inverse());
    filterangle_odom_rtk_ptr_ =
        std::make_shared<rtk_odom_component::RTKOdom>(config_file);
    gps2_enu_ptr_ = std::make_unique<LocalCartesianENU>();
    imu_sub_ =
        nh.subscribe(Config::imu_topic_, 10, &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe(
        Config::rtk_topic_, 10, &LocalizationWrapper::GpsPositionCallback, this);
    wheel_sub_ =
        nh.subscribe(Config::wheel_topic_, 10, &LocalizationWrapper::WheelCallback, this);
    odom2_pub_ = nh.advertise<nav_msgs::Path>("odom2_path", 10);
    odom1_pub_ = nh.advertise<nav_msgs::Path>("odom1_path", 10);
    rtk_dr_pub_ = nh.advertise<nav_msgs::Path>("rtk_dr_path", 10);
    rtk_true_pub_ = nh.advertise<nav_msgs::Path>("rtk_true_path", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 50);
    run_thread_ = std::thread(&LocalizationWrapper::Evaluate, this);
    run_thread_rtk_ = std::thread(&LocalizationWrapper::EvaluateRTK, this);
}
void LocalizationWrapper::Evaluate() {
    while (start_alg_) {
        {
            std::unique_lock<std::shared_timed_mutex> lock(m_data_mutex_);
            data_condition_.wait(
                lock, [this] { return !imu_buf_.empty() || !wheelOdom_buf_.empty(); });
            while (!imu_buf_.empty()) {
                auto imu_msg = imu_buf_.front();
                imu_buf_.pop_front();
                // Unlock mutex during the processing to allow other threads to push data.
                lock.unlock();
                imu_wheel_localizer_ptr_->setImu(
                    imu_msg.timestamp, imu_msg.acc(0), imu_msg.acc(1), imu_msg.acc(2),
                    imu_msg.gyro(0), imu_msg.gyro(1), imu_msg.gyro(2));
                if (DR_ODOM1)
                    imu_wheel_loc_ptr_->setImu(
                        imu_msg.timestamp, imu_msg.acc(0), imu_msg.acc(1), imu_msg.acc(2),
                        imu_msg.gyro(0), imu_msg.gyro(1), imu_msg.gyro(2));
                lock.lock();
            }

            // Process all Wheel Odom data available.
            while (!wheelOdom_buf_.empty()) {
                auto odom_msg = wheelOdom_buf_.front();
                wheelOdom_buf_.pop_front();
                // Similar unlock, process, and re-lock pattern.
                lock.unlock();
                imu_wheel_localizer_ptr_->setWheel(odom_msg.timestamp, odom_msg.vel(0),
                                                   odom_msg.gyr(2));
                if (DR_ODOM1)
                    imu_wheel_loc_ptr_->setWheel(odom_msg.timestamp, odom_msg.vel(0),
                                                 odom_msg.gyr(2));
                lock.lock();
            }
        }
        imu_wheel_localizer_ptr_->Optimization();
        if (DR_ODOM1)
            imu_wheel_loc_ptr_->Optimization();
        OdometryData pose_out;
        imu_wheel_localizer_ptr_->getPose(pose_out);
        ConvertStateToRosTopic(pose_out, dr2_path_);
        odom2_pub_.publish(dr2_path_);
        if (DR_ODOM1) {
            OdometryData pose_out_dr1;
            imu_wheel_loc_ptr_->getPose(pose_out_dr1);
            ConvertStateToRosTopic(pose_out_dr1, dr1_path_);
            odom1_pub_.publish(dr1_path_);
        }

        {
            std::unique_lock<std::shared_timed_mutex> lock_rtk(rtk_dr_mutex_);
            dr_buf_.push_back(pose_out);
        }
        rtk_dr_condition_.notify_one();
    }
}
void LocalizationWrapper::EvaluateRTK() {
    while (start_alg_) {
        std::unique_lock<std::shared_timed_mutex> lock_rtk(rtk_dr_mutex_);
        rtk_dr_condition_.wait(
            lock_rtk, [this] { return !rtk_buf_.empty() || !dr_buf_.empty(); });
        while (!rtk_buf_.empty()) {
            auto rtk_msg = rtk_buf_.front();
            rtk_buf_.pop_front();
            filterangle_odom_rtk_ptr_->inputGnssMsg(rtk_msg);
        }
        while (!dr_buf_.empty()) {
            auto odom_msg = dr_buf_.front();
            dr_buf_.pop_front();
            //lock_rtk.unlock();
            filterangle_odom_rtk_ptr_->inputOdomMsg(odom_msg);
            OdometryData pub_pose =
                filterangle_odom_rtk_ptr_->getOdometry();
            // std::cout << "fusion path:" << pub_pose.position << std::endl;
            ConvertStateToRosTopic(pub_pose, rtk_path_);
            rtk_dr_pub_.publish(rtk_path_);

            nav_msgs::Odometry odom;
            odom.header.stamp = ros::Time().fromSec(pub_pose.time);
            odom.header.frame_id = "world";
            //set the position
            odom.pose.pose.position.x = pub_pose.position[0];
            odom.pose.pose.position.y = pub_pose.position[1];
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.w = (float)pub_pose.rotation.w();
            odom.pose.pose.orientation.x = (float)pub_pose.rotation.x();
            odom.pose.pose.orientation.y = (float)pub_pose.rotation.y();
            odom.pose.pose.orientation.z = (float)pub_pose.rotation.z();
            //set the velocity
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;
            //publish the message
            odom_pub_.publish(odom);
            //lock_rtk.lock();
        }
    }
}
LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    start_alg_ = false;
    file_gps_.close();
    run_thread_.join();
    run_thread_rtk_.join();
    std::cerr << "localizationwrapper over" << std::endl;
}

void LocalizationWrapper::ImuCallback(
    const sensor_msgs::ImuConstPtr &imu_msg_ptr) {
    IMUDataPtr imu_data_ptr =
        std::make_shared<IMUData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x,
        imu_msg_ptr->linear_acceleration.y, imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
        imu_msg_ptr->angular_velocity.y, imu_msg_ptr->angular_velocity.z;
    {
        std::unique_lock<std::shared_timed_mutex> lock(m_data_mutex_);
        imu_buf_.push_back(*imu_data_ptr);
        while (imu_buf_.size() > 100) imu_buf_.pop_front();
    }
    data_condition_.notify_one();
}

void LocalizationWrapper::GpsPositionCallback(
    const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr) {
    // Check the gps_status.
    bool rtk_good = true;
    if (gps_msg_ptr->position_covariance[0] + gps_msg_ptr->position_covariance[4] > 0.003) { // RTK 噪声过大，认为是不可信的 fixme:是否存在中间的临界状态？
        rtk_good = false;
    }
    if (gps_msg_ptr->status.status != 4 || !rtk_good) {
        //LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }
    if (last_reveive_rtk_time == gps_msg_ptr->header.stamp.toSec()) return;

    Eigen::Matrix<double, 7, 1> pose_out;
    pose_out[0] = gps_msg_ptr->header.stamp.toSec();
    gps2_enu_ptr_->Forward(gps_msg_ptr->longitude, gps_msg_ptr->latitude, gps_msg_ptr->altitude, pose_out[1], pose_out[2], pose_out[3]);
    ConvertStateToRosTopic(pose_out, rtk_true_path_);
    rtk_true_pub_.publish(rtk_true_path_);
    GnssData gps_data(
        gps_msg_ptr->header.stamp.toSec(), gps_msg_ptr->longitude, gps_msg_ptr->latitude, gps_msg_ptr->altitude, pose_out[1], pose_out[2], pose_out[3], gps_msg_ptr->status.status,
        gps_msg_ptr->status.service);
    {
        std::unique_lock<std::shared_timed_mutex> lock_rtk(rtk_dr_mutex_);
        rtk_buf_.push_back(gps_data);
        last_reveive_rtk_time = gps_data.time;
    }
    rtk_dr_condition_.notify_one();
}
void LocalizationWrapper::WheelCallback(
    const geometry_msgs::TwistStampedConstPtr &twist_msg_ptr) {
    VelocityDataPtr wheel_data_ptr =
        std::make_shared<VelocityData>();
    wheel_data_ptr->timestamp = twist_msg_ptr->header.stamp.toSec();
    wheel_data_ptr->vel << twist_msg_ptr->twist.linear.x * Config::wheel_sx_,
        twist_msg_ptr->twist.linear.y, twist_msg_ptr->twist.linear.z;
    wheel_data_ptr->gyr << twist_msg_ptr->twist.angular.x,
        twist_msg_ptr->twist.angular.y, twist_msg_ptr->twist.angular.z;
    {
        std::unique_lock<std::shared_timed_mutex> lock(m_data_mutex_);
        wheelOdom_buf_.push_back(*wheel_data_ptr);
        while (wheelOdom_buf_.size() > 100) wheelOdom_buf_.pop_front();
    }
    data_condition_.notify_one();
}
void LocalizationWrapper::ConvertStateToRosTopic(
    Eigen::Matrix<double, 7, 1> pose_out, nav_msgs::Path &in_path) {
    in_path.header.frame_id = "world";
    in_path.header.stamp = ros::Time().fromSec(pose_out(0));
    geometry_msgs::PoseStamped pose;
    pose.header = in_path.header;
    pose.pose.position.x = pose_out(1);
    pose.pose.position.y = pose_out(2);
    pose.pose.position.z = pose_out(3);
    in_path.poses.push_back(pose);
}
void LocalizationWrapper::ConvertStateToRosTopic(
    OdometryData pose_out, nav_msgs::Path &in_path) {
    in_path.header.frame_id = "world";
    in_path.header.stamp = ros::Time().fromSec(pose_out.time);
    geometry_msgs::PoseStamped pose;
    pose.header = in_path.header;
    pose.pose.position.x = pose_out.position[0];
    pose.pose.position.y = pose_out.position[1];
    pose.pose.position.z = pose_out.position[2];
    in_path.poses.push_back(pose);
}