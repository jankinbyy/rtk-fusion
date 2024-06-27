#include <thread>
#include <unistd.h>
#include <math.h>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include "rtk_odom.h"
using namespace std;

namespace rtk_odom_component {
static const double mRtkCov = 0.05 * 0.05;
static const double mOdomCov = 0.001 * 0.001;
static double mKFilter = mOdomCov;
static double mLastAngle = 0;
static double mfModefyDelta = 0;
static double mfDegreeAlert = 0;
static const double mfAngleAlertThInDegree = 3;
static double TempAngleDiff = 0.0;
static double LastAngleDiff = 0.;
//static double mfPairedRtkOdomTimeDiff_s = 0.005;
static double mfPairedRtkOdomTimeDiff_s = 0.03;

RTKOdom::RTKOdom(const string &config_file) {
    Config::readConfig(config_file);
    string save_log_path = Config::log_;
    ofLogOut.open(save_log_path + "/rtk_odom/cmd_rtk_odom.txt");
    ofLogOut << "init RTKOdom" << endl;
    ofRtkKalman.open(save_log_path + "/rtk_odom/rtk_kalman.txt");
    ofLogOut << std::fixed << std::setprecision(3);
    ofRtkKalman << std::fixed << std::setprecision(3);
    thd_process = std::thread(&RTKOdom::process_rtk_odom, this);
    ofLogOut << "init RTKOdom ok" << endl;
    rtk_odom_log_.Init("rtk odom", 3, "../log/rtk_odom.log"); //3
}
RTKOdom::~RTKOdom() {
    start_thd_ = false;
    thd_process.join();
    std::cerr << "rtk odom class over!" << std::endl;
}
void RTKOdom::Stop() {
    start_thd_ = false;
}
Gnss_With_DrOdom RTKOdom::getFilteredRtk() {
    Gnss_With_DrOdom tmp;
    mutex_output_pos_.lock();
    tmp = publish_pos;
    mutex_output_pos_.unlock();
    return tmp;
}
void RTKOdom::updateOutputPos(const Gnss_With_DrOdom pub_pos) {
    mutex_output_pos_.lock();
    publish_pos = pub_pos;
    mutex_output_pos_.unlock();
}
//data
void RTKOdom::inputGnssMsg(const GnssData &rtk_msg) {
    if (rtk_msg.status == 4) {
        if (!rtk_stt_.first) {
            rtk_stt_.second = true;
            rtk_odom_log_(4, "rtk_turn_fix");
        } else {
            rtk_stt_.second = false;
        }
        rtk_stt_.first = true;
        mutex_rtk_data_.lock();
        rtk_buf_.push_back(rtk_msg);
        RtkDataBuf.push_back(rtk_msg);
        while (RtkDataBuf.size() > RTK_BUF_SIZE) {
            RtkDataBuf.pop_front();
        }
        mutex_rtk_data_.unlock();
    } else {
        if (rtk_stt_.first) {
            rtk_odom_log_(4, "rtk_turn_float");
            rtk_stt_.second = true;
            rtk_stt_.first = false;
        } else {
            rtk_stt_.first = false;
            rtk_stt_.second = false;
        }
    }
}
void RTKOdom::inputOdomMsg(const OdometryData &odom_msg) {
    mutex_odom_data_.lock();
    odom_buf_.push_back(odom_msg);
    OdomDataBuf.push_back(odom_msg);
    // if (RtkDataBuf.size() > 0)
    //     std::cout << std::fixed << std::setprecision(9) << "time rtk:" << RtkDataBuf.back().timestamp << "odom" << odom_msg.time << std::endl;
    while (OdomDataBuf.size() > RTK_BUF_SIZE * 25)
        OdomDataBuf.pop_front();
    mutex_odom_data_.unlock();
}

static bool AlignCoordinate(const std::list<Eigen::Vector3d> &global_data, const std::list<Eigen::Vector3d> &local_data,
                            Eigen::Matrix<double, 4, 4> &L_T_G, double &error_sum) {
    if (global_data.size() != local_data.size()) {
        std::cout << "error align coordinate size!" << std::endl;
        std::abort();
    }
    if (global_data.size() < 3) {
        std::cout << "align coordinate global_data.size = " << global_data.size() << std::endl;
        return false;
    }
    Eigen::Vector3d p1(0., 0., 0.), p2(0., 0., 0.);
    int N = int(global_data.size());

    double max_dist = 0.0;

    auto itr2 = local_data.begin();
    for (auto itr1 = global_data.begin(); itr1 != global_data.end(); ++itr1, ++itr2) {
        p1 += (*itr1);
        p2 += (*itr2);
        max_dist = std::fmax(max_dist, ((*itr1) - global_data.front()).norm());
    }

    if (max_dist < 0.2) {
        std::cout << "too short to AlignCooToORB_2D" << std::endl;
        return false;
    }

    p1 /= N;
    p2 /= N;

    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();

    itr2 = local_data.begin();
    for (auto itr1 = global_data.begin(); itr1 != global_data.end(); ++itr1, ++itr2) {
        Eigen::Vector3d q1 = (*itr1) - p1;
        Eigen::Vector3d q2 = (*itr2) - p2;
        W += q1 * q2.transpose();
    }

    //SVD on W
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    if (U.determinant() * V.determinant() < 0) {
        for (int x = 0; x < 2; ++x)
            U(x, 1) *= -1;
    }

    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = p1 - R_ * p2;

    Eigen::Matrix<double, 4, 4> Trans_ = Eigen::Matrix<double, 4, 4>::Identity();
    Trans_.block<3, 3>(0, 0) = R_;
    Trans_.block<3, 1>(0, 3) = t_;
    //check the error of R t TODO:2D
    double dist_error = 0.0;
    auto pos1 = global_data.begin();
    for (auto const &pos : local_data) {
        dist_error += (R_ * pos + t_ - (*pos1)).norm();
        if (pos1 != global_data.end())
            pos1++;
        else
            break;
    }
    error_sum = dist_error;
    if ((dist_error / double(double(global_data.size()) + 0.01) > 0.5)) {
        std::cout << "AlignCoordi big error error_mean = "
                  << dist_error / double(double(global_data.size()) + 0.01)
                  << "  cm  Num: " << global_data.size() << std::endl;
        return false;
    } else {
        L_T_G = Trans_.inverse();
        std::cout << "agular is:" << L_T_G.block<3, 3>(0, 0).eulerAngles(0, 1, 2).z() << std::endl;
        return true;
    }
}

int RTKOdom::convertLLA_insertToHoldList(list<GnssData> &lRtkData, const std::list<GnssData> &rtk_buf) {
    auto itr_rtk = rtk_buf.begin();
    while (itr_rtk != rtk_buf.end()) {
        GnssData gnss_pos;
        gnss_pos.timestamp = itr_rtk->timestamp;

        int ret = 0;
        if (ret = trans_to_enu_.convertLLA2ENU(itr_rtk->LonLatAlt.x(), itr_rtk->LonLatAlt.y(), itr_rtk->LonLatAlt.z(),
                                               gnss_pos.xyz.x(), gnss_pos.xyz.y(), gnss_pos.xyz.z())) {
            lRtkData.push_back(gnss_pos);
        } else if (ret == 0) { //means setOrigin
            if (bfirstRtkFlag) {
                originRtkPosData = gnss_pos;
                bfirstRtkFlag = false;
                ofLogOut << "origin " << gnss_pos.timestamp << " " << gnss_pos.xyz.x() << " " << gnss_pos.xyz.y() << " " << gnss_pos.xyz.z() << std::endl;
            }
            lRtkData.push_back(gnss_pos);
        } else {
            ofLogOut << "fail trans lla " << itr_rtk->timestamp << std::endl;
        }
        itr_rtk++;
    }
    return rtk_buf.size();
}

//-1, no odom, pls wait
//-2, no got one, pls wait
//1, no nearest one(may lost  odom data), one, can pop front
//0, got one,can pop front
static int queryBestTimeOdom(const GnssData &rtk, const std::list<OdometryData> &lOdomData, OdometryData &outOdom) {
    if (lOdomData.size() == 0)
        return -1;
    if (lOdomData.back().time <= rtk.timestamp)
        return -2;
    auto lit = lOdomData.begin();
    double aim_time_s = rtk.timestamp;
    double min_time_s = fabs(lit->time - aim_time_s);
    OdometryData bestOdom = *lit;
    while (lit != lOdomData.end()) {
        double time_diff0 = fabs(lit->time - aim_time_s);
        if (time_diff0 < min_time_s) {
            min_time_s = time_diff0;
            bestOdom = *lit;
        }
        if (lit->time > rtk.timestamp)
            break;
        lit++;
    }

    if (min_time_s < mfPairedRtkOdomTimeDiff_s) {
        outOdom = bestOdom;
        return 0;
    }
    return 1;
}

int RTKOdom::tryCalcTransOdomToRtk(list<GnssData> &lRtkData, const std::list<OdometryData> &lOdomData) {
    if (lRtkData.size() < 20) {
        return -1;
    }
    OdometryData bestOdom;
    int ret_query = 1;
    while (lRtkData.size() && (ret_query = queryBestTimeOdom(lRtkData.front(), lOdomData, bestOdom)) > 0) {
        lRtkData.pop_front();
    }
    if (lRtkData.size() == 0 || ret_query < 0) {
        ofLogOut << " lOdomData front time " << lOdomData.front().time << " back-time " << lOdomData.back().time << " ";
        if (lRtkData.size() == 0)
            ofLogOut << " lRtkData.size()==0" << endl;
        else
            ofLogOut << " ret_query < 0 " << lRtkData.front().timestamp << " ret_query = " << ret_query << endl;
        return -1;
    }
    float min_dist_rtk = 0.05;
    auto lit = lRtkData.begin();
    list<GnssData> lSelectRtk;
    list<OdometryData> lSelectOdom;
    std::list<Eigen::Vector3d> lOdomXY, lRtkXY;

    //first pair
    lRtkXY.push_back(lit->xyz);
    lOdomXY.push_back(Eigen::Vector3d(bestOdom.position(0), bestOdom.position(1), 0.0));
    lSelectOdom.push_back(bestOdom);
    lSelectRtk.push_back(*lit);

    GnssData last_rtk = *lit;

    while (lit != lRtkData.end()) {
        float dx = (last_rtk.xyz.x() - lit->xyz.x());
        float dy = (last_rtk.xyz.y() - lit->xyz.y());
        float dist_ = sqrt(dx * dx + dy * dy);
        if (dist_ > min_dist_rtk) {
            if (0 == queryBestTimeOdom(*lit, lOdomData, bestOdom)) {
                //paired
                lRtkXY.push_back(lit->xyz);
                lOdomXY.push_back(Eigen::Vector3d(bestOdom.position(0), bestOdom.position(1), 0.0));
                lSelectRtk.push_back(*lit);
                lSelectOdom.push_back(bestOdom);
                last_rtk = *lit;

                if (lRtkXY.size() > 13)
                    break;
            }
        }
        lit++;
    }
    if (lRtkXY.size() < 13) {
        ofLogOut << " lRtkXY.size() < 13 " << lRtkData.front().timestamp << " lRtkXY.size() " << lRtkXY.size() << endl;
        return -1;
    }

    auto litRbag = lSelectRtk.begin();
    auto litObag = lSelectOdom.begin();
    // ofLogOut << fixed << setprecision(6);
    for (auto litR = lRtkXY.begin(), litO = lOdomXY.begin(); litR != lRtkXY.end(); litR++, litO++) {
        //ofLogOut << litRbag->time << " " << litR->transpose() << " " << litObag->time << " " << litO->transpose() << endl;
        litRbag++;
        litObag++;
    }
    if (!AlignCoordinate(lRtkXY, lOdomXY, IMU_T_ENU_, error_sum)) {
        std::cout << "AlignCoordina failure !!!" << std::endl;
        //failed
        lRtkData.pop_front();
        return -1;
    }
    return 0;
}
void RTKOdom::CalRtkYaw(list<GnssData> &_rtk_buf, list<GnssData> &_lRtkData) {
    if (_rtk_buf.size() > 0) {
        auto itr_rtk = _rtk_buf.begin();
        while (itr_rtk != _rtk_buf.end()) {
            if (itr_rtk->cov(0, 0) + itr_rtk->cov(1, 1) < 0.003) {
                Eigen::Vector3d XYZRtkFrame = itr_rtk->xyz;
                double out_yaw1 = 0.0;
                if (EstimateYawInENU(XYZRtkFrame, out_yaw1)) {
                    itr_rtk->rpy[2] = out_yaw1;
                }
            }
            _lRtkData.push_back(*itr_rtk);
            itr_rtk++;
        }
    }
}
//main process
void RTKOdom::process_rtk_odom() {
    list<GnssData> lRtkData;
    std::list<OdometryData> lOdomData;
    while (start_thd_) {
        //prepare data
        bool has_rtk_data = false;
        bool has_odom_data = false;
        std::list<GnssData> rtk_buf;
        std::list<GnssData> svd_rtk_buf;
        mutex_rtk_data_.lock();
        if (rtk_buf_.size() > 0) {
            has_rtk_data = true;
            rtk_buf.swap(rtk_buf_);
        }
        svd_rtk_buf = RtkDataBuf;
        mutex_rtk_data_.unlock();
        std::list<OdometryData> svd_odom_buf;
        mutex_odom_data_.lock();
        if (odom_buf_.size() > 0) {
            has_odom_data = true;
            lOdomData.insert(lOdomData.end(), odom_buf_.begin(), odom_buf_.end());
            odom_buf_.clear();
        }
        svd_odom_buf = OdomDataBuf;
        mutex_odom_data_.unlock();
        if (has_odom_data) {
            OdometryData tmp_out = sync_dr_rtk(lOdomData.back(), filtered_pos); //预测
            mutex_output_pos_.lock();
            pub_pose_ = tmp_out;
            mutex_output_pos_.unlock();
        }
        if (has_rtk_data) {
            CalRtkYaw(rtk_buf, lRtkData);
            auto itr_rtk = lRtkData.begin();
            while (itr_rtk != lRtkData.end() && lOdomData.size()) {
                if (itr_rtk->timestamp > lOdomData.back().time) {
                    break;
                }
                GnssData cur_gnss_pos = *itr_rtk;
                int ret_cal = calculate_rtk_with_odom(lOdomData, cur_gnss_pos);
                if (ret_cal == -2) { //wait for next odom
                    cerr << fixed << setprecision(6) << " pls wait next odom " << cur_gnss_pos.timestamp << endl;
                    break;
                } else if (ret_cal == 0) {
                    updateOutputPos(filtered_pos);
                }
                itr_rtk++;
                lRtkData.pop_front();
            } //end while rtk
            if (0) {
                if (tryCalcTransOdomToRtk(svd_rtk_buf, svd_odom_buf) == 0) {
                    getTransFlag_ = true;
                }
            }
        }
        usleep(5000); //10ms
    }                 //end while
    std::cout << "process_rtk_odom_over" << std::endl;
}

void resetAngleFilter() {
    mKFilter = mOdomCov;
    mfModefyDelta = 0;
    mLastAngle = 0;
}

double getLastAngle() {
    return mLastAngle;
}

double filter_angle(double angle_diff00, double &KalmanCov) {
    double diff00 = restrict_angle_range(angle_diff00 - mLastAngle);

    double ret_angle = restrict_angle_range(mLastAngle + diff00 * mKFilter / (mKFilter + mRtkCov));
    mKFilter = mKFilter * mRtkCov / (mKFilter + mRtkCov);
    KalmanCov = mKFilter;
    mLastAngle = ret_angle;

    {
        double degree = mLastAngle * 180 / M_PI;
        if (fabs(degree - mfDegreeAlert) > mfAngleAlertThInDegree) {
            mfDegreeAlert = degree;
        }
    }

    return ret_angle;
}

void restartFilter(double angleInput) {
    mKFilter = mOdomCov;
    mLastAngle = restrict_angle_range(angleInput);
}

void updateAngleFilterCov() {
    mKFilter = mKFilter + mOdomCov;
}
//-2, pls wait the next odom; break loop outside;
//-1; give up this one, loop continue pls; do not pub pose
//0, normal; outside pub-pose;
int RTKOdom::calculate_rtk_with_odom(std::list<OdometryData> &odomData, GnssData &rtk_pos) {
    OdometryData TmpOdomData;
    TmpOdomData.covariance[0] = 0.03;
    TmpOdomData.covariance[1] = 0.03;
    TmpOdomData.covariance[2] = 0.03;
    TmpOdomData.covariance[5] = 3 * kDeg2Rad; //默认航向误差为180度
    while (odomData.size() && odomData.front().time < rtk_pos.timestamp) {
        TmpOdomData = odomData.front(); //rtk的前一帧odom数据
        odomData.pop_front();
    }
    if (odomData.size() == 0) {
        odomData.push_back(TmpOdomData);
        return -2;
    }
    //选则最近的odom作为tmpOdom
    if ((odomData.front().time - rtk_pos.timestamp) < (rtk_pos.timestamp - TmpOdomData.time))
        TmpOdomData = odomData.front();
    filtered_pos.status = 0;
    filtered_pos.time = rtk_pos.timestamp;
    if (abs(rtk_pos.rpy[2]) < M_PI) {
        {
            filtered_pos.status = 1; //rtk角度
        }
    }
    filtered_pos.SetData(rtk_pos, TmpOdomData);
    return 0;
}
OdometryData RTKOdom::sync_dr_rtk(
    OdometryData &dr_update_pose,          //dr_odom更新数据
    Gnss_With_DrOdom &align_rtk_dr_pose) { //rtk与当时dr_odom对齐的数据，内涵rtk和dr_odom数据
    OdometryData tmp_dr_pose = dr_update_pose;
    if (dr_que_.size()) {
        if (dr_que_.back().time != tmp_dr_pose.time)
            dr_que_.push_back(tmp_dr_pose);
        while (dr_que_.size() > 200) {
            dr_que_.pop_front();
        }
    } else {
        dr_que_.push_back(tmp_dr_pose);
    }
    Gnss_With_DrOdom align_rtk_dr = align_rtk_dr_pose;
    bool g_align_pose_update = false;
    if (last_align_rtk_dr_pos_.time != align_rtk_dr.time) { //如果有更新。更新队列
        g_align_pose_update = true;
        algin_gnss_dr_que_.push_back(align_rtk_dr);
        while (algin_gnss_dr_que_.size() > 10) {
            algin_gnss_dr_que_.pop_front();
        }
    }
    if (align_rtk_dr.status < 0) {
        return tmp_dr_pose;
    }
    OdometryData cur_predict_pos; //修复rtk position与yaw的延时
    if (g_align_pose_update) {    //rtk准确,更新直接更新,未更新直接递推
        if (align_rtk_dr.status == 1) {
            if (dr_que_.size()) {
                while (dr_que_.size()) {
                    if (dr_que_[0].time < align_rtk_dr.time && dr_que_[1].time > align_rtk_dr.time) {
                        OdometryData close_dr;
                        if (abs(dr_que_[0].time - align_rtk_dr.time) > abs(dr_que_[1].time - align_rtk_dr.time)) {
                            close_dr = dr_que_[1];
                        } else {
                            close_dr = dr_que_[0];
                        }
                        //std::cout << "repair time:" << abs(align_rtk_dr.time - close_dr.time) << "," << dr_que_.back().time - close_dr.time << std::endl;
                        align_rtk_dr.gnss_pose_.xyz += Eigen::Vector3d{cos(align_rtk_dr.gnss_pose_.rpy[2]) * (dr_que_.back().position - close_dr.position).norm(), sin(align_rtk_dr.gnss_pose_.rpy[2]) * (dr_que_.back().position - close_dr.position).norm(), 0};
                        align_rtk_dr.gnss_pose_.rpy[2] += dr_que_.back().rpy[2] - close_dr.rpy[2];
                        align_rtk_dr.gnss_pose_.timestamp = dr_que_.back().time;
                        algin_gnss_dr_que_.back().gnss_pose_.rpy[2] = align_rtk_dr.gnss_pose_.rpy[2]; //时间对齐
                        break;
                    } else {
                        dr_que_.pop_front();
                    }
                }
            }
        } else {
            if (dr_que_.size()) {
                while (dr_que_.size()) {
                    if (dr_que_[0].time < align_rtk_dr.time && dr_que_[1].time > align_rtk_dr.time) {
                        OdometryData close_dr;
                        if (abs(dr_que_[0].time - align_rtk_dr.time) > abs(dr_que_[1].time - align_rtk_dr.time)) {
                            close_dr = dr_que_[1];
                        } else {
                            close_dr = dr_que_[0];
                        }
                        //std::cout << "repair time:" << abs(align_rtk_dr.time - close_dr.time) << "," << dr_que_.back().time - close_dr.time << std::endl;
                        double rtk_yaw = cur_predict_pos.rpy.z() - (dr_que_.back().rpy[2] - close_dr.rpy[2]);
                        align_rtk_dr.gnss_pose_.xyz += Eigen::Vector3d{cos(rtk_yaw) * (dr_que_.back().position - close_dr.position).norm(), sin(rtk_yaw) * (dr_que_.back().position - close_dr.position).norm(), 0};
                        align_rtk_dr.gnss_pose_.timestamp = dr_que_.back().time;
                        break;
                    } else {
                        dr_que_.pop_front();
                    }
                }
            }
        }
    }
    if (!rtk_stt_.first && rtk_stt_.second && algin_gnss_dr_que_.size() > 0) //rtk变差
    {
        rtk_stt_.second = false;
        for (int i = algin_gnss_dr_que_.size() - 1; i > 0; i--)
            if (algin_gnss_dr_que_[i].status == 1)
                if (((last_predict_pose_.position - algin_gnss_dr_que_[i].gnss_pose_.xyz).norm() > 2) && (last_predict_pose_.time - algin_gnss_dr_que_[i].gnss_pose_.timestamp < 5.0)) {
                    std::cout << "repair yaw,now dr yaw:" << last_dr_pose_.rpy.z()
                              << " last dr yaw:" << algin_gnss_dr_que_[i].dr_pose_.rpy.z()
                              << " last abs yaw:" << algin_gnss_dr_que_[i].gnss_pose_.rpy.z()
                              << " now abs yaw:" << last_predict_pose_.rpy.z()
                              << std::endl;
                    align_rtk_dr.gnss_pose_.rpy[2] = algin_gnss_dr_que_[i].gnss_pose_.rpy.z() + tmp_dr_pose.rpy.z() - algin_gnss_dr_que_[i].dr_pose_.rpy.z();
                    Eigen::Vector3d delta_pos = E3dEulerToMatrix(align_rtk_dr.gnss_pose_.rpy) * (Eigen::Vector3d((tmp_dr_pose.position - last_dr_pose_.position).norm(), 0.0, 0.0));
                    Eigen::Matrix3d J_prev_cur_imu = Eigen::Matrix3d::Identity();
                    if (abs(last_predict_pose_.rpy[2] - align_rtk_dr.gnss_pose_.rpy[2]) > 0.00001) {
                        Eigen::Matrix3d R_prev_cur_imu = E3dEulerToMatrix(last_predict_pose_.rpy).transpose() * E3dEulerToMatrix(align_rtk_dr.gnss_pose_.rpy); // Imu获取角度估计,计算变化值
                        Eigen::AngleAxisd delta_se3(R_prev_cur_imu);
                        double phi = delta_se3.angle();
                        Eigen::Vector3d axis = delta_se3.axis();
                        if (abs(phi) > 0.00001)
                            J_prev_cur_imu = sin(phi) / phi * Eigen::Matrix3d::Identity() + (1 - sin(phi) / phi) * axis * axis.transpose() + ((1 - cos(phi)) / phi) * skew(axis); //视觉14讲，p73 4.26公式,右乘一个微小位移
                    }
                    delta_pos = J_prev_cur_imu * delta_pos;
                    align_rtk_dr.gnss_pose_.xyz = last_predict_pose_.position + delta_pos;
                    g_align_pose_update = true;
                }
    }
    if (g_align_pose_update) {
        ekf_sp_.Update(align_rtk_dr.gnss_pose_);
    } else {
        ekf_sp_.Predict(tmp_dr_pose);
    }
    OdometryData eskf_sp_out;
    ekf_sp_.GetPose(eskf_sp_out);
    cur_predict_pos.time = tmp_dr_pose.time;
    cur_predict_pos.position = eskf_sp_out.position;
    cur_predict_pos.rpy.z() = eskf_sp_out.rpy.z();
    last_predict_pose_ = cur_predict_pos;
    last_align_rtk_dr_pos_ = align_rtk_dr;
    cur_predict_pos.rpy[0] = 0;
    cur_predict_pos.rpy[1] = 0;
    last_dr_pose_ = tmp_dr_pose;
    if (std::isnan(cur_predict_pos.position(0)) || std::isinf(cur_predict_pos.position(0))) {
        std::cout << "cur position:" << cur_predict_pos.position << std::endl;
        std::cin.get(); // 暂停程序，等待用户输入
    }
    OdometryData pub_pose =
        OdometryData(
            cur_predict_pos.time, "rtk_with_dr", cur_predict_pos.position,
            euler3d_to_quaternion(cur_predict_pos.rpy));

    return pub_pose;
}
bool RTKOdom::EstimateYawInENU(Eigen::Vector3d in_pose, double &out_yaw) {
    bool tmp_ret = false;
    double ang_err = 1.0;
    double len_dis = 0.1;
    if (!getTransFlag_)
        ang_err = 3.0;
    if (!getTransFlag_)
        len_dis = 0.05;
    if (rtk_stt_.second) {
        g_last_rtk = in_pose;
        rtk_odom_log_(4, "rtk_turn_fix,donot estimator yaw");
    }
    double tmp_length = (g_last_rtk - in_pose).norm();
    if (tmp_length > len_dis && tmp_length < 0.8) {
        est_yaw[1] = atan2(in_pose[1] - g_last_rtk[1], in_pose[0] - g_last_rtk[0]) * 180.0 / M_PI;
        if (abs(est_yaw[1] - est_yaw[0]) < ang_err) {
            out_yaw = est_yaw[1] * M_PI / 180.0;
            tmp_ret = true;
        }
        est_yaw[0] = est_yaw[1];
        g_last_rtk = in_pose;
    } else {
        g_last_rtk = in_pose;
    }
    return tmp_ret;
}
OdometryData RTKOdom::getOdometry() {
    OdometryData tmp;
    mutex_output_pos_.lock();
    tmp = pub_pose_;
    mutex_output_pos_.unlock();
    return tmp;
}
} // namespace rtk_odom_component
