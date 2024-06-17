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
    YAML::Node config_node = YAML::LoadFile(config_file);
    YAML::Node common_config_node = config_node["common"];
    //    if(common_config_node["set_log_save_path"])
    string save_log_path = common_config_node["log_save_path"].as<std::string>();

    ofLogOut.open(save_log_path + "/rtk_odom/cmd_rtk_odom.txt");
    ofLogOut << "init RTKOdom" << endl;
    //ofDataOut.open(save_log_path + "/rtk_odom/filter_rtk_odom.txt");
    //ofRtkOriginOut.open(save_log_path + "/rtk_odom/rtk_origin.txt");
    //ofRtkTrajectory.open(save_log_path + "/rtk_odom/rtk_trajectory.txt");
    ofRtkKalman.open(save_log_path + "/rtk_odom/rtk_kalman.txt");

    ofLogOut << std::fixed << std::setprecision(3);
    //ofDataOut << std::fixed << std::setprecision(3);
    //ofRtkOriginOut << std::fixed << std::setprecision(8);
    // ofRtkTrajectory << std::fixed << std::setprecision(3);
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
        // if (rtk_stt_.second) {
        //     RtkDataBuf.clear();
        // }
        mutex_rtk_data_.unlock();
    } else {
        if (rtk_stt_.first)
            rtk_odom_log_(4, "rtk_turn_float");
        rtk_stt_.first = false;
    }
}
void RTKOdom::inputOdomMsg(const OdometryData &odom_msg) {
    mutex_odom_data_.lock();
    odom_buf_.push_back(odom_msg);
    OdomDataBuf.push_back(odom_msg);
    while (OdomDataBuf.size() > RTK_BUF_SIZE * 25)
        OdomDataBuf.pop_front();
    // if (rtk_stt_.second)
    //     OdomDataBuf.clear();
    mutex_odom_data_.unlock();
}

static bool AlignCoordinate(const std::list<Eigen::Vector3d> &global_data, const std::list<Eigen::Vector3d> &local_data,
                            Eigen::Matrix<double, 4, 4> &L_T_G, Eigen::Matrix<double, 4, 4> &G_T_L, double &CalAglDif, double &error_sum,
                            std::ofstream &ofLogOut) {
    if (global_data.size() != local_data.size()) {
        ofLogOut << "error align coordinate size!" << std::endl;
        std::abort();
    }

    if (global_data.size() < 3) {
        ofLogOut << "align coordinate global_data.size = " << global_data.size() << std::endl;
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
        ofLogOut << "too short to AlignCooToORB_2D" << std::endl;
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

    L_T_G = Trans_;

    G_T_L.block<3, 3>(0, 0) = L_T_G.block<3, 3>(0, 0).transpose();
    G_T_L.block<3, 1>(0, 3) = -L_T_G.block<3, 3>(0, 0).transpose() * L_T_G.block<3, 1>(0, 3);

    //calculate anguler
    Eigen::Vector3d eulerAngle = L_T_G.block<3, 3>(0, 0).eulerAngles(2, 1, 0);
    //std::cout << "init angle is:" << eulerAngle(0) << std::endl;
    if (abs(eulerAngle(1)) > M_PI / 2.0) {
        eulerAngle(0) = M_PI + eulerAngle(0);
        //std::cout << "add M_PI" << eulerAngle(0) << std::endl;
    }
    CalAglDif = restrict_angle_range(eulerAngle(0));
    //std::cout << "Cal agl Dif angular:" << CalAglDif << std::endl;
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

    if ((dist_error / double(double(global_data.size()) + 0.01) > 3.0)) {
        ofLogOut << "AlignCoordi big error error_mean = "
                 << dist_error / double(double(global_data.size()) + 0.01)
                 << "  cm  Num: " << global_data.size() << std::endl;
        return false;
    } else
        return true;
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
    if (!AlignCoordinate(lRtkXY, lOdomXY, T_OdomToRtk, T_rtkToOdom, CalAglDif_.second, error_sum, ofLogOut)) {
        ofLogOut << "AlignCoordina failure !!!" << std::endl;
        //failed
        lRtkData.pop_front();
        return -1;
    }
    if (error_sum > 0.5) {
        //std::cout << "error sum:" << error_sum << std::endl;
        CalAglDif_.second = CalAglDif_.first;
        return -1;
    } else {
        //std::cout << "error sum:" << error_sum << std::endl;
        CalAglDif_.first = CalAglDif_.second;
    }
    return 0;
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
        if (!has_odom_data && !has_rtk_data) {
            usleep(5000); //5ms
            continue;
        }
        if (has_odom_data) {
            OdometryData tmp_out = fusion20ms_with_rtk(lOdomData.back(), filtered_pos);
            mutex_output_pos_.lock();
            pub_pose_ = tmp_out;
            mutex_output_pos_.unlock();
        }
        if (rtk_buf.size() > 0) {
            rtk_odom_log_(2, "time1");
            auto itr_rtk = rtk_buf.begin();
            while (itr_rtk != rtk_buf.end()) {
                Eigen::Vector3d XYZRtkFrame = itr_rtk->xyz;
                double out_yaw1 = 0.0;
                if (EstimateYawInENU(XYZRtkFrame, out_yaw1)) {
                    itr_rtk->rpy[2] = out_yaw1;
                }
                lRtkData.push_back(*itr_rtk);
                itr_rtk++;
            }
            rtk_odom_log_(2, "time2");
        }
        while (lRtkData.size() && lRtkData.back().timestamp < lOdomData.front().time) { lRtkData.pop_front(); }
        while (lOdomData.size() && lOdomData.front().time < lOdomData.back().time - 6) { lOdomData.pop_front(); }

        if (lRtkData.size() == 0) {
            usleep(5000); //5ms
            continue;
        }
        if (1) {
            rtk_odom_log_(2, "time3");
            int ret_try = tryCalcTransOdomToRtk(svd_rtk_buf, svd_odom_buf);
            if (ret_try == 0) {
                getTransFlag_ = true;
            }
            rtk_odom_log_(2, "time4");
        }

        //got T, and new RTK, pls try output Pose
        auto itr_rtk = lRtkData.begin();
        rtk_odom_log_(1, "calculate rtk");
        while (itr_rtk != lRtkData.end() && lOdomData.size()) {
            if (itr_rtk->timestamp > lOdomData.back().time) {
                break;
            }
            rtk_odom_log_(1, "calculate rtk");
            GnssData cur_gnss_pos = *itr_rtk;
            //has aligned
            int ret_cal = calculate_rtk_with_odom(lOdomData, cur_gnss_pos);
            if (ret_cal == -2) { //wait for next odom
                cerr << fixed << setprecision(6) << " pls wait next odom " << cur_gnss_pos.timestamp << endl;
                break;
            } else if (ret_cal == 0) {
                updateOutputPos(filtered_pos);
            }
            itr_rtk++;
            lRtkData.pop_front();
        }              //end while rtk
        usleep(10000); //10ms
    }                  //end while
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
    bool IsKalmanFlag = false;
    int IsKalman = 0;
    double TmpYawRtK = -1000.0, TmpYawDR = -1000.0, TmpOut = -1000.0;
    OdometryData TmpOdomData;
    TmpOdomData.covariance[0] = 0.03;
    TmpOdomData.covariance[1] = 0.03;
    TmpOdomData.covariance[2] = 0.03;
    TmpOdomData.covariance[5] = 180 * kDeg2Rad; //默认航向误差为180度
    rtk_odom_log_(1, "calculate_rtk_with_odom");
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
    TmpOut = last_odom_pose_.rpy[2]; //默认为上次角度
    filtered_pos.status = 0;
    filtered_pos.time = rtk_pos.timestamp;
    TmpYawRtK = rtk_pos.rpy[2];
    TmpYawDR = restrict_angle_range(TmpOdomData.rpy[2] + CalAglDif_.second);
    //将RTK乘以旋转矩阵
    if (abs(TmpYawRtK) < M_PI) {
        TmpOdomData.covariance[5] = 3 * kDeg2Rad; //rtk角度误差3度
        if (getTransFlag_) {
            TmpOdomData.covariance[5] = abs(TmpYawDR - TmpYawRtK);
            std::cout
                << "DR anguler:" << TmpOdomData.rpy[2]
                << " ,RTK with DR dif is:" << CalAglDif_.second
                << " ,DR to Global:" << TmpYawDR
                << " ,TmpRTKYaw:" << TmpYawRtK
                << " ,Augler error:" << TmpOdomData.covariance[5] << std::endl;
            if (TmpOdomData.covariance[5] > 3 * kDeg2Rad) { //rtk误差在3度及延时,是否更新CalAglDif_的值
                filtered_pos.status = 1;                    //rtk anguler
                TmpOut = TmpYawRtK;
                // CalAglDif_.second = TmpYawRtK - TmpOdomData.rpy[2];
            } else {
                filtered_pos.status = 2; //DR anguler
                TmpOut = TmpYawDR;
            }
        } else {
            TmpOut = TmpYawRtK;
            filtered_pos.status = 1; //rtk角度
        }
    }
    last_odom_pose_ = TmpOdomData;
    last_rtk_data = rtk_pos;
    TmpOdomData.position = rtk_pos.xyz;
    TmpOdomData.rpy[2] = TmpOut;
    filtered_pos.gnss_data = rtk_pos;
    filtered_pos.ref_drOdom = TmpOdomData;
    return 0;
}
OdometryData RTKOdom::fusion20ms_with_rtk(
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
    OdometryData cur_predict_pos; // for pub, 50Hz;
    double delta_angule = tmp_dr_pose.rpy[2] - last_dr_pose_.rpy[2];
    if (align_rtk_dr.status == 1 && g_align_pose_update) { //rtk准确,更新直接更新,未更新直接递推
        if (dr_que_.size()) {
            while (dr_que_.size()) {
                if (dr_que_.front().time > align_rtk_dr.time) {
                    cur_predict_pos.rpy[2] = align_rtk_dr.gnss_data.rpy[2] + dr_que_.back().rpy[2] - dr_que_.front().rpy[2];
                    break;
                } else {
                    dr_que_.pop_front();
                }
            }
        } else
            cur_predict_pos.rpy[2] = align_rtk_dr.gnss_data.rpy[2]; //+ 0.9 * (last_predict_pose_.angle_xyz[2] + delta_angule);
        rtk_odom_log_(3, "rtk yaw is right:" + to_string(cur_predict_pos.rpy[2]));
    } else if (align_rtk_dr.status == 2) { //rtk与dr相差较小,CalAglDif_误差小,更新
        cur_predict_pos.rpy[2] = tmp_dr_pose.rpy[2] + CalAglDif_.second;
        rtk_odom_log_(3, "CalAglDif is right:" + to_string(cur_predict_pos.rpy[2]));
    } else {
        cur_predict_pos.rpy[2] = last_predict_pose_.rpy[2] + delta_angule; //预测
    }
    Eigen::Vector3d delta_pos = E3dEulerToMatrix(cur_predict_pos.rpy) * (Eigen::Vector3d((tmp_dr_pose.position - last_dr_pose_.position).norm(), 0.0, 0.0));
    if (g_align_pose_update) {
        cur_predict_pos.position = align_rtk_dr.gnss_data.xyz;
    } else {
        Eigen::Matrix3d J_prev_cur_imu = Eigen::Matrix3d::Identity();
        if (abs(last_predict_pose_.rpy[2] - cur_predict_pos.rpy[2]) > 0.00001) {
            Eigen::Matrix3d R_prev_cur_imu = E3dEulerToMatrix(last_predict_pose_.rpy).transpose() * E3dEulerToMatrix(cur_predict_pos.rpy); // Imu获取角度估计,计算变化值
            Eigen::AngleAxisd delta_se3(R_prev_cur_imu);
            double phi = delta_se3.angle();
            Eigen::Vector3d axis = delta_se3.axis();
            if (abs(phi) > 0.00001)
                J_prev_cur_imu = sin(phi) / phi * Eigen::Matrix3d::Identity() + (1 - sin(phi) / phi) * axis * axis.transpose() + ((1 - cos(phi)) / phi) * skew(axis); //视觉14讲，p73 4.26公式,右乘一个微小位移
        }
        delta_pos = J_prev_cur_imu * delta_pos;
        cur_predict_pos.position = last_predict_pose_.position + delta_pos;
    }
    if (g_align_pose_update) {
        Eigen::Vector3d update_data = {cur_predict_pos.position.x(), cur_predict_pos.position.y(), cur_predict_pos.rpy[2]};
        eskf_fusion_.update(update_data);
    } else {
        Eigen::Vector3d delta_pos12 = {delta_pos.x(), delta_pos.y(), delta_angule};
        eskf_fusion_.predict(delta_pos12);
    }
    if (0) {
        cur_predict_pos.position << eskf_fusion_.x.x(), eskf_fusion_.x.y(), 0.0;
        cur_predict_pos.rpy.z() = eskf_fusion_.x.z();
    }
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
    double len_dis = 0.15;
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
