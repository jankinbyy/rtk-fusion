// process map & command
// mapbe split later
#pragma once
#include "sensor_type.h"
#include <mutex>
#include <thread>
using namespace sensor_msgs_z;
namespace rtk_odom_component {
class MapCommand {
public:
    MapCommand(const std::string save_log_path, const double end_map_dist);
    ~MapCommand();

    void getCurrentPose(Eigen::Vector3d &pose);

    SLAMMODE slam_mode_ = IDLE;
    SLAMMODE last_slam_mode_ = IDLE;

    //mapping
    bool b_start_mapping_ = false;
    bool record_start_point_ = false;
    Eigen::Vector3d start_map_point_ = {0., 0., 0.};

    WorkArea work_area_;
    ContourInfo contour_info_;
    int created_area_num_ = -1;

    bool b_mapping_end_ = false;

private:
    void judgeIsNearStartPoint();
    std::thread *thd_judge_ptr_ = nullptr;

    std::mutex mutex_receive_new_pose_;
    Eigen::Vector3d cur_pose_;

    std::string save_log_path_;
    std::ofstream ofLog_;

    double end_map_dist_ = 0.; // m dist for judge when to end map
};

} // namespace rtk_odom_component
