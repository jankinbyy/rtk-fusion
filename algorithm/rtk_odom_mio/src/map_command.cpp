
#include "map_command.h"

#include <unistd.h>

#include <iomanip>
#include <iostream>

namespace rtk_odom_component {
double getCurrentTimeStamp_s() {
  auto now = std::chrono::high_resolution_clock::now();
  auto duration = now.time_since_epoch();
  double timestamp = std::chrono::duration<double>(duration).count();  // s
  return timestamp;
}
MapCommand::MapCommand(const std::string save_log_path,
                       const double end_map_dist) {
  save_log_path_ = save_log_path;
  end_map_dist_ = end_map_dist;
  ofLog_.open(save_log_path + "/map_command.txt");
  ofLog_ << std::fixed << std::setprecision(15);
  thd_judge_ptr_ = new std::thread(&MapCommand::judgeIsNearStartPoint, this);
  ofLog_ << "MapCommand start " << std::endl;
  ofLog_ << "end_map_dist_ = " << end_map_dist_ << std::endl;
}

MapCommand::~MapCommand() { thd_judge_ptr_->join(); }

void MapCommand::getCurrentPose(Eigen::Vector3d &pose) {
  std::unique_lock<std::mutex> lock(mutex_receive_new_pose_);
  cur_pose_ = pose;
}

void MapCommand::judgeIsNearStartPoint() {
  int ret = pthread_setname_np(pthread_self(), "slam_map_command");
  if (ret == 0) {
    std::cerr << " set judgeIsNearStartPoint thread name " << std::endl;
  } else {
    std::cerr << "failed set name to  judgeIsNearStartPoint " << std::endl;
  }
  bool b_leave_start_point = false;
  Eigen::Vector3d cur_pose = Eigen::Vector3d(0, 0, 0);
  while (true) {
    if (!b_start_mapping_) {
      usleep(10000);  // 10ms
      continue;
    }

    {
      std::unique_lock<std::mutex> lock(mutex_receive_new_pose_);
      cur_pose = cur_pose_;
    }

    double dist_move = (cur_pose[0] - start_map_point_[0]) *
                           (cur_pose[0] - start_map_point_[0]) +
                       (cur_pose[1] - start_map_point_[1]) *
                           (cur_pose[1] - start_map_point_[1]);

    dist_move = sqrt(dist_move);
    if (dist_move > end_map_dist_ && !b_leave_start_point) {
      b_leave_start_point = true;
      ofLog_ << "first leave start point: " << getCurrentTimeStamp_s() << " "
             << cur_pose << std::endl;
    }

    if (b_leave_start_point && !b_mapping_end_ && dist_move < end_map_dist_) {
      b_mapping_end_ = true;
      ofLog_ << "end mapping: " << getCurrentTimeStamp_s() << " " << cur_pose
             << std::endl;
    }

    usleep(10000);
  }
}

}  // namespace rtk_odom_component
