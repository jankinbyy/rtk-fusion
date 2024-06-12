#include "config.h"
using namespace std;
YAML::Node Config::config_node_ = YAML::Node();
string Config::imu_topic_;
string Config::wheel_topic_;
string Config::rtk_topic_;
Eigen::Matrix4d Config::imu_T_wheel_;
double Config::wheel_sx_;

void Config::readConfig(std::string path) {
    config_node_ = YAML::LoadFile(path);
    YAML::Node topic_config_node_ = config_node_["topic"];
    imu_topic_ = topic_config_node_["imu_topic"].as<string>();
    std::cout << "imu topic:" << imu_topic_ << std::endl;
    wheel_topic_ = topic_config_node_["wheel_topic"].as<string>();
    std::cout << "wheel topic:" << wheel_topic_ << std::endl;
    rtk_topic_ = topic_config_node_["rtk_topic"].as<string>();
    std::cout << "rtk topic:" << rtk_topic_ << std::endl;
    YAML::Node read_config_node_ = config_node_["config"];
    imu_T_wheel_.setIdentity();
    wheel_sx_ = read_config_node_["wheel_sx"].as<double>();
    std::cout << "wheel_sx:" << wheel_sx_ << std::endl;
    for (int i = 0; i < 3; i++) imu_T_wheel_(i, 3) = read_config_node_["imu_t_wheel"][i].as<double>();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            imu_T_wheel_(i, j) = read_config_node_["imu_R_wheel"][3 * i + j].as<double>();
        }
    }
    std::cout << "read config over" << std::endl;
}