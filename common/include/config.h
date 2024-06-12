#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <stdio.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream> // std::ifstream
#include <iostream>
#include <vector>
#include "common.h"
using namespace std;
class Config {
public:
    Config() {
    }
    static void readConfig(std::string path);

public:
    // common
    static string imu_topic_;
    static string wheel_topic_;
    static string rtk_topic_;
    static Eigen::Matrix4d imu_T_wheel_;
    static double wheel_sx_;

    static YAML::Node config_node_;
};

#endif // SRC_CONFIG_H