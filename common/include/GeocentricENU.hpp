#pragma once
#include <iostream>
#include <cmath>
class LocalCartesianENU {
public:
    LocalCartesianENU() {
        ori_lat_ = 0.0;
        ori_lon_ = 0.0;
        ori_alt_ = 0.0;
    }
    void Reset(double in_lon, double in_lat, double in_alt) {
        double lat_rad = in_lat * M_PI / 180.0;
        double para_temp1 = 1.0 - squre_e;
        double para_temp2 = 1.0 - squre_e * sin(lat_rad) * sin(lat_rad);

        double RM = long_axis * para_temp1 / sqrt(para_temp2 * para_temp2 * para_temp2);
        double RN = long_axis / sqrt(para_temp2);
        a11 = RM * M_PI / 180.0;
        a22 = RN * cos(lat_rad) * M_PI / 180.0;
        yval = a11 * in_lat;
        xval = a22 * in_lon;
        ori_lat_ = in_lat;
        ori_lon_ = in_lon;
        ori_alt_ = in_alt;
        ori_init_ = true;
    }
    void Forward(double in_lon, double in_lat, double in_alt, double &local_E, double &local_N, double &local_U) {
        if (!ori_init_) {
            Reset(in_lon, in_lat, in_alt);
            local_E = 0.0;
            local_N = 0.0;
            local_U = 0.0;
        } else {
            local_E = a22 * in_lon - xval;
            local_N = a11 * in_lat - yval;
            local_U = in_alt - ori_alt_;
        }
    }

private:
    double ori_alt_;
    double ori_lat_;
    double ori_lon_;
    double a11 = 0.0;
    double a22 = 0.0;
    double xval = 0.0;
    double yval = 0.0;
    const double long_axis = 6378137.0;
    const double short_axis = 6356752.3142;
    const double squre_e = 0.006694380004261;
    bool ori_init_ = false;
};