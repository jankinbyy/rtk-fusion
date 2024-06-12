#ifndef TRANS_RTK_ENU_H
#define TRANS_RTK_ENU_H

namespace rtk_odom_component { 

class LLAToENUConvertor {
public:
    LLAToENUConvertor() = default;
    ~LLAToENUConvertor() = default;

    int setECEFOrigin(double lon_rad, double lat_rad, double alt_meter);

    int convertLLA2ENU(double lon_degree, double lat_degree, double alt_meter,
                    double &p_Xe, double &p_Xn, double &p_Xu);

private:

    int convertLLA2ECEF(double p_Lon, double p_Lat, double p_Alt,
                    double &p_Xecef, double &p_Yecef, double &p_Zecef);

    int convertECEF2ENU(double x_ecef, double y_ecef, double z_ecef,
                        double &fE, double &fN, double &fU);
private:
    struct Earth_Param {
        const double e2 = 6.694379990097e-3;
        const double a = 6378137.0;
        const double f = 3.352810664725e-3;
    }WGS84_mio;

    /// origin
    bool m_bOriginSeted = false;
    double mfBaseX_ECEF, mfBaseY_ECEF, mfBaseZ_ECEF;
    double mBaseTransMatrix[3][3];
};

} // namespace  rtk_odom_component

#endif // TRANS_RTK_ENU_H
