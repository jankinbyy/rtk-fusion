#include "trans_rtk_enu.h"
#include <cmath>
#include <iostream>
#include <iomanip>


namespace rtk_odom_component {

static double degreeToRad(double degree)
{
    return degree*M_PI/180.0;
}
int LLAToENUConvertor::setECEFOrigin(double lon_rad, double lat_rad, double alt_meter) {

    /// calculate transform-matrix
    double sinLat = std::sin(lat_rad);
    double cosLat = std::cos(lat_rad);
    double sinLon = std::sin(lon_rad);
    double cosLon = std::cos(lon_rad);
    mBaseTransMatrix[0][0] = -sinLon;
    mBaseTransMatrix[0][1] = cosLon;
    mBaseTransMatrix[0][2] = 0;
    mBaseTransMatrix[1][0] = -sinLat * cosLon;
    mBaseTransMatrix[1][1] = -sinLat * sinLon;
    mBaseTransMatrix[1][2] = cosLat;
    mBaseTransMatrix[2][0] = cosLat * cosLon;
    mBaseTransMatrix[2][1] = cosLat * sinLon;
    mBaseTransMatrix[2][2] = sinLat;

    /// set ecef origin-coordinates
    convertLLA2ECEF(lon_rad, lat_rad, alt_meter, mfBaseX_ECEF, mfBaseY_ECEF, mfBaseZ_ECEF);
    m_bOriginSeted = true;
    return 0;
}

int LLAToENUConvertor::convertLLA2ENU(double lon_degree, double lat_degree, double alt_meter,
                    double &fE, double &fN, double &fU)
{
    double lon_rad = degreeToRad(lon_degree);
    double lat_rad = degreeToRad(lat_degree);
    if (!m_bOriginSeted) {
        setECEFOrigin(lon_rad, lat_rad, alt_meter);
        fE = fN = fU = 0;
        return 0;
    }

    double x, y, z;
    convertLLA2ECEF(lon_rad, lat_rad, alt_meter, x, y, z);
    convertECEF2ENU(x, y, z, fE, fN, fU);
    return 1;
}

int LLAToENUConvertor::convertLLA2ECEF(double p_Lon, double p_Lat, double p_Alt,
                    double &p_Xecef, double &p_Yecef, double &p_Zecef) 
{
    double sinLat = std::sin((p_Lat));
    double cosLat = std::cos((p_Lat));
    double sinLon = std::sin((p_Lon));
    double cosLon = std::cos((p_Lon));
    double r = WGS84_mio.a / (1 - WGS84_mio.e2 * sinLat * sinLat);

    p_Xecef = (r + p_Alt) * cosLat * cosLon;
    p_Yecef = (r + p_Alt) * cosLat * sinLon;
    p_Zecef = (r * (1 - WGS84_mio.e2) + p_Alt) * sinLat;

    return 0;
}

int LLAToENUConvertor::convertECEF2ENU(double x_ecef, double y_ecef, double z_ecef,
                    double &fE, double &fN, double &fU)
{
    double dx = x_ecef - mfBaseX_ECEF;
    double dy = y_ecef - mfBaseY_ECEF;
    double dz = z_ecef - mfBaseZ_ECEF;

    fE = mBaseTransMatrix[0][0] * dx + mBaseTransMatrix[0][1] * dy + mBaseTransMatrix[0][2] * dz;
    fN = mBaseTransMatrix[1][0] * dx + mBaseTransMatrix[1][1] * dy + mBaseTransMatrix[1][2] * dz;
    fU = mBaseTransMatrix[2][0] * dx + mBaseTransMatrix[2][1] * dy + mBaseTransMatrix[2][2] * dz;

    return 0;
}
} // namespace rtk_odom_component
