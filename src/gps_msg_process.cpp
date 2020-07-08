/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Localization Group
 * Version: 0.2.0
 * Date: 2019.10
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#include "gps_msg_process.h"

#define GPS_DUBG 0

namespace robosense
{
namespace decode
{
GpsMsgProcess::GpsMsgProcess()
{
  is_set_gps_origin_ = false;
  gps_origin_ << 0, 0, 0;
  origin_ECEF_ << 0, 0, 0;
}

GpsMsgProcess::~GpsMsgProcess()
{
  // std::cout<<"destructor GpsMsgProcess"<<std::endl;
}

void GpsMsgProcess::setGpsOrigin(const Eigen::Vector3d& gps_origin_lon_lat_alt)
{
  is_set_gps_origin_ = true;
  //  longitude, latitude, altitude
  gps_origin_ = gps_origin_lon_lat_alt;  // 116.3045885, 39.9680502,

  Eigen::Vector3d origin_WGS84(gps_origin_[0], gps_origin_[1], gps_origin_[2]);
  origin_ECEF_ = WGS84toECEF(origin_WGS84);
#if GPS_DUBG
  COUT << "gps_origin: " << gps_origin_.transpose() << END;
  COUT << "origin_ECEF: " << origin_ECEF_ << END;
#endif
}

Eigen::VectorXd GpsMsgProcess::matrixToPose(const Eigen::Matrix4d& cur_tf)
{
  Eigen::VectorXd cur_pose(6);
  cur_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  /// get current pose according to ndt transform matrix
  tf::Matrix3x3 mat_rotate;
  mat_rotate.setValue(cur_tf(0, 0), cur_tf(0, 1), cur_tf(0, 2), cur_tf(1, 0), cur_tf(1, 1), cur_tf(1, 2), cur_tf(2, 0),
                      cur_tf(2, 1), cur_tf(2, 2));

  cur_pose[0] = cur_tf(0, 3);
  cur_pose[1] = cur_tf(1, 3);
  cur_pose[2] = cur_tf(2, 3);
  mat_rotate.getRPY(cur_pose[3], cur_pose[4], cur_pose[5], 1);

  return cur_pose;
}

bool GpsMsgProcess::gps2xyz(const Eigen::Vector3d& lon_lat_alt, Eigen::Vector3d& xyz)
{
  return gps2xyz(lon_lat_alt[0], lon_lat_alt[1], lon_lat_alt[2], xyz);
}

bool GpsMsgProcess::gps2xyz(const double& longitude, const double& latitude, const double& altitude,
                            Eigen::Vector3d& xyz)
{
#if GPS_DUBG
  INFO << "gps2xyz longitude: " << longitude << " latitude: " << latitude << " altitude: " << altitude << REND;
#endif
  if (!is_set_gps_origin_)
  {
    ERROR << "[>> " << name() << "] Don't set gps origin!" << RESET << END;
    return false;
  }

  if (std::isnan(longitude) || std::isnan(latitude) || std::isnan(altitude))
  {
    ERROR << "[>> " << name() << "] GPS value is NAN!" << RESET << END;
    return false;
  }

  if (fabs(longitude) < 0.01 || fabs(latitude) < 0.01)
  {
    ERROR << "[>> " << name() << "] GPS value is smaller than 0.01!" << RESET << END;
    return false;
  }
  Eigen::Vector3d gps(longitude, latitude, altitude);
  Eigen::Vector3d gps_ECEF = WGS84toECEF(gps);

  //××××××××处理GPS数据
  double rad_lon = gps_origin_[0] / 180 * M_PI;
  double rad_lat = gps_origin_[1] / 180 * M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();

  // clang-format off
  rot << -sin_lon, cos_lon, 0,
        -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
        cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  // clang-format on

  Eigen::Vector3d diff_ECEF = gps_ECEF - origin_ECEF_;
  Eigen::Vector3d xyz_ECEF = rot * diff_ECEF;
  xyz << xyz_ECEF[0], xyz_ECEF[1], xyz_ECEF[2];
#if GPS_DUBG
  INFO << "diff_ECEF: " << diff_ECEF << REND;
  INFO << "xyz_ECEF: " << xyz_ECEF << REND;
  INFO << "xyz: " << xyz << REND;
#endif
  return true;
}

bool GpsMsgProcess::xyz2gps(const Eigen::Vector3d& xyz, Eigen::Vector3d& lon_lat_alt)
{
  return xyz2gps(xyz, lon_lat_alt[0], lon_lat_alt[1], lon_lat_alt[2]);
}

bool GpsMsgProcess::xyz2gps(const Eigen::Vector3d& xyz, double& longitude, double& latitude, double& altitude)
{
  if (!is_set_gps_origin_)
  {
    ERROR << "[>> " << name() << "] Don't set gps origin!" << RESET << END;
    return false;
  }

  if (std::isnan(xyz.x()) || std::isnan(xyz.x()) || std::isnan(xyz.x()))
  {
    ERROR << "[>> " << name() << "] XYZ value is NAN!" << RESET << END;
    return false;
  }

  // if (fabs(xyz.x()) < 0.01 || fabs(xyz.y()) < 0.01)
  // {
  //   ERROR << "[>> " << name() << "] XYZ value is smaller than 0.01!" << RESET << END;
  //   return false;
  // }

  double rad_lon = gps_origin_[0] / 180 * M_PI;
  double rad_lat = gps_origin_[1] / 180 * M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();

  // clang-format off
    rot << -sin_lon, cos_lon, 0,
           -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
            cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  // clang-format on

  Eigen::Vector3d diff_ECEF = rot.inverse() * xyz;
  Eigen::Vector3d gps_ECEF = diff_ECEF + origin_ECEF_;
  Eigen::Vector3d gps_WGS84 = ECEFtoWGS84(gps_ECEF);

#if GPS_DUBG
  INFO << "xyz2gps: " << gps_WGS84.transpose() << REND;
#endif

  longitude = gps_WGS84[0];
  latitude = gps_WGS84[1];
  altitude = gps_WGS84[2];
  return true;
}

Eigen::Matrix3d GpsMsgProcess::getENURotation(const Eigen::Vector3d& lon_lat_alt)
{
  double rad_lon = lon_lat_alt[0] / 180 * M_PI;
  double rad_lat = lon_lat_alt[1] / 180 * M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();

  // clang-format off
  rot << -sin_lon, cos_lon, 0,
          -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
          cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  // clang-format on
  return rot;
}

Eigen::Vector3d GpsMsgProcess::WGS84toECEF(const Eigen::Vector3d& gps)
{
  double SEMI_MAJOR_AXIS = 6378137.0;
  //    double RECIPROCAL_OF_FLATTENING = 298.257223563;
  //    double SEMI_MINOR_AXIS = 6356752.3142;
  double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;
  //    double SECOND_ECCENTRICITY_SQUARED = 6.73949674228e-3;

  double lon = gps[0];
  double lat = gps[1];
  double ati = 22.3820579024458;

  if (!std::isnan(gps[2]))
    ati = gps[2];

  double rad_lon = lon / 180 * M_PI;
  double rad_lat = lat / 180 * M_PI;

  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  double chi = sqrt(1.0 - FIRST_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
  double N = SEMI_MAJOR_AXIS / chi + ati;

  // clang-format off
  Eigen::Vector3d ret = Eigen::Vector3d::Zero();
  ret << N * cos_lat * cos_lon,
        N * cos_lat * sin_lon,
        (SEMI_MAJOR_AXIS * (1.0 - FIRST_ECCENTRICITY_SQUARED) / chi + ati) * sin_lat;
  // clang-format on
  return ret;
}

// refer from 'https://github.com/sameeptandon/sail-car-log/blob/master/process/WGS84toENU.py'
Eigen::Vector3d GpsMsgProcess::ECEFtoWGS84(const Eigen::Vector3d& xyz)
{
  double X = xyz[0];
  double Y = xyz[1];
  double Z = xyz[2];

  double a = 6378137.0;          // earth semimajor axis in meters
  double f = 1 / 298.257223563;  // reciprocal flattening
  double b = a * (1.0 - f);      // semi-minor axis

  double e2 = 2.0 * f - pow(f, 2.0);               // first eccentricity squared
  double ep2 = f * (2.0 - f) / pow((1.0 - f), 2);  // second eccentricity squared

  double r2 = pow(X, 2) + pow(Y, 2);
  double r = sqrt(r2);
  double E2 = pow(a, 2) - pow(b, 2);
  double F = 54 * pow(b, 2) * pow(Z, 2);
  double G = r2 + (1.0 - e2) * pow(Z, 2) - e2 * E2;
  double c = (e2 * e2 * F * r2) / (G * G * G);
  double s = pow((1.0 + c + sqrt(c * c + 2 * c)), 1.0 / 3.0);
  double P = F / (3.0 * pow((s + 1.0 / s + 1), 2) * G * G);
  double Q = sqrt(1 + 2 * e2 * e2 * P);
  double ro = -(e2 * P * r) / (1.0 + Q) +
              sqrt((a * a / 2.0) * (1 + 1.0 / Q) - ((1.0 - e2) * P * pow(Z, 2)) / (Q * (1.0 + Q)) - P * r2 / 2.0);
  double tmp = pow((r - e2 * ro), 2);
  double U = sqrt(tmp + pow(Z, 2));
  double V = sqrt(tmp + (1.0 - e2) * pow(Z, 2));
  double zo = (pow(b, 2) * Z) / (a * V);

  double h = U * (1.0 - pow(b, 2) / (a * V));
  double phi = atan((Z + ep2 * zo) / r);
  double lam = atan2(Y, X);
  // longitude, latitude, altitude
  return Eigen::Vector3d(180 / M_PI * lam, 180 / M_PI * phi, h);
}

Eigen::Vector3d GpsMsgProcess::WGS84ToBD09(const Eigen::Vector3d& wgs84)
{
  Eigen::Vector3d gcj02 = WGS84ToGCJ02(wgs84);
  return GCJ02ToBD09(gcj02);
}

Eigen::Vector3d GpsMsgProcess::WGS84ToGCJ02(const Eigen::Vector3d& wgs84)
{
  Eigen::Vector3d gcj02 = wgs84;
  if (outOfChina(gcj02))
  {
    return gcj02;
  }

  std::cout << "gcj02 bf:" << gcj02.transpose() << std::endl;
  getOffset(gcj02);
  std::cout << "gcj02 af:" << gcj02.transpose() << std::endl;
  return gcj02;
}

Eigen::Vector3d GpsMsgProcess::GCJ02ToWGS84(const Eigen::Vector3d& gcj02)
{
  Eigen::Vector3d wgs84 = gcj02;
  if (outOfChina(wgs84))
  {
    return wgs84;
  }

  std::cout << "wgs84 bf:" << wgs84.transpose() << std::endl;
  Eigen::Vector3d offset = getOffset(wgs84);
  wgs84(0) -= offset(0);
  wgs84(1) -= offset(1);

  std::cout << "wgs84 af:" << wgs84.transpose() << std::endl;
  return wgs84;
}

Eigen::Vector3d GpsMsgProcess::GCJ02ToBD09(const Eigen::Vector3d& gcj02)
{
  static double X_PI = 3.14159265358979324 * 3000.0 / 180.0D;

  double z = sqrt(gcj02(0) * gcj02(0) + gcj02(1) * gcj02(1)) + 0.00002 * sin(gcj02(1) * X_PI);
  double theta = atan2(gcj02(1), gcj02(0)) + 0.000003 * cos(gcj02(0) * X_PI);
  double bd_lng = z * cos(theta) + 0.0065;
  double bd_lat = z * sin(theta) + 0.006;

  return Eigen::Vector3d{ bd_lng, bd_lat, gcj02(2) };
}

// 经度偏移量
double GpsMsgProcess::transformLng(double lng, double lat)
{
  double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * sqrt(abs(lng));
  ret += (20.0 * sin(6.0 * lng * M_PI) + 20.0 * sin(2.0 * lng * M_PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(lng * M_PI) + 40.0 * sin(lng / 3.0 * M_PI)) * 2.0 / 3.0;
  ret += (150.0 * sin(lng / 12.0 * M_PI) + 300.0 * sin(lng / 30.0 * M_PI)) * 2.0 / 3.0;
  return ret;
}

// 纬度偏移量
double GpsMsgProcess::transformLat(double lng, double lat)
{
  double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * sqrt(abs(lng));
  ret += (20.0 * sin(6.0 * lng * M_PI) + 20.0 * sin(2.0 * lng * M_PI)) * 2.0 / 3.0;
  ret += (20.0 * sin(lat * M_PI) + 40.0 * sin(lat / 3.0 * M_PI)) * 2.0 / 3.0;
  ret += (160.0 * sin(lat / 12.0 * M_PI) + 320 * sin(lat * M_PI / 30.0)) * 2.0 / 3.0;
  return ret;
}

// 偏移量
Eigen::Vector3d GpsMsgProcess::getOffset(Eigen::Vector3d& long_lat_alt)
{
  double SEMI_MAJOR_AXIS = 6378245.0;
  double FIRST_ECCENTRICITY_SQUARED = 6.69342162296594323e-3;

  double dlng = transformLng(long_lat_alt(0) - 105.0, long_lat_alt(1) - 35.0);
  double dlat = transformLat(long_lat_alt(0) - 105.0, long_lat_alt(1) - 35.0);

  double radlat = long_lat_alt(1) / 180.0 * M_PI;
  double magic = sin(radlat);
  magic = 1 - FIRST_ECCENTRICITY_SQUARED * magic * magic;

  double sqrtmagic = sqrt(magic);
  dlng = (dlng * 180.0) / (SEMI_MAJOR_AXIS / sqrtmagic * cos(radlat) * M_PI);
  dlat = (dlat * 180.0) / ((SEMI_MAJOR_AXIS * (1 - FIRST_ECCENTRICITY_SQUARED)) / (magic * sqrtmagic) * M_PI);

  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  offset(0) += dlng;
  offset(1) += dlat;

  return offset;
}
}  // namespace decode
}  // namespace robosense
