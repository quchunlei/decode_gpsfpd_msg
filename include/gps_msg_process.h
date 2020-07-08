/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2019, Robosense Co.,Ltd. - www.robosense.ai
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

#ifndef RS_DECODE_GPSMSG_H
#define RS_DECODE_GPSMSG_H

#include <iostream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "prompt.h"
#include <tf/transform_datatypes.h>

namespace robosense
{
namespace decode
{
class GpsMsgProcess
{
public:
  explicit GpsMsgProcess();
  ~GpsMsgProcess();

  void setGpsOrigin(const Eigen::Vector3d& gps_origin_lon_lat_alt);
  bool gps2xyz(const Eigen::Vector3d& lon_lat_alt, Eigen::Vector3d& xyz);
  bool gps2xyz(const double& longitude, const double& latitude, const double& altitude, Eigen::Vector3d& xyz);
  bool xyz2gps(const Eigen::Vector3d& xyz, double& longitude, double& latitude, double& altitude);
  bool xyz2gps(const Eigen::Vector3d& xyz, Eigen::Vector3d& lon_lat_alt);
  Eigen::Vector3d getOffset(Eigen::Vector3d& long_lat_alt);

  const std::string name() const
  {
    return "GpsMsgProcess";
  }
  Eigen::Vector3d WGS84toECEF(const Eigen::Vector3d& gps);
  Eigen::Vector3d WGS84ToGCJ02(const Eigen::Vector3d& wgs84);
  Eigen::Vector3d WGS84ToBD09(const Eigen::Vector3d& wgs84);
  Eigen::Vector3d GCJ02ToWGS84(const Eigen::Vector3d& gcj02);
  Eigen::Vector3d GCJ02ToBD09(const Eigen::Vector3d& gcj02);
  Eigen::Vector3d ECEFtoWGS84(const Eigen::Vector3d& xyz);
  Eigen::Matrix3d getENURotation(const Eigen::Vector3d& lon_lat_alt);

  Eigen::VectorXd matrixToPose(const Eigen::Matrix4d& cur_tf);

private:
  double transformLat(double lng, double lat);
  double transformLng(double lng, double lat);

  inline bool outOfChina(Eigen::Vector3d& lon_lat_alt)
  {
    if (lon_lat_alt(0) < 72.004 || lon_lat_alt(0) > 137.8347)
    {
      return true;
    }
    if (lon_lat_alt(1) < 0.8293 || lon_lat_alt(1) > 55.8271)
    {
      return true;
    }
    return false;
  }

private:
  bool is_set_gps_origin_;
  Eigen::Vector3d gps_origin_;
  Eigen::Vector3d origin_ECEF_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace decode
}  // namespace robosense
#endif  // RS_decode_GPSMSG_H
