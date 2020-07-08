/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2019, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author:  Robosense Localization Group
 * Version: 0.2.0
 * Date: 2019.10
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#ifndef DECODE_POSE_TRANSFORM_H
#define DECODE_POSE_TRANSFORM_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

namespace robosense
{
namespace decode
{
class Transform
{
public:
  Transform()
  {
  }
  ~Transform()
  {
  }
  Eigen::Matrix4d poseToMatrix(const double& pose_x, const double& pose_y, const double& pose_z,
                               const double& pose_roll, const double& pose_pitch, const double& pose_yaw)
  {
    /// generate transform matrix according to current pose
    Eigen::AngleAxisd current_rotation_x(pose_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd current_rotation_y(pose_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd current_rotation_z(pose_yaw, Eigen::Vector3d::UnitZ());

    Eigen::Translation3d current_translation(pose_x, pose_y, pose_z);
    Eigen::Matrix4d transform_matrix =
        (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();

    return transform_matrix;
  }

  Eigen::VectorXd matrixToPose(const Eigen::Matrix4d& cur_tf)
  {
    Eigen::VectorXd cur_pose(6);
    cur_pose << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    /// get current pose according to ndt transform matrix
    tf::Matrix3x3 mat_rotate;
    mat_rotate.setValue(cur_tf(0, 0), cur_tf(0, 1), cur_tf(0, 2), cur_tf(1, 0), cur_tf(1, 1), cur_tf(1, 2),
                        cur_tf(2, 0), cur_tf(2, 1), cur_tf(2, 2));

    cur_pose[0] = cur_tf(0, 3);
    cur_pose[1] = cur_tf(1, 3);
    cur_pose[2] = cur_tf(2, 3);
    mat_rotate.getRPY(cur_pose[3], cur_pose[4], cur_pose[5], 1);

    return cur_pose;
  }

  template <typename T>
  Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 1> eulerAnglesZYX(T&& q_in)
  {
    typedef typename std::remove_reference<T>::type::Scalar Scalar;

    Eigen::Matrix<Scalar, 4, 1> q = q_in.normalized().coeffs();

    Scalar s = -2 * (q(0) * q(2) - q(3) * q(1));
    if (s > 1)
    {
      s = 1;
    }
    return (Eigen::Matrix<Scalar, 3, 1>()
                << atan2f(2 * (q(0) * q(1) + q(3) * q(2)), q(3) * q(3) + q(0) * q(0) - q(1) * q(1) - q(2) * q(2)),
            asin(s), atan2(2 * (q(1) * q(2) + q(3) * q(0)), q(3) * q(3) - q(0) * q(0) - q(1) * q(1) + q(2) * q(2)))
        .finished();
  }
};

}  // namespace decode
}  // namespace robosense
#endif  // V2R_POSE_TRANSFORM_H