/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************
 * @file decode_gps.cpp
 * @brief
 * @author CHUNLEI_QU
 * @version 1.0.0
 * @date 2019-07-02
 * @copyright RoboSense
 *****************************************************************************/

#include <ros/ros.h>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <signal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "decode_gps/gpfpd.h"
#include "gps_msg_process.h"
#include "pose_transform.h"

std::ofstream save_total_frame_txt_;
bool is_transform_gps_ = false;
double gps_yaw_origin_ = 0.0;
float trans_x_ = 0.0;
float trans_y_ = 0.0;
float trans_z_ = 0.0;

pcl::PointXYZI gps_point_origin_;
ros::Subscriber sub_gps_;
ros::Publisher pub_gps_trajectory_;
pcl::PointCloud<pcl::PointXYZI> gps_trajectory_;
std::shared_ptr<robosense::decode::GpsMsgProcess> gps_msg_process_ptr_;
std::shared_ptr<robosense::decode::Transform> transform_ptr_;

void decodeGpsHandle(int sig)
{
  if (std::fabs(gps_yaw_origin_) > 0.01)
  {
    double current_yaw = 0;
    pcl::PointXYZI last_gps_point;
    last_gps_point.x = 0.0;
    last_gps_point.y = 0.0;
    last_gps_point.z = 0.0;
    last_gps_point.intensity = 0;
    Eigen::Matrix4d transform_gps = Eigen::Matrix4d::Identity();
    Eigen::Vector3d cur_gps = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < gps_trajectory_.size(); ++i)
    {
      pcl::PointXYZI raw_point = gps_trajectory_.at(i);
      Eigen::Vector4d cur_gps_point{ raw_point.x, raw_point.y, raw_point.z, 1 };
      Eigen::Matrix4d transform_to_axis = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d rotation_to_axis = Eigen::Matrix4d::Identity();
      if (0 == i || (std::fabs(raw_point.x) + std::fabs(raw_point.y) < 0.5))
      {
        transform_to_axis = transform_ptr_->poseToMatrix(trans_x_, trans_y_, trans_z_, 0, 0, 0);
        rotation_to_axis = transform_ptr_->poseToMatrix(0, 0, 0, 0, 0, gps_yaw_origin_);
      }
      else
      {
        if (std::fabs(raw_point.y - last_gps_point.y) < 0.01 && std::fabs(raw_point.x - last_gps_point.x) < 0.01)
        {
          current_yaw = gps_yaw_origin_;
        }
        else
        {
          current_yaw = std::atan2(raw_point.y - last_gps_point.y, raw_point.x - last_gps_point.x);
          gps_yaw_origin_ = current_yaw;
        }
        std::cout << "current_yaw: " << current_yaw << std::endl;
        transform_to_axis = transform_ptr_->poseToMatrix(trans_x_, trans_y_, trans_z_, 0, 0, 0);
        rotation_to_axis = transform_ptr_->poseToMatrix(0, 0, 0, 0, 0, current_yaw);
      }
      Eigen::Matrix4d transform_matrix = rotation_to_axis * transform_to_axis;
      // std::cout << "transform_to_axis: " << transform_to_axis << std::endl;
      std::cout << "transform_matrix: " << transform_matrix << std::endl;
      transform_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
      std::cout << "transform_matrix: " << transform_matrix << std::endl;
      std::cout << "---------" << std::endl;
      Eigen::Vector4d calib_gps_point = transform_matrix * cur_gps_point;

      gps_msg_process_ptr_->xyz2gps(Eigen::Vector3d(calib_gps_point(0), calib_gps_point(1), calib_gps_point(2)),
                                    cur_gps);
      last_gps_point = raw_point;
      save_total_frame_txt_ << std::setprecision(20) << cur_gps(0) << "," << cur_gps(1) << "," << cur_gps(2)
                            << std::endl;
    }
  }
  transform_ptr_.reset();
  gps_msg_process_ptr_.reset();
  save_total_frame_txt_.close();
  sub_gps_.shutdown();
  pub_gps_trajectory_.shutdown();
  ros::shutdown();
}

void publishCloud(const ros::Publisher* in_publisher, pcl::PointCloud<pcl::PointXYZI>& in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header.frame_id = "map";
  in_publisher->publish(cloud_msg);
}

void gpsCallback(const decode_gps::gpfpd& gps_msg)
{
  static uint32_t total_frame = 0;
  static bool start_frame = true;
  if (std::isnan(gps_msg.Longitude) || std::isnan(gps_msg.Lattitude) || std::isnan(gps_msg.Altitude))
    return;

  Eigen::Vector3d cur_gps = Eigen::Vector3d::Zero();
  Eigen::Vector3d gps_xyz = Eigen::Vector3d::Zero();
  cur_gps << gps_msg.Longitude, gps_msg.Lattitude, gps_msg.Altitude;
  if (0 == total_frame)
  {
    gps_msg_process_ptr_->setGpsOrigin(cur_gps);
  }
  else
  {
    gps_msg_process_ptr_->gps2xyz(cur_gps, gps_xyz);
  }

  pcl::PointXYZI gps_point;
  gps_point.x = gps_xyz(0);
  gps_point.y = gps_xyz(1);
  gps_point.z = gps_xyz(2);
  gps_point.intensity = 0;

  gps_trajectory_.push_back(gps_point);
  publishCloud(&pub_gps_trajectory_, gps_trajectory_);
  if (is_transform_gps_)
  {
    double distance = 0.0;
    if (0 == total_frame)
    {
      gps_point_origin_ = gps_point;
    }
    else
    {
      distance = std::sqrt((gps_point.x - gps_point_origin_.x) * (gps_point.x - gps_point_origin_.x) +
                           (gps_point.y - gps_point_origin_.y) * (gps_point.y - gps_point_origin_.y));
    }

    if (distance > 1.0 && start_frame)
    {
      gps_yaw_origin_ = std::atan2(gps_point.y - gps_point_origin_.y, gps_point.x - gps_point_origin_.x);
      start_frame = false;
    }
    std::cout << "gps_yaw_origin: " << gps_yaw_origin_ << std::endl;
  }
  else
  {
    save_total_frame_txt_ << std::setprecision(20) << cur_gps(0) << "," << cur_gps(1) << "," << cur_gps(2) << std::endl;
  }

  std::cerr << "current_frame: " << total_frame << "\r" << std::flush;
  total_frame++;
}

int main(int argc, char* argv[])
{
  signal(SIGINT, decodeGpsHandle);
  ros::init(argc, argv, "decode_rtk_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");

  std::string data_path;
  std::string gps_topic;
  pri_nh.param<std::string>("data_path", data_path, "/home/data/document_data/");
  pri_nh.param<std::string>("gps_topic", gps_topic, "/navsat/fix");
  pri_nh.param<float>("trans_x", trans_x_, 0.0);
  pri_nh.param<float>("trans_y", trans_y_, 0.0);
  pri_nh.param<float>("trans_z", trans_z_, 0.0);

  if (std::abs(trans_x_) + std::abs(trans_y_) + std::abs(trans_z_) > 0.01)
  {
    is_transform_gps_ = true;
  }

  std::cout << "data_path: " << data_path << std::endl;
  pub_gps_trajectory_ = nh.advertise<sensor_msgs::PointCloud2>("/gps_trajectory", 10, true);
  std::string file_path = data_path + "gps.txt";
  save_total_frame_txt_.open(file_path);
  gps_msg_process_ptr_.reset(new robosense::decode::GpsMsgProcess());
  transform_ptr_.reset(new robosense::decode::Transform());

  sub_gps_ = nh.subscribe(gps_topic, 100, gpsCallback);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
