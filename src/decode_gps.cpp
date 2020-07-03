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

std::ofstream save_total_frame_txt_;
ros::Subscriber sub_gps_;
ros::Publisher pub_gps_trajectory_;
pcl::PointCloud<pcl::PointXYZI> gps_trajectory_;
std::shared_ptr<robosense::decode::GpsMsgProcess> gps_msg_process_ptr_;

void decodeGpsHandle(int sig)
{
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

  save_total_frame_txt_ << std::setprecision(20) << cur_gps(0) << "," << cur_gps(1) << "," << cur_gps(2) << std::endl;
  std::cerr << "current_frame: " << total_frame << "\r" << std::flush;
  total_frame++;
}

int main(int argc, char* argv[])
{
  signal(SIGINT, decodeGpsHandle);
  ros::init(argc, argv, "decode_rtk_node");
  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");

  std::string data_path;
  std::string gps_topic;
  pri_nh.param<std::string>("data_path", data_path, "/home/data/document_data/");
  pri_nh.param<std::string>("gps_topic", gps_topic, "/navsat/fix");
  pub_gps_trajectory_ = nh.advertise<sensor_msgs::PointCloud2>("/gps_trajectory", 10, true);
  std::string file_path = data_path + "gps.txt";
  save_total_frame_txt_.open(file_path);
  gps_msg_process_ptr_.reset(new robosense::decode::GpsMsgProcess());

  sub_gps_ = nh.subscribe(gps_topic, 100, gpsCallback);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}
