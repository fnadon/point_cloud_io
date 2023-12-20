/*
 * Read.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "point_cloud_io/Read.hpp"

#include <filesystem>

// PCL
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef HAVE_VTK
#include <pcl/io/vtk_lib_io.h>
#endif

namespace point_cloud_io {

Read::Read() :
  rclcpp::Node("mesh_publisher"),
  pointCloudMessage_(new sensor_msgs::msg::PointCloud2())
{
  if (!readParameters()) {
    RCLCPP_WARN(this->get_logger(),"Shutting down due to missing parameters");
    rclcpp::shutdown();
  }

  pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pointCloudTopic_, rclcpp::QoS(1));
  initialize();
}

bool Read::readParameters() {
  this->declare_parameter("file_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
  this->declare_parameter("rate", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("scale", rclcpp::PARAMETER_DOUBLE);

  std::vector<rclcpp::Parameter> params;
  // throws exception if not found
  params = this->get_parameters({"file_path", "topic", "frame_id"});

  filePath_ = params[0].as_string();
  pointCloudTopic_ = params[1].as_string();
  pointCloudFrameId_ = params[2].as_string();

  rclcpp::Parameter rate_param("rate", 0.0);
  this->get_parameter("rate", rate_param);
  double updateRate = rate_param.as_double();
  RCLCPP_INFO(this->get_logger(), "Rate: %f", updateRate);
  if (updateRate <= 0.0) {
    isContinuouslyPublishing_ = false;
  } else {
    isContinuouslyPublishing_ = true;
    updateDuration_ = std::chrono::microseconds((long int)(1000000.0 / updateRate) );
    RCLCPP_INFO(this->get_logger(), "Publishing mesh at %f hz", updateRate);
  }

  rclcpp::Parameter scale_param("scale", 1.0);
  this->get_parameter("scale", scale_param);
  scale_ = scale_param.as_double();


  return true;
}

void Read::initialize() {
  if (!readFile(filePath_, pointCloudFrameId_)) {
    RCLCPP_WARN(this->get_logger(),"Shutting down2");
    rclcpp::shutdown();
  }

  if (isContinuouslyPublishing_) {
    timer_ = this->create_wall_timer(updateDuration_, std::bind(&Read::timerCallback, this));

  } else {
    // Need this to get things ready before publishing.
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    if (!publish()) {
      RCLCPP_ERROR(this->get_logger(),"Something went wrong when trying to read and publish the point cloud file.");
    }
    // This causes exception
    RCLCPP_WARN(this->get_logger(),"Single shot published. Shutting down.");
    rclcpp::shutdown();
  }
}

bool Read::readFile(const std::string& filePath, const std::string& pointCloudFrameId) {
  if (std::filesystem::path(filePath).extension() == ".ply") {
    // Load .ply file.
    pcl::PointCloud<pcl::PointXYZRGBNormal> pointCloud;
    if (pcl::io::loadPLYFile(filePath, pointCloud) != 0) {
      return false;
    }
    for( auto & pt : pointCloud.points)
    {
      pt.x *= scale_;
      pt.y *= scale_;
      pt.z *= scale_;
    }
    RCLCPP_INFO(this->get_logger(),"Set scale to: %f", scale_);


    // Define PointCloud2 message.
    pcl::toROSMsg(pointCloud, *pointCloudMessage_);
  }
#ifdef HAVE_VTK
  else if (std::filesystem::path(filePath).extension() == ".vtk") {
    // Load .vtk file.
    pcl::PolygonMesh polygonMesh;
    pcl::io::loadPolygonFileVTK(filePath, polygonMesh);

    // Define PointCloud2 message.
    pcl_conversions::moveFromPCL(polygonMesh.cloud, *pointCloudMessage_);
  }
#endif
  else {
    RCLCPP_ERROR_STREAM(this->get_logger(),"Data format not supported.");
    return false;
  }

  pointCloudMessage_->header.frame_id = pointCloudFrameId;

  RCLCPP_INFO_STREAM(this->get_logger(),"Loaded point cloud with " << pointCloudMessage_->height * pointCloudMessage_->width << " points.");
  return true;
}

void Read::timerCallback() {
  if (!publish()) {
    RCLCPP_ERROR(this->get_logger(),"Something went wrong when trying to read and publish the point cloud file.");
  }
}

bool Read::publish() {
  pointCloudMessage_->header.stamp = this->get_clock()->now();
  if (pointCloudPublisher_->get_subscription_count() > 0u) {
    pointCloudPublisher_->publish(*pointCloudMessage_);
    RCLCPP_DEBUG_STREAM(this->get_logger(),"Point cloud published to topic \"" << pointCloudTopic_ << "\".");
  }
  return true;
}

}  // namespace point_cloud_io
