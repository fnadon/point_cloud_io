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
#include <pcl/common/transforms.h>

#include <pcl_ros/transforms.hpp>

#ifdef HAVE_VTK
#include <pcl/io/vtk_lib_io.h>
#endif


namespace point_cloud_io {

bool pclToRclcpp(const pcl::PolygonMesh &pclmesh, shape_msgs::msg::Mesh::SharedPtr & msg, rclcpp::Logger logger)
{
  msg = std::make_shared<shape_msgs::msg::Mesh>();
  msg->vertices.resize(pclmesh.cloud.height*pclmesh.cloud.width);

  /*DEBUG std::cout << "height: " << pclmesh.cloud.height
  << ", width: " << pclmesh.cloud.width
  << ", point_step: " << pclmesh.cloud.point_step
  << ", row_step: " << pclmesh.cloud.row_step
  << ", is_dense: " << (int)pclmesh.cloud.is_dense << std::endl;*/
  size_t x_offset, y_offset, z_offset, stride = pclmesh.cloud.point_step;

  // Usually the fields have size 4 and contains "x", "y", "z", "rgb" from a ply file
  bool type_error = false;
  for(size_t i=0; i< pclmesh.cloud.fields.size(); i++)
  {
    /*DEBUG std::cout << "field: " << pclmesh.cloud.fields[i].name
    << ", offset: " << pclmesh.cloud.fields[i].offset
    << ", dtype: " << (pcl::PCLPointField::PointFieldTypes)pclmesh.cloud.fields[i].datatype
    << ", count: " << pclmesh.cloud.fields[i].count << std::endl;*/
    if(pclmesh.cloud.fields[i].name == "x")
    {
      x_offset = pclmesh.cloud.fields[i].offset;
      type_error |= pclmesh.cloud.fields[i].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
    if(pclmesh.cloud.fields[i].name == "y")
    {
      y_offset = pclmesh.cloud.fields[i].offset;
      type_error |= pclmesh.cloud.fields[i].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
    if(pclmesh.cloud.fields[i].name == "z")
    {
      z_offset = pclmesh.cloud.fields[i].offset;
      type_error |= pclmesh.cloud.fields[i].datatype != pcl::PCLPointField::PointFieldTypes::FLOAT32;
    }
  }
  if(type_error)
  {
    RCLCPP_ERROR(logger,"Only support FLOAT32 format.");
    return false;
  }
  /*std::cout << "xyz: " << x_offset << ", " << y_offset << ", " << z_offset << ", " << std::endl;*/
  for(size_t i=0; i<msg->vertices.size(); i++)
  {
    msg->vertices[i].x = static_cast<double>((*reinterpret_cast<const float*>(&pclmesh.cloud.data[stride*i + x_offset])));
    msg->vertices[i].y = static_cast<double>((*reinterpret_cast<const float*>(&pclmesh.cloud.data[stride*i + y_offset])));
    msg->vertices[i].z = static_cast<double>((*reinterpret_cast<const float*>(&pclmesh.cloud.data[stride*i + z_offset])));
  }

  msg->triangles.resize( pclmesh.polygons.size() );
  for(size_t i=0; i<msg->triangles.size(); i++)
  {
    if(pclmesh.polygons[i].vertices.size() != 3)
    {
      RCLCPP_ERROR(logger,"Detecting non-triangel primatives");
    }
    msg->triangles[i].vertex_indices[0] = pclmesh.polygons[i].vertices[0];
    msg->triangles[i].vertex_indices[1] = pclmesh.polygons[i].vertices[1];
    msg->triangles[i].vertex_indices[2] = pclmesh.polygons[i].vertices[2];
  }
  return true;
}

Read::Read() :
  rclcpp::Node("mesh_publisher"),
  pointCloudMessage_(new sensor_msgs::msg::PointCloud2())
{
  if (!readParameters()) {
    RCLCPP_WARN(this->get_logger(),"Shutting down due to missing parameters");
    rclcpp::shutdown();
  }

  pointCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    pointCloudTopic_, rclcpp::QoS(1));

  if(!meshTopic_.empty())
  {
    meshPublisher_ = this->create_publisher<shape_msgs::msg::Mesh>(
      meshTopic_, rclcpp::QoS(1));
  }
  initialize();
}

bool Read::readParameters() {

  this->declare_parameter("file_path", rclcpp::PARAMETER_STRING);
  this->declare_parameter("topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
  this->declare_parameter("rate", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("scale", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("mesh_topic", rclcpp::PARAMETER_STRING);
  this->declare_parameter("rpy_deg", ypr_);

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
    isContinuouslyPublishing_ = false; // TODO: this is broken in ROS2
  } else {
    isContinuouslyPublishing_ = true;
    updateDuration_ = std::chrono::microseconds((long int)(1000000.0 / updateRate) );
    RCLCPP_INFO(this->get_logger(), "Publishing mesh at %f hz", updateRate);
  }

  rclcpp::Parameter scale_param("scale", 1.0);
  this->get_parameter("scale", scale_param);
  scale_ = scale_param.as_double();

  this->get_parameter("mesh_topic", meshTopic_);

  this->get_parameter("rpy_deg", ypr_);

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
  RCLCPP_INFO(this->get_logger(),"Set scale to: %f", scale_);
  RCLCPP_INFO(this->get_logger(),"Set eular ypr to: [%f, %f, %f]", ypr_[0], ypr_[1], ypr_[2]);
  Eigen::Affine3d tf = Eigen::Affine3d::Identity();
  const double TO_RAD = EIGEN_PI/180.0;
  tf = Eigen::AngleAxisd(ypr_[0]*TO_RAD, Eigen::Vector3d::UnitX())
  * Eigen::AngleAxisd(ypr_[1]*TO_RAD, Eigen::Vector3d::UnitY())
  * Eigen::AngleAxisd(ypr_[2]*TO_RAD, Eigen::Vector3d::UnitZ());
  tf.scale(scale_);
  pcl_ros::transformPointCloud(tf.cast<float>().matrix(), *pointCloudMessage_, *pointCloudMessage_);

  RCLCPP_INFO_STREAM(this->get_logger(),"Loaded point cloud msg with " << pointCloudMessage_->height * pointCloudMessage_->width << " points.");

  if(!meshTopic_.empty())
  {
    pcl::PolygonMesh polygonMesh;
    if( pcl::io::loadPLYFile(filePath, polygonMesh) !=0) {
      return false;
    }
    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
    pcl::fromPCLPointCloud2(polygonMesh.cloud, pointCloud);
    pcl::transformPointCloud(pointCloud, pointCloud, tf);
    pcl::toPCLPointCloud2(pointCloud, polygonMesh.cloud);

    bool res = pclToRclcpp(polygonMesh, meshMessage_, this->get_logger());
    RCLCPP_INFO_STREAM(this->get_logger(),"Loaded mesh msg with " << meshMessage_->vertices.size() << " points.");
    if( !res ) {
      return false;
    }
  }
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
  if( meshPublisher_ && meshPublisher_->get_subscription_count() > 0u) {
    meshPublisher_->publish(*meshMessage_);
    RCLCPP_DEBUG_STREAM(this->get_logger(),"Mesh published to topic \"" << meshTopic_ << "\".");
  }
  return true;
}

}  // namespace point_cloud_io
