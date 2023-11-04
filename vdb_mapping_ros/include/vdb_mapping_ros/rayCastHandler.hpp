
#ifndef VDB_RAYCASTER_HANDLER
#define VDB_RAYCASTER_HANDLER

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <openvdb/io/Stream.h>
#include <vdb_mapping_ros/VDBMappingTools.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <vdb_mapping_ros/VDBMappingROSConfig.h>

// scan stuff
#include <vdb_mapping_ros/ray_patterns/VelodynPattern.hpp>
#include <vdb_mapping_msgs/UpdateScanParams.h>
#include <vdb_mapping_msgs/GetRayHits.h>

using namespace std::chrono;

template <typename VDBMappingT>
class RayCastHandler
{
public:
  RayCastHandler(const ros::NodeHandle &nh, VDBMappingROS<VDBMappingT> *mapper) : node_handle_(nh), vdb_mapper_(mapper)
  {

    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("ray_cast_check", 0);
    ray_hits_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("ray_hits", 0);
    velodyn_pattern_.reset(5.0, 5.0, 25.0, 10.0);
    velodyn_pattern_.getRayDirections(ray_directions_);

    trigger_update_scan_params_service_ =
        node_handle_.advertiseService("update_scan_params", &RayCastHandler::setScanParamsCallBack, this);
  
    trigger_get_ray_cast_service_ = 
        node_handle_.advertiseService("get_ray_cast", &RayCastHandler::rayCastCallBack, this);
      
  }

  ~RayCastHandler()
  {
    marker_publisher_.shutdown();
    ray_hits_publisher_.shutdown();
  }

  void setParams(const std::string& map_frame){
    map_frame_ = map_frame;
  }

  bool setScanParamsCallBack(vdb_mapping_msgs::UpdateScanParams::Request &req,
                             vdb_mapping_msgs::UpdateScanParams::Response &res)

  {
    velodyn_pattern_.reset(req.hor_resolution, req.ver_resolution, req.vertical_fov, 10.0);
    {
      std::lock_guard<std::mutex> lock(scan_pattern_mutex_);
      velodyn_pattern_.getRayDirections(ray_directions_);
    }

    ROS_INFO_STREAM(ray_directions_.rows() << " " << ray_directions_.cols() << " " << ray_directions_.size());

    res.num_points = (int) ray_directions_.size();
    res.success = true;
    return true;
  }

  bool rayCastCallBack(vdb_mapping_msgs::GetRayHits::Request &,
                      vdb_mapping_msgs::GetRayHits::Response &res){
    
    // performRayCast
    Eigen::VectorXf ray_hits;
    if (!performRayCast(10.0, ray_hits))
    {
      ROS_ERROR_STREAM("Ray casting failed");
      return false;
    }

    // fill result data
    std::vector<float> ray_hits_data;
    ray_hits_data.resize(ray_hits.size());
    Eigen::VectorXf::Map(&ray_hits_data[0], ray_hits.size()) = ray_hits;
    res.data=ray_hits_data;
    res.layout.dim.resize(1);
    res.layout.dim[0].label = "ray_hits";
    res.layout.dim[0].size = ray_hits.size();
    res.success = true;

    return true;
  }

    bool rayCastAndPublish(){
    Eigen::VectorXf ray_hits;
    if (!performRayCast(10.0, ray_hits))
    {
      ROS_ERROR_STREAM("Ray casting failed");
      return false;
    }

    // fill result data
    std::vector<float> ray_hits_data;
    ray_hits_data.resize(ray_hits.size());
    Eigen::VectorXf::Map(&ray_hits_data[0], ray_hits.size()) = ray_hits;

    std_msgs::Float32MultiArray ray_hits_msg;
    ray_hits_msg.data = ray_hits_data;
    ray_hits_msg.layout.dim.resize(1);
    ray_hits_msg.layout.dim[0].label = "ray_hits";
    ray_hits_msg.layout.dim[0].size = ray_hits.size();

    ray_hits_publisher_.publish(ray_hits_msg);
    return true;
  }

  void setMapper(VDBMappingROS<VDBMappingT> *mapper)
  {
    vdb_mapper_ = mapper;
  }

  // TODO: implement with the pointcloud version
  bool performRayCast(const double max_range,
                      Eigen::VectorXf &ray_hits_base_flatten)
  {

    Eigen::MatrixXd rays;
    int num_rays = 0;
    {
      std::lock_guard<std::mutex> lock(scan_pattern_mutex_);
      rays = ray_directions_;
      num_rays = ray_directions_.rows();
    }

    geometry_msgs::TransformStamped tf_b_to_m, tf_m_to_b;
    Eigen::Matrix<double, 4, 4> tf_b_to_m_eigen;
    Eigen::Matrix<double, 3, 3> rot_m_to_b;
    tf_b_to_m_eigen.setIdentity();
    rot_m_to_b.setIdentity();


    try
    {
      tf_b_to_m = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time::now(), ros::Duration(0.05));
      tf_b_to_m_eigen = tf2::transformToEigen(tf_b_to_m).matrix();

      rot_m_to_b = tf_b_to_m_eigen.block(0, 0, 3, 3).transpose();
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Transform to map frame failed:" << ex.what());
      return false;
    }

    Eigen::VectorXf ray_hits;
    ray_hits.setZero(num_rays * 3);

    // perform ray casting
    Eigen::Matrix<double, 4, 1> origin_in_map(0.0, 0.0, 0.0, 1.0);
    origin_in_map = tf_b_to_m_eigen * origin_in_map;
    openvdb::Vec3d origin_vdb(origin_in_map[0], origin_in_map[1], origin_in_map[2]);


    auto start = high_resolution_clock::now();

    ray_hits_base_flatten.setZero(num_rays * 3);
    for (int i = 0; i < num_rays; i++)
    {
      Eigen::Vector3d direction_in_map = tf_b_to_m_eigen.block(0, 0, 3, 3) * rays.row(i).transpose();
      openvdb::Vec3d direction_vdb(direction_in_map[0], direction_in_map[1], direction_in_map[2]);
      openvdb::Vec3d end_point;

      bool ray_hit = vdb_mapper_->getMap().raytrace(origin_vdb,
                                                    direction_vdb,
                                                    max_range,
                                                    end_point);

      int data_index = i * 3;
      if (ray_hit)
      {
        ray_hits.segment<3>(data_index) = Eigen::Vector3f(end_point[0], end_point[1], end_point[2]);
      }
      else
      {
        ray_hits.segment<3>(data_index) = (origin_in_map.head(3) + max_range * direction_in_map).cast<float>();
      }
      
      // transform into base frame
      Eigen::Vector3d ray_hit_base = rot_m_to_b * (ray_hits.segment<3>(data_index).cast<double>() - origin_in_map.head(3));
      ray_hits_base_flatten.segment<3>(data_index) = ray_hit_base.cast<float>();
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "raycast time" << duration.count() / 1000000.0 << std::endl;

    // TODO: move to other thread
    visualization_msgs::Marker marker_debug;
    marker_debug.header.frame_id = "map";
    marker_debug.header.stamp = ros::Time();
    marker_debug.ns = "ray_cast_handler";
    marker_debug.id = 0;
    marker_debug.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_debug.action = visualization_msgs::Marker::ADD;
    marker_debug.pose.orientation.x = 0.0;
    marker_debug.pose.orientation.y = 0.0;
    marker_debug.pose.orientation.z = 0.0;
    marker_debug.pose.orientation.w = 1.0;
    marker_debug.scale.x = 0.2;
    marker_debug.scale.y = 0.2;
    marker_debug.scale.z = 0.2;
    marker_debug.color.a = 1.0; // Don't forget to set the alpha!
    marker_debug.color.r = 0.0;
    marker_debug.color.g = 1.0;
    marker_debug.color.b = 0.0;
    marker_debug.points.resize(num_rays);
    marker_debug.colors.resize(num_rays);

    for (int i = 0; i < num_rays; i++){
      int data_index = i * 3;

      geometry_msgs::Point obj;
      obj.x = ray_hits.segment<3>(data_index)[0];
      obj.y =  ray_hits.segment<3>(data_index)[1];
      obj.z =  ray_hits.segment<3>(data_index)[2];
      marker_debug.points[i] = obj;
      marker_debug.colors[i].a = 1.0;
      marker_debug.colors[i].r = 0.0;
      marker_debug.colors[i].g = 0.0;
      marker_debug.colors[i].b = 1.0;
    }

    marker_publisher_.publish(marker_debug);

    return true;
  }

private:
  ros::NodeHandle node_handle_;
  VDBMappingROS<VDBMappingT> *vdb_mapper_;
  ros::Publisher marker_publisher_;
  ros::Publisher ray_hits_publisher_;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base";
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  VelodynPattern velodyn_pattern_;
  Eigen::MatrixXd ray_directions_;

  ros::ServiceServer trigger_update_scan_params_service_;
  ros::ServiceServer trigger_get_ray_cast_service_;

  std::mutex scan_pattern_mutex_;
};

#endif /* VDB_RAYCASTER_HANDLER */
