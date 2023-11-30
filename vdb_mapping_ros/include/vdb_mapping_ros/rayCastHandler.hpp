
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
#include <vdb_mapping_ros/ray_patterns/OmniPattern.hpp>
#include <vdb_mapping_msgs/UpdateScanParams.h>
#include <vdb_mapping_msgs/GetRayHits.h>
#include <vdb_mapping_msgs/GetTerrainHeight.h>

#include <thread>  

using namespace std::chrono;

template <typename VDBMappingT>
class RayCastHandler
{
public:
  RayCastHandler(const ros::NodeHandle &nh, VDBMappingROS<VDBMappingT> *mapper) : node_handle_(nh), vdb_mapper_(mapper)
  {

    terminateTFThread_ = false;
    tfUpdateThread_.reset(new std::thread(&RayCastHandler::TFUpdateThread, this));


    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("ray_cast_check", 0);
    marker_publisher2_ = node_handle_.advertise<visualization_msgs::Marker>("ray_cast_check2", 0);
    ray_hits_publisher_ = node_handle_.advertise<std_msgs::Float32MultiArray>("ray_hits", 0);
    scan_pattern_.reset(5.0, 5.0, 25.0, 40.0, 10.0);
    scan_pattern_.getRayDirections(ray_directions_);

    trigger_update_scan_params_service_ =
        node_handle_.advertiseService("update_scan_params", &RayCastHandler::setScanParamsCallBack, this);
  
    trigger_get_terrain_height_service_ = 
        node_handle_.advertiseService("get_terrain_height", &RayCastHandler::getTerrainHeightCallBack, this);

    trigger_get_ray_cast_service_ = 
        node_handle_.advertiseService("get_ray_cast", &RayCastHandler::rayCastCallBack, this); 
  }

  ~RayCastHandler()
  {
    marker_publisher_.shutdown();
    ray_hits_publisher_.shutdown();
    terminateTFThread_ = true;
    tfUpdateThread_->join();
  }

  void setParams(const std::string& map_frame){
    map_frame_ = map_frame;
  }

  bool setScanParamsCallBack(vdb_mapping_msgs::UpdateScanParams::Request &req,
                             vdb_mapping_msgs::UpdateScanParams::Response &res)

  {
    scan_pattern_.reset(req.hor_resolution, req.ver_resolution, req.vertical_fov_up, req.vertical_fov_down, 10.0);
    {
      std::lock_guard<std::mutex> lock(scan_pattern_mutex_);
      scan_pattern_.getRayDirections(ray_directions_);
    }

    ROS_INFO_STREAM(ray_directions_.rows() << " " << ray_directions_.cols() << " " << ray_directions_.size());

    res.num_points = (int) ray_directions_.rows();
    res.success = true;
    return true;
  }


  bool getTerrainHeightCallBack(vdb_mapping_msgs::GetTerrainHeight::Request &req,
                                vdb_mapping_msgs::GetTerrainHeight::Response &res)

  {
    double x = req.x; double y = req.y;
    

    Eigen::VectorXf ray_hits;
    Eigen::VectorXf ray_distances;
    Eigen::MatrixXd ray_directions;
    Eigen::VectorXd origin_in_base(4);
    origin_in_base << x, y, 0.0, 1.0;
    
    // grid around xy
    ray_directions.resize(9, 3);
    int ray_idx = 0;
    double offset_angle = 10.0;
    double offset_tan = std::tan(offset_angle * M_PI / 180.0);
    for (int i = -1; i < 2; i++){
      for (int j = -1; j < 2; j++){
        double x_offset = i * offset_tan;
        double y_offset = j * offset_tan;
        ray_directions.row(ray_idx) << x_offset,  y_offset, -1.0;
        ray_directions.row(ray_idx).normalize();
        ray_idx++;
      }
    }


    // wip
    if (!performRayCast(10.0, origin_in_base, ray_directions, ray_hits, ray_distances))
    {
      ROS_ERROR_STREAM("[getTerrainHeightCallBack] Ray casting failed");
      res.success = true;
      return false;
    } 

    double mean_z = 0.0;
    double num_hits = 0.0;
    for (int i = 0; i < ray_directions.rows(); i++){

      if (ray_distances[i] > 0.0 && ray_distances[i] < 5.0){ // filter out weird values
        double z_this_point = ray_hits[i*3 + 2];
        mean_z += z_this_point;
        num_hits += 1.0;
      }
    }
    if (num_hits > 0.0){
      mean_z /= num_hits;
    } else {
      mean_z = 0.0;
    }

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
    marker_debug.color.r = 1.0;
    marker_debug.color.g = 0.0;
    marker_debug.color.b = 0.0;
    marker_debug.points.resize(9);
    marker_debug.colors.resize(9);


    // for debugging
    Eigen::Matrix<double, 3, 3> rot_b_to_m;
    rot_b_to_m = tf_b_to_m_eigen_.block(0, 0, 3, 3);
    Eigen::Vector3d origin_in_map = (tf_b_to_m_eigen_ * origin_in_base).head(3);

    for (int i = 0; i < 9; i++){
      int data_index = i * 3;

      Eigen::Vector3d ray_hit_world = rot_b_to_m * ray_hits.segment<3>(data_index).cast<double>();
      ray_hit_world += origin_in_map;

      geometry_msgs::Point obj;
      obj.x = ray_hit_world[0];
      obj.y = ray_hit_world[1];
      obj.z = ray_hit_world[2];
      marker_debug.points[i] = obj;
      marker_debug.colors[i].a = 1.0;
      marker_debug.colors[i].r = 1.0;
      marker_debug.colors[i].g = 0.0;
      marker_debug.colors[i].b = 0.0;
    }

    marker_publisher2_.publish(marker_debug);


    res.data = mean_z; // in base frame
    res.success = true;
    return true;
  }

  bool rayCastCallBack(vdb_mapping_msgs::GetRayHits::Request &,
                      vdb_mapping_msgs::GetRayHits::Response &res){
    
    // performRayCast
    Eigen::VectorXf ray_hits;
    Eigen::VectorXf ray_distances;
    Eigen::MatrixXd ray_directions;
    Eigen::VectorXd origin_in_base(4);
    origin_in_base << 0.0, 0.0, 0.0, 1.0;
    {
      std::lock_guard<std::mutex> lock(scan_pattern_mutex_);
      ray_directions = ray_directions_;
    }

    if (!performRayCast(10.0, origin_in_base, ray_directions, ray_hits, ray_distances))
    {
      ROS_ERROR_STREAM("Ray casting failed");
      res.success = false;

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
    Eigen::VectorXf ray_distances;
    Eigen::MatrixXd ray_directions;
    Eigen::VectorXd origin_in_base(4);
    origin_in_base << 0.0, 0.0, 0.0, 1.0;
    {
      std::lock_guard<std::mutex> lock(scan_pattern_mutex_);
      ray_directions = ray_directions_;
    }
    if (!performRayCast(10.0, origin_in_base, ray_directions, ray_hits, ray_distances))
    {
      ROS_ERROR_STREAM("Ray casting failed");
      return false;
    }

    // fill result data
    std::vector<float> ray_distances_data;
    ray_distances_data.resize(ray_distances.size());
    Eigen::VectorXf::Map(&ray_distances_data[0], ray_distances.size()) = ray_distances;

    std_msgs::Float32MultiArray ray_msg;
    ray_msg.data = ray_distances_data;
    ray_msg.layout.dim.resize(1);
    ray_msg.layout.dim[0].label = "ray_data";
    ray_msg.layout.dim[0].size = ray_distances.size();

    ray_hits_publisher_.publish(ray_msg);
    return true;
  }

  void setMapper(VDBMappingROS<VDBMappingT> *mapper)
  {
    vdb_mapper_ = mapper;
  }


  void TFUpdateThread(){
    while(!terminateTFThread_){

      if (tf_buffer_.canTransform(map_frame_, base_frame_, ros::Time::now(), ros::Duration(0.05)))
      {
        tf_b_to_m_ = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.05));
        tf_b_to_m_eigen_ = tf2::transformToEigen(tf_b_to_m_).matrix();
        tf_updated_ = true;
      }
      else
      {
        ROS_WARN_STREAM("Transform to map frame failed");
      }

      using namespace std::chrono_literals;
      std::this_thread::sleep_for(200000us); // 5 Hz
    }
  }

  // TODO: implement with the pointcloud version
  bool performRayCast(const double max_range,
                      const Eigen::VectorXd &origin_in_base,
                      const Eigen::MatrixXd &ray_directions,
                      Eigen::VectorXf &ray_hits_base_flatten,
                      Eigen::VectorXf &ray_distances_flatten
                      )
  {

    int num_rays = ray_directions.rows();

    if (!tf_updated_){
      ROS_WARN_STREAM("Transform not updated");
      return false;
    }

    Eigen::Matrix<double, 3, 3> rot_m_to_b;
    rot_m_to_b = tf_b_to_m_eigen_.block(0, 0, 3, 3).transpose();

    Eigen::VectorXf ray_hits;
    ray_hits.setZero(num_rays * 3);

    // perform ray casting
    Eigen::VectorXd  origin_in_map = tf_b_to_m_eigen_ * origin_in_base;
    openvdb::Vec3d origin_vdb(origin_in_map[0], origin_in_map[1], origin_in_map[2]);


    // auto start = high_resolution_clock::now();
    ray_hits_base_flatten.setZero(num_rays * 3);
    ray_distances_flatten.setZero(num_rays);
    for (int i = 0; i < num_rays; i++)
    {
      Eigen::Vector3d direction_in_map = tf_b_to_m_eigen_.block(0, 0, 3, 3) * ray_directions.row(i).transpose();
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

        // transform into base frame
        Eigen::Vector3d ray_hit_base = rot_m_to_b * (ray_hits.segment<3>(data_index).cast<double>() - origin_in_map.head(3));
        ray_hits_base_flatten.segment<3>(data_index) = ray_hit_base.cast<float>();
        ray_distances_flatten[i] = ray_hit_base.norm();
      }
      else
      {
        // ray_hits.segment<3>(data_index) = (origin_in_map.head(3) + max_range * direction_in_map).cast<float>();
        ray_hits.segment<3>(data_index) = (origin_in_map.head(3) + 0.0 * direction_in_map).cast<float>();
        
        ray_hits_base_flatten.segment<3>(data_index).setZero(); 
        ray_distances_flatten[i] = 0.0;
      }
    }

    // auto stop = high_resolution_clock::now();
    // auto duration = duration_cast<microseconds>(stop - start);
    // std::cout << "raycast time" << duration.count() / 1000000.0 << std::endl;

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
  ros::Publisher marker_publisher2_;
  ros::Publisher ray_hits_publisher_;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base";
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  OmniPattern scan_pattern_;
  Eigen::MatrixXd ray_directions_;

  ros::ServiceServer trigger_update_scan_params_service_;
  ros::ServiceServer trigger_get_terrain_height_service_;
  ros::ServiceServer trigger_get_ray_cast_service_;

  // transform
  std::unique_ptr<std::thread> tfUpdateThread_;
  geometry_msgs::TransformStamped tf_b_to_m_;
  Eigen::Matrix<double, 4, 4> tf_b_to_m_eigen_;
  bool terminateTFThread_ = false;
  bool tf_updated_ = false;

  std::mutex scan_pattern_mutex_;
};

#endif /* VDB_RAYCASTER_HANDLER */
