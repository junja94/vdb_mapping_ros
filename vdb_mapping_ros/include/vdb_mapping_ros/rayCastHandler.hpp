
#ifndef VDB_RAYCASTER_HANDLER
#define VDB_RAYCASTER_HANDLER

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <vdb_mapping_msgs/GetMapSection.h>
#include <vdb_mapping_msgs/GetOccGrid.h>
#include <vdb_mapping_msgs/LoadMap.h>
#include <vdb_mapping_msgs/Raytrace.h>
#include <vdb_mapping_msgs/TriggerMapSectionUpdate.h>
#include <vdb_mapping_msgs/UpdateGrid.h>
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

template <typename VDBMappingT>
class RayCastHandler
{
public:
  RayCastHandler(const ros::NodeHandle &nh, VDBMappingROS<VDBMappingT> *mapper) : node_handle_(nh), vdb_mapper_(mapper)
  {

    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("ray_cast_check", 0);
  }

  ~RayCastHandler()
  {
  }

  void setMapper(VDBMappingROS<VDBMappingT> *mapper)
  {
    vdb_mapper_ = mapper;
  }

  // TODO: implement with the pointcloud version
  bool performRayCast(const Eigen::MatrixXd &ray_directions,
                      const double max_range,
                      std::string robot_frame_id = "base")
  {

    geometry_msgs::TransformStamped tf_b_to_m;
    Eigen::Matrix<double, 4, 4> tf_b_to_m_eigen;
    tf_b_to_m_eigen.setIdentity();

    try
    {
      tf_b_to_m = tf_buffer_.lookupTransform(map_frame_, robot_frame_id, ros::Time::now(), ros::Duration(0.05));
      tf_b_to_m_eigen= tf2::transformToEigen(tf_b_to_m).matrix();
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Transform to map frame failed:" << ex.what());
      return false;
    }

    Eigen::VectorXd ray_hits;
    ray_hits.setZero(ray_directions.rows() * 3);
    Eigen::Matrix<double, 4, 1> origin_in_map(0.0, 0.0, 0.0, 1.0);
    origin_in_map  = tf_b_to_m_eigen * origin_in_map;
    openvdb::Vec3d origin_vdb(origin_in_map[0], origin_in_map[1], origin_in_map[2]);

    for (int i = 0; i < ray_directions.rows(); i++){
      Eigen::Vector3d direction_in_map = tf_b_to_m_eigen.block(0,0,3,3) * ray_directions.row(i).transpose();
      openvdb::Vec3d direction_vdb(direction_in_map[0], direction_in_map[1], direction_in_map[2]); 
      openvdb::Vec3d end_point;

      bool ray_hit = vdb_mapper_->getMap().raytrace(origin_vdb,
                                                    direction_vdb,
                                                    max_range,
                                                    end_point);

      int data_index = i * 3;
      if (ray_hit){
        ray_hits.segment<3>(data_index) = Eigen::Vector3d(end_point[0], end_point[1], end_point[2]);
      }else{
        ray_hits.segment<3>(data_index) = origin_in_map.head(3) + max_range * direction_in_map;
      }
    }


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
    marker_debug.points.resize(ray_directions.rows());
    marker_debug.colors.resize(ray_directions.rows());

    for (int i = 0; i < ray_directions.rows(); i++){
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

  void test()
  {
    openvdb::Vec3d end_point;

    Eigen::Vector3d ray_direction(1.0, 1.0, 1.0);
    ray_direction.normalize();
    bool test_good = vdb_mapper_->getMap().raytrace(openvdb::Vec3d(0.0, 0.0, 0.0),
                                                    openvdb::Vec3d(ray_direction[0], ray_direction[1], ray_direction[2]),
                                                    10.0,
                                                    end_point);

    std::cout << test_good << " end_point: " << end_point << std::endl;

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
    marker_debug.points.resize(2);
    marker_debug.colors.resize(2);

    geometry_msgs::Point obj;
    obj.x = 0.0;
    obj.y = 0.0;
    obj.z = 0.0;
    marker_debug.points[0] = obj;
    marker_debug.colors[0].a = 1.0;
    marker_debug.colors[0].r = 1.0;
    marker_debug.colors[0].g = 1.0;
    marker_debug.colors[0].b = 1.0;

    geometry_msgs::Point obj2;
    obj2.x = end_point[0];
    obj2.y = end_point[1];
    obj2.z = end_point[2];
    marker_debug.points[1] = obj2;
    marker_debug.colors[1].a = 1.0;
    marker_debug.colors[1].r = 1.0;
    marker_debug.colors[1].g = 0.0;
    marker_debug.colors[1].b = 0.0;
    marker_publisher_.publish(marker_debug);
  }

private:
  ros::NodeHandle node_handle_;
  VDBMappingROS<VDBMappingT> *vdb_mapper_;
  ros::Publisher marker_publisher_;
  std::string map_frame_ = "map";
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

};

#endif /* VDB_RAYCASTER_HANDLER */
