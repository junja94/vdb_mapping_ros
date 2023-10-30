#pragma once

#include <Eigen/Core>
#include <functional>
#include <iostream>
#include <random>
#include <vector>


  class VelodynPattern
  {
  public:
    VelodynPattern() {}

    void reset(const double hor_resolution, 
              const double ver_resolution,
              const double vertical_fov, 
              const double max_range = 10.0)
    {
      maxRange_ = max_range;

      // Generate mesh grid
      int num_hor_points = std::ceil(360 / hor_resolution );
      int num_ver_points = std::ceil((vertical_fov + ver_resolution) / ver_resolution);

      std::vector<double> hor_points(num_hor_points);
      std::vector<double> ver_points(num_ver_points);
      for (int i = 0; i < num_hor_points; i++)
      {
        hor_points[i] = i * hor_resolution;
        hor_points[i] *= M_PI / 180.0; // deg to rad
      }

      for (int i = 0; i < num_ver_points; i++)
      {
        ver_points[i] = i * ver_resolution - 0.5 * vertical_fov;
        ver_points[i] *= M_PI / 180.0; // deg to rad
      }

      // meshgrid (xy indexing)
      int numPoints = num_hor_points * num_ver_points;
      rayAngles_ = Eigen::MatrixXd::Zero(numPoints, 2);
      rayDirections_ = Eigen::MatrixXd::Zero(numPoints, 3);

      for (int i = 0; i < num_ver_points; i++)
      {
        for (int j = 0; j < num_hor_points; j++)
        {
          int data_index = i * num_hor_points + j;
          rayAngles_(data_index, 0) = hor_points[j];
          rayAngles_(data_index, 1) = ver_points[i];

          rayDirections_(data_index, 0) = std::cos(hor_points[j]);
          rayDirections_(data_index, 1) = std::sin(hor_points[j]);
          rayDirections_(data_index, 2) = std::tan(ver_points[i]);
        }
      }

      // initialize buffers
      heightMeasurmentsVisualizer_ = Eigen::MatrixXd::Zero(numPoints, 4);
    }

    void getRayDirections(Eigen::MatrixXd &rayDirections)
    {
      rayDirections = rayDirections_;
    }

  protected:
    Eigen::MatrixX2d rayAngles_;
    Eigen::MatrixX3d rayDirections_;
    Eigen::MatrixX4d heightMeasurmentsVisualizer_;
    double maxRange_ = 10.0;
  };