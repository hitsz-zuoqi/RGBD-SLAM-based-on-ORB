#pragma once

// for std 
#include "fstream"
#include "iostream"
#include "sstream"
#include "vector"
#include "algorithm"
#include "cmath"
#include "string"
#include "map"
#include "memory"
using namespace std;

// for pcl
#include "pcl/point_types.h"
// #include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/common/transforms.h"
#include "pcl/visualization/cloud_viewer.h"
// #include "pcl/visualization/pcl_visualizer.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
// typedef
typedef pcl::PointXYZRGBA PoinT;
typedef pcl::PointCloud<PoinT> PointCloud_Copy;

// for opencv
#include "opencv2/core/core.hpp"
// #include "opencv2/core/eigen.hpp"
// transform between opencv and rigen
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
using namespace cv;


// for eigen
#include "Eigen/Core"
#include "Eigen/Geometry"

using Eigen::Vector3d;
using Eigen::Vector2d;

// for g2o
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"



struct CAMERA_INTRINSIC 
{
  double cx,cy,fx,fy,scale;
};
// four element of a basic frame
struct FRAME
{
  cv::Mat rgb,depth;
  std::vector<cv::KeyPoint> kp;
  cv::Mat desp;
  int  frameID;
};

struct RESULT_PNP
{
  cv::Mat rvec,tvec;
  int inliers;
};

