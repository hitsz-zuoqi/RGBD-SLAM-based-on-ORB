#include "camera.h"

namespace ZQ_SLAM
{
Camera::Camera()
{
    // static ParameterReader pr;
    // camera_.cx = atof(pr.getData("camera.cx").c_str());
    // camera_.cy = atof(pr.getData("camera.cy").c_str());
    // camera_.fx = atof(pr.getData("camera.fx").c_str());
    // camera_.fy = atof(pr.getData("camera.fy").c_str());
    // camera_.scale = atof(pr.getData("camera.scale").c_str());
    camera_.cx=325.5;
    camera_.cy=253.5;
    camera_.fx=518.0;
    camera_.fy=519.0;
    camera_.scale=1000.0;

}

Point3f Camera::pixel2cam(const Point2f& p_p,const CAMERA_INTRINSIC& camera,const double depth)
{
   Point3f p;
   p.z = depth/camera_.scale;
   p.x = (p_p.x - camera_.cx)*p.z/camera_.fx;
   p.y = (p_p.y - camera_.cy)*p.z/camera_.fy;
   return p;
}

PointCloud_Copy::Ptr Camera::image2PointCloud(const Mat& rgb,const Mat& depth,const CAMERA_INTRINSIC& camera)
{
  PointCloud_Copy::Ptr cloud(new PointCloud_Copy);
  
  for(int m=0;m<depth.rows;m++)
  {
    for(int n=0;n<depth.cols;n++)
    {
      ushort d = depth.ptr<ushort>(m)[n];

      if(d==0) continue;

      PoinT p;
      p.z = double(d)/camera_.scale;
      p.x = p.z*(n - camera_.cx)/camera_.fx;
      p.y = p.z*(m - camera_.cy)/camera_.fy;

      p.b = rgb.ptr<uchar>(m)[3*n];
      p.g = rgb.ptr<uchar>(m)[3*n+1];
      p.r = rgb.ptr<uchar>(m)[3*n+2];
      
      cloud->points.push_back(p);
    }
  }

  cloud->height = 1;
  cloud->width = cloud->points.size();
  cloud->is_dense = false;

  return cloud;

}

void Camera::computeKpAndDesp(FRAME& frame)
{
  cv::Mat rgb = frame.rgb;
  cv::Mat depth = frame.depth;
  vector<KeyPoint> kp;
  cv::Mat desp;
  cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
  cv::Ptr<cv::DescriptorExtractor> extractor = cv::ORB::create();
  detector->detect(rgb,kp);
  extractor->compute(rgb,kp,desp);
  frame.kp = kp;
  frame.desp = desp;
  return;
}

RESULT_PNP Camera::estimate_RT(const FRAME& frame1,const FRAME& frame2,const CAMERA_INTRINSIC& C)
{
   vector<DMatch> matches;
   RESULT_PNP result;
   cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
   matcher->match(frame1.desp,frame2.desp,matches);
   double min_dis = 10000;
   for(auto m:matches)
   {
     if(m.distance<min_dis) min_dis = m.distance;
   }
   if(min_dis<=10)
   {
     min_dis=10;
   }
   vector<DMatch> nice_matches;
   for(auto m:matches)
   {
    //  if(m.distance<= max(5*min_dis,50.0))
    //  {
    //    nice_matches.push_back(m);
    //  }
    if(m.distance<= 3*min_dis)
    {
      nice_matches.push_back(m);
    }
   }
   if(nice_matches.size()<=10)
   {
     result.inliers = -1;
     return result;
   }
   cout<<"nice matches:"<<nice_matches.size()<<endl;
   vector<Point3f> pts_3d_1;
   vector<Point2f> pts_2d_2;
   for(auto m:nice_matches)
   {
     Point2f uv_1 = frame1.kp[m.queryIdx].pt;
     ushort d = frame1.depth.ptr<ushort> ((int)(uv_1.y)) [(int)(uv_1.x)];
     if(d==0) continue;
     Point3f xyz_1 = Camera::pixel2cam(uv_1,C,d);
     Point2f uv_2 = frame2.kp[m.trainIdx].pt;
     pts_3d_1.push_back(xyz_1);
     pts_2d_2.push_back(uv_2);
   }
    if (pts_3d_1.size() <= 10 || pts_2d_2.size() <= 10)
    {
        result.inliers = -1;
        return result;
    }
   Mat cam_Intrin = (Mat_<double>(3,3)<<C.fx,0,C.cx,0,C.fy,C.cy,0,0,1);
   Mat rvec,tvec,inliers;
   cv::solvePnPRansac(pts_3d_1,pts_2d_2,cam_Intrin,Mat(),rvec,tvec,false,100,1.0,0.99,inliers);
   cout<<"find inliers:"<<inliers.size()<<endl;
   result.rvec = rvec;
   result.tvec = tvec;
   result.inliers = inliers.rows;
   return result;

}
Eigen::Isometry3d Camera::RT2ISO(const cv::Mat& rvec,const cv::Mat& tvec)
{
 cv::Mat R;
 Rodrigues(rvec,R);
 Eigen::Matrix3d r;
 r<<R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
    R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
    R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
 Eigen::Vector3d t(tvec.at<double>(0,0),tvec.at<double>(0,1),tvec.at<double>(0,2));
 Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
 T.rotate(r);
 T.pretranslate(t);
 return T;
}
PointCloud_Copy::Ptr Camera::JointCloud(const PointCloud_Copy::Ptr& preCloud,const PointCloud_Copy::Ptr& newCloud,const Eigen::Isometry3d& T)
{
  PointCloud_Copy::Ptr nowCloud(new PointCloud_Copy);
  pcl::transformPointCloud(*preCloud,*nowCloud,T.matrix());
  *nowCloud+=*newCloud;
  static pcl::VoxelGrid<PoinT> voxel;
  static ParameterReader pr;
  double grid_resolution = atof(pr.getData("voxel_grid").c_str());
  voxel.setLeafSize(grid_resolution,grid_resolution,grid_resolution);
  voxel.setInputCloud(nowCloud);
  PointCloud_Copy::Ptr tmp(new PointCloud_Copy);
  voxel.filter(*tmp);
  return tmp;
}
}