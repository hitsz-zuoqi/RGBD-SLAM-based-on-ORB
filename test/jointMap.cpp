// #include "camera.h"
// using namespace ZQ_SLAM;
// int main(int argc,char** argv)
// {
// //   ParameterReader pd;
//   FRAME frame1,frame2;
//   frame1.rgb = imread("../image/1.png");
//   frame1.depth = imread("../image/1_depth.png",-1);
//   frame2.rgb = imread("../image/2.png");
//   frame2.depth = imread("../image/2_depth.png",-1);

//   Camera::Ptr camera (new Camera());
//   // exam the factory function
//   cout<<camera->camera_.fx;
//   camera->computeKpAndDesp(frame1);
//   camera->computeKpAndDesp(frame2);
//   RESULT_PNP result = camera->estimate_RT(frame1,frame2,camera->camera_);

  
//   Mat R;
//   Rodrigues(result.rvec,R);
//   Eigen::Matrix3d r;
//   r<< R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
//       R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
//       R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
//   Vector3d t(result.tvec.at<double>(0,0),result.tvec.at<double>(0,1),result.tvec.at<double>(0,2));

//   Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//   T.rotate(r);
//   T.pretranslate(t);
//   cout<<"Transformation matrix:"<<T.matrix()<<endl;
//   PointCloud::Ptr cloud1 = camera->image2PointCloud(frame1.rgb,frame1.depth,camera->camera_);
//   PointCloud::Ptr cloud2 = camera->image2PointCloud(frame2.rgb,frame2.depth,camera->camera_);
//   cout<<"combining clouds ---------"<<endl;
//   PointCloud::Ptr rcloud1 (new PointCloud());
//   pcl::transformPointCloud(*cloud1,*rcloud1,T.matrix());
//   *rcloud1 += *cloud2;
//   pcl::io::savePCDFile("../resultOfConnection.pcd",*rcloud1);
//   cout<<"result has been saved----check it!"<<endl;
//   cv::waitKey(0);
//   return 0;
  



// }