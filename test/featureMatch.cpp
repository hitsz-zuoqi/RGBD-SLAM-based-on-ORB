// #include "camera.h"
// using namespace ZQ_SLAM;
// int main(int argc,char** argv)
// {
//   // load image files
//   Mat rgb1 = imread("../image/1.png");
//   Mat depth1 = imread("../image/1_depth.png",-1);

//   Mat rgb2 = imread("../image/2.png");
//   Mat depth2 = imread("../image/2_depth.png",-1);
//   // using orb feature detector to extract feature and do matches
//   cv::Ptr<cv::FeatureDetector> detector = ORB::create();
//   cv::Ptr<cv::DescriptorExtractor> extractor = ORB::create();
//   cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
//   vector<KeyPoint> kp1,kp2;
//   detector->detect(rgb1,kp1);
//   detector->detect(rgb2,kp2);
//   Mat desp1,desp2;
//   extractor->compute(rgb1,kp1,desp1);
//   extractor->compute(rgb2,kp2,desp2);
//   vector<DMatch> matches;
//   matcher->match(desp1,desp2,matches);

//   cout<<"Matches:"<<matches.size()<<endl;

//   // find the min dis and max dis
//   double min_dis=10000,max_dis=0;
//   for(int i=0;i<matches.size();i++)
//   {
//     double dis = matches[i].distance;
//     if(dis<min_dis) min_dis = dis;
//     if(dis>max_dis) max_dis = dis;
//   }
//   // filt out some wrong matches
//   vector<DMatch> nice_matches;
//   for(int i=0;i<matches.size();i++)
//   {
//     if(matches[i].distance<= max(2*min_dis,30.0) )
//     {
//       nice_matches.push_back(matches[i]);
//     }
//   }
//   cout<<"nice_matches:"<<nice_matches.size()<<endl;
//   // using the left nice_matches for PnP
//   vector<Point3f> pts_3d_1;
//   vector<Point2f> pts_2d_2;
  
//   // transform 2d points into 3d points using pixel2cam in class camera
//   Camera::Ptr camPtr (new Camera());
//   for(auto m:nice_matches)
//   {
//       Point2f uv_1 = kp1[m.queryIdx].pt;
//       Point2f uv_2 = kp2[m.trainIdx].pt;
//       ushort d = depth1.ptr<ushort> (int(uv_1.y))[(int(uv_1.x))];
//       // be careful to filter out those feature points with 0 depth
//       if(d==0) continue;
//       Point3f xyz_1 = camPtr->pixel2cam(uv_1,camPtr->camera_,double(d));
//       pts_3d_1.push_back(xyz_1);
//       pts_2d_2.push_back(uv_2);
//   }
//   cout<<"3d points: -- "<< pts_3d_1.size()<<endl;
//   // constrcut camera intrinsic matrix cause pnp needs it
//   Mat C = (Mat_<double>(3,3)<<camPtr->camera_.fx,0,camPtr->camera_.cx,0,camPtr->camera_.fy,camPtr->camera_.cy,0,0,1);
//   Mat rvec,tvec,inliers;
//   solvePnPRansac(pts_3d_1,pts_2d_2,C,cv::Mat(),rvec,tvec,false,100,1.0,0.99,inliers);
//   cout<<"R="<<rvec<<endl;
//   cout<<"t="<<tvec<<endl;
//   cout<<"inliers="<<inliers.rows<<endl;
//   cv::waitKey(0);
//   return 0;
  


// }

