#pragma once

#include "common_include.h"

namespace ZQ_SLAM
{
class Camera
{
public:
    typedef std::shared_ptr<Camera> Ptr;
    CAMERA_INTRINSIC  camera_;
    // factory function
    Camera();
    Camera(CAMERA_INTRINSIC camera):
    camera_(camera){}
    
    Point3f pixel2cam(const Point2f& p_p, const CAMERA_INTRINSIC& camera,const double depth);
    PointCloud_Copy::Ptr image2PointCloud(const Mat& rgb, const Mat& depth,const CAMERA_INTRINSIC& camera);
    // compute feature points and descriptors
    void computeKpAndDesp(FRAME& frame);
    // estimate Rotation and translation between two frame
    RESULT_PNP estimate_RT(const FRAME& frame1,const FRAME& frame2,const CAMERA_INTRINSIC& camera);
    // transform the rvec and tvec to Eigen::Isometry3d
    Eigen::Isometry3d RT2ISO(const cv::Mat& rvec,const cv::Mat& tvec);
    // add frame point cloud to previous point cloud
    PointCloud_Copy::Ptr JointCloud(const PointCloud_Copy::Ptr& preCloud,const PointCloud_Copy::Ptr& newCloud,const Eigen::Isometry3d& T);
};
class ParameterReader
{
public:
    ParameterReader(string filename="../config/parameter.txt")
    {
        ifstream fin(filename.c_str());
        if(!fin)
        {
            cerr<<"parameter file does not exist!"<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline(fin,str);
            if(str[0] == '#')
            {
                continue;
            }
            int pos =str.find("=");
            if(pos==-1)
            {
                continue;
            }
            string key =str.substr(0,pos);
            string value = str.substr(pos+1,str.length());
            data[key] = value;
            if(!fin.good())
            break;
        }
    }
    string getData(string key)
    {
        map<string,string>::iterator iter = data.find(key);
        if(iter == data.end())
        {
            cerr<<"Parameter name:"<<key<<"not found"<<endl;
            return string ("NOT FOUND");
        }
        return iter->second;
    }
public:
    map<string,string> data;
};


}