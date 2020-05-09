#include "camera.h"
using namespace ZQ_SLAM;

// enumerate the check_result
enum CHECK_RESULT {NOT_MATCHED=0,TOO_FAR_AWAY,TOO_CLOSE,KEYFRAME};

// declare the function about close-loop detect
CHECK_RESULT checkKeyframes(FRAME& f1,FRAME& f2,g2o::SparseOptimizer& opti, bool is_loops = false);

void checkNearbyLoops(vector<FRAME>& frames,FRAME& currFrame,g2o::SparseOptimizer& opti);

void checkRandomLoops(vector<FRAME>& frames,FRAME& currFrame,g2o::SparseOptimizer& opti);

// // for given index, read the frame
FRAME readFrame(int index,ParameterReader& pd);
// // measure the norm of the motion
double normOfTransform(cv::Mat rvec,cv::Mat tvec);

int main(int argc,char** argv)
{
  Camera::Ptr camera(new Camera());
  // cout<<"obtained camera intrin:"<<camera->camera_.fx<<endl; 

  // create a vector for keyframes
  std::vector<FRAME> keyframes;
  cout<<"Program initializing-----------"<<endl;
  for(int i=5;i>0;i--)
  {
    cout<<i<<endl;
    cv::waitKey(1000);
  }
  // initializing--------


  ParameterReader pd;
  int startIndex = atoi(pd.getData("start_index").c_str());
  int endIndex = atoi(pd.getData("end_index").c_str());
  //cout<<"start:"<<startIndex<<endl;
  int currentIndex = startIndex;
  FRAME currFrame = readFrame(currentIndex,pd);
  // imshow("rgb",lastFrame.rgb);
  // imshow("depth",lastFrame.depth);
  camera->computeKpAndDesp(currFrame);
  // PointCloud_Copy::Ptr cloud = camera->image2PointCloud(currFrame.rgb,currFrame.depth,camera->camera_);
  // pcl::visualization::CloudViewer viewer("viewer");
  

  /**
   * for g2o initialization
   */
  typedef g2o::BlockSolver_6_3 BS_6_3;
  //typedef g2o::LinearSolverEigen<BS_6_3::PoseMatrixType> SlamLinearSolver;
  std::unique_ptr<BS_6_3::LinearSolverType> linearSolver (new g2o::LinearSolverEigen<BS_6_3::PoseMatrixType>);
  std::unique_ptr<BS_6_3> BS (new BS_6_3(std::move(linearSolver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(BS));
  g2o::SparseOptimizer Optimizer;
  Optimizer.setAlgorithm(solver);
  // no debug information
  Optimizer.setVerbose(false);

  // add the first vertex and fix it 
  g2o::VertexSE3* v = new g2o::VertexSE3();
  v->setId(currentIndex);
  v->setFixed(true);
  v->setEstimate(Eigen::Isometry3d::Identity());
  Optimizer.addVertex(v);

  
  keyframes.push_back( currFrame );
  double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());
  bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");
  // start mapping ;end g2o initialization
  for(currentIndex = startIndex+1;currentIndex<endIndex;currentIndex++)
  {
    cout<<"Reading files:"<<currentIndex<<"th!---------------------------"<<endl;
    FRAME currFrame = readFrame(currentIndex,pd);
    // imshow("stream",currFrame.rgb);
    // cv::waitKey(5);
    camera->computeKpAndDesp(currFrame);
    // match currframe and lastFrame
    CHECK_RESULT result = checkKeyframes(keyframes.back(),currFrame,Optimizer);
    switch (result)
    {
    case NOT_MATCHED:
      cout<<"Not enough inliers."<<endl;
      break;
    case TOO_FAR_AWAY:
      cout<<"TOO FAR AWAY, MAY BE AN ERROR"<<endl;
      break;
    case TOO_CLOSE:
      cout<<"TOO CLOSE, NOT A KEYFRAME."<<endl;
      break;
      case KEYFRAME:
      cout<<"This a new KEYFRAME."<<endl;
      if(check_loop_closure)
      {
        checkNearbyLoops(keyframes,currFrame,Optimizer);
        // checkRandomLoops(keyframes,currFrame,Optimizer);
      }
      keyframes.push_back(currFrame);
      break;
    default:
      break;
    }
  }

  cout<<"optimizing pose graph, vertices: "<<Optimizer.vertices().size()<<endl;
  Optimizer.save("../result_before.g2o");
  Optimizer.initializeOptimization();
  Optimizer.optimize( 100 ); 
  Optimizer.save( "../result_after.g2o" );
  cout<<"Optimization done."<<endl;


  // connect the pointcloud map
  cout<<"saving the point cloud map..."<<endl;
  PointCloud_Copy::Ptr output(new PointCloud_Copy()); // the whole map
  PointCloud_Copy::Ptr tmp(new PointCloud_Copy());

  pcl::VoxelGrid<PoinT> voxel; // adjust the resolution of the map
  pcl::PassThrough<PoinT> pass; // z direction filters, remove the remote points since the contrained depth
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0,4.0);

  double gridsize = atof(pd.getData("voxel_grid").c_str());
  voxel.setLeafSize(gridsize,gridsize,gridsize);
  for(size_t i =0;i<keyframes.size();i++)
  {
    // get a frame from g2o
    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(Optimizer.vertex(keyframes[i].frameID));
    Eigen::Isometry3d pose = vertex->estimate();
    cout<<"IsoMetry:"<<pose.matrix()<<endl;
    PointCloud_Copy::Ptr newcloud = camera->image2PointCloud(keyframes[i].rgb,keyframes[i].depth,camera->camera_);
    // filters
    voxel.setInputCloud(newcloud);
    voxel.filter(*tmp);
    pass.setInputCloud(tmp);
    pass.filter(*newcloud);
    pcl::transformPointCloud(*newcloud,*tmp,pose.matrix());
    *output+=*tmp;
    tmp->clear();
    newcloud->clear();
  }
  voxel.setInputCloud(output);
  voxel.filter(*tmp);
  pcl::io::savePCDFile("../result.pcd",*tmp);
  cout<<"Final map is saved."<<endl;
  cout<<"The number of keyframes: "<<keyframes.size()<<endl;






  Optimizer.clear();
  return 0;
  
}

// define readFrame here
FRAME readFrame(int index,ParameterReader& pd)
{
  FRAME frame;
  string rgbDir = pd.getData("rgb_dir");
  string depthDir =pd.getData("depth_dir");
  string rgbExt = pd.getData("rgb_extension");
  string depthExt = pd.getData("depth_extension");
  // cout<<rgbDir<<endl;
  // cout<<depthDir<<endl;
  // cout<<rgbExt<<endl;
  // cout<<depthExt<<endl;
  stringstream ss;
  ss<<rgbDir<<index<<rgbExt;
  string filename;
  ss>>filename;
  // cout<<filename<<endl;
  frame.rgb = cv::imread(filename);
  // cout<<frame.rgb.cols<<endl;
  ss.clear();
  filename.clear();
  ss<<depthDir<<index<<depthExt;
  ss>>filename;
  frame.depth = cv::imread(filename,-1);
  frame.frameID = index;
  return frame;
}
double normOfTransform(cv::Mat rvec,cv::Mat tvec)
{
  return fabs(min(cv::norm(rvec),2*M_PI-cv::norm(rvec))+fabs(cv::norm(tvec)));
}

CHECK_RESULT checkKeyframes(FRAME& f1,FRAME& f2,g2o::SparseOptimizer& opti,bool is_loops)
{
  static ParameterReader pd;
  static int min_inliers = atoi(pd.getData("min_inliers").c_str());
  static double max_norm = stof(pd.getData("max_norm").c_str());
  static double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());
  static double max_norm_lp = atof(pd.getData("max_norm_lp").c_str());
  static Camera::Ptr camera(new Camera());
  static CAMERA_INTRINSIC camIntrin = camera->camera_;
  static g2o::RobustKernel* robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
  // compare f1 and f2
  RESULT_PNP result = camera->estimate_RT(f1,f2,camIntrin);
  cout<<"checkeyframe inliers:"<<result.inliers<<endl;
  // check the inliers 
  if(result.inliers <= min_inliers)
  {
    return NOT_MATCHED;
  }

  // check the motion
  double norm_motion = normOfTransform(result.rvec,result.tvec);
  if(is_loops==false)
  {
    if(norm_motion >= max_norm)
    {
      return TOO_FAR_AWAY;
    }
  }
  else 
  {
    if(norm_motion>=max_norm_lp)
    {
      return TOO_FAR_AWAY;
    }
  }

  if(norm_motion<=keyframe_threshold)
  {
    return TOO_CLOSE;
  }
  // if all satisfied, add a vertex and the connected edge with the last vertex
  if(is_loops==false)
  {
    g2o::VertexSE3* v = new g2o::VertexSE3();
    // add frameId
    v->setId(f2.frameID);
    v->setEstimate(Eigen::Isometry3d::Identity());
    opti.addVertex(v);
  }
  // for edge 
  g2o::EdgeSE3* edge = new g2o::EdgeSE3();
  // connect two points
  edge->vertices()[0] = opti.vertex(f1.frameID);
  edge->vertices()[1] = opti.vertex(f2.frameID);
  edge->setRobustKernel(robustKernel);
  // information matrix
  Eigen::Matrix<double,6,6> information = Eigen::Matrix<double,6,6>::Identity();
  information(0,0) = information(1,1) = information(2,2) = 100;
  information(3,3) = information(4,4) = information(5,5) = 100;
  edge->setInformation(information);
  Eigen::Isometry3d T = camera->RT2ISO(result.rvec,result.tvec);
  edge->setMeasurement(T.inverse());
  // add into graph
  opti.addEdge(edge);
  return KEYFRAME;
  }

  void checkNearbyLoops(vector<FRAME>& frames,FRAME& currFrame, g2o::SparseOptimizer& opti)
  {
    static ParameterReader pd;
    static int nearby_loops = atoi(pd.getData("nearby_loops").c_str());

    // measure 
    if(frames.size()<= nearby_loops)
    {
      // no enough frames at beginning, check all
      for(size_t i = 0; i<frames.size();i++)
      {
        checkKeyframes(frames[i],currFrame,opti,true);
      }
    }
    else
    {
      // check the nearest ones
      for(size_t i = frames.size() - nearby_loops;i<frames.size();i++)
      {
        checkKeyframes(frames[i],currFrame,opti,true);
      }
    }
    
  }

  void checkRandomLoops(vector<FRAME>& frames,FRAME& currFrame,g2o::SparseOptimizer& opti)
  {
    static ParameterReader pd;
    static int random_loops = atoi(pd.getData("random_loops").c_str());
    srand((unsigned int) time(NULL));

    if(frames.size()<=random_loops)
    {
      // no enough frames , check all
      for(size_t i=0;i<frames.size();i++)
      {
        checkKeyframes(frames[i],currFrame,opti,true);
      }
    }
    else 
    {
      // randomly select frames 
      for(int i =0 ; i<random_loops;i++)
      {
        int index = rand()%frames.size();
        checkKeyframes(frames[index],currFrame,opti,true);
      }
    }
  }