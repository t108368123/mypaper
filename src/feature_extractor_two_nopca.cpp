#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/intersections.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <chrono> //system time
#include <pcl/visualization/pcl_visualizer.h>

#include "hough-3d-lines/hough3dlines.h"

#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
using namespace std;
#define DISPLAY_ALL false
#define POINT_TYPE pcl::PointXYZI
bool IS_CLOCKWISE = true;

ros::Publisher publisher_FullPcd;
ros::Publisher publisher_PassfilteredPcd;
ros::Publisher publisher_RansacPcd;
ros::Publisher publisher_ConvexHullPcd;
ros::Publisher publisher_marker_pub;
ros::Publisher publisher_testpca1;
ros::Publisher publisher_testpca2;
ros::Publisher publisher_testpca3;
image_transport::Publisher image_publisher;
image_transport::Publisher pcd2image_publisher;

struct initial_parameters_t
{
  string image_filename, pcd_filename;
  cv::Size chessboard_pattern_size;
  int square_length;                // in millimetres
  cv::Size board_dimensions;        // in millimetres
  cv::Point3d cb_translation_error; // in millimetres
  cv::Mat cameramat, distcoeff;
  vector<float> counterclockwise_icp_init_rotmat;
  vector<float> counterclockwise_icp_init_tranmat;
  vector<float> counterclockwise_icp_rotmat;
  vector<float> counterclockwise_icp_tranmat;
  vector<float> clockwise_icp_init_rotmat;
  vector<float> clockwise_icp_init_tranmat;
  vector<float> clockwise_icp_rotmat;
  vector<float> clockwise_icp_tranmat;
  float camera_size_width,camera_size_height;  //相機解析度
  float x_min_ccw,x_max_ccw,y_min_ccw,y_max_ccw,z_min_ccw,z_max_ccw;  //逆時針的閥值
  float x_min_cw,x_max_cw,y_min_cw,y_max_cw,z_min_cw,z_max_cw;  //順時針的閥值
  vector<int> ccw_r_2d; //逆時針重投影誤差 2d順序
  vector<int> ccw_r_3d; //逆時針重投影誤差 3d順序
  vector<int> cw_r_2d; //順時針重投影誤差 2d順序
  vector<int> cw_r_3d; //順時針重投影誤差 3d順序
};

initial_parameters_t i_params;

void get_params(ros::NodeHandle nh_)
{
  vector<float> cameramat;
  vector<float> distcoeff;
  nh_.param<vector<float>>("mypaper/cameramat", cameramat, vector<float>());
  nh_.param<vector<float>>("mypaper/distcoeff", distcoeff, vector<float>());
  nh_.param<float>("mypaper/camera_size/width", i_params.camera_size_width, 640);
  nh_.param<float>("mypaper/camera_size/height", i_params.camera_size_height, 480);

  i_params.cameramat = cv::Mat::zeros(3, 3, CV_64F);
  i_params.cameramat.at<double>(0, 0) = cameramat[0];
  i_params.cameramat.at<double>(0, 2) = cameramat[2];
  i_params.cameramat.at<double>(1, 1) = cameramat[4];
  i_params.cameramat.at<double>(1, 2) = cameramat[5];
  i_params.cameramat.at<double>(2, 2) = cameramat[8];
  i_params.distcoeff = cv::Mat::eye(1, 4, CV_64F);
  i_params.distcoeff.at<double>(0) = distcoeff[0];
  i_params.distcoeff.at<double>(1) = distcoeff[1];
  i_params.distcoeff.at<double>(2) = distcoeff[2];
  i_params.distcoeff.at<double>(3) = distcoeff[3];

  nh_.param<string>("mypaper/image_filename", i_params.image_filename, "");
  nh_.param<string>("mypaper/pcd_filename", i_params.pcd_filename, "");

  nh_.param<int>("mypaper/chessboard_pattern_size/width", i_params.chessboard_pattern_size.width, 5);
  nh_.param<int>("mypaper/chessboard_pattern_size/height", i_params.chessboard_pattern_size.height, 7);
  nh_.param<int>("mypaper/board_dimensions/width", i_params.board_dimensions.width, 590);
  nh_.param<int>("mypaper/board_dimensions/height", i_params.board_dimensions.height, 840);
  nh_.param<int>("mypaper/square_length", i_params.square_length, 95);
  nh_.param<double>("mypaper/cb_translation_error/x", i_params.cb_translation_error.x, 0.0);
  nh_.param<double>("mypaper/cb_translation_error/y", i_params.cb_translation_error.y, 0.0);

  nh_.param<vector<float>>("mypaper/counterclockwise/icp_init_rotmat", i_params.counterclockwise_icp_init_rotmat, vector<float>());
  nh_.param<vector<float>>("mypaper/counterclockwise/icp_init_tranmat", i_params.counterclockwise_icp_init_tranmat, vector<float>());
  nh_.param<vector<float>>("mypaper/counterclockwise/icp_rotmat", i_params.counterclockwise_icp_rotmat, vector<float>());
  nh_.param<vector<float>>("mypaper/counterclockwise/icp_tranmat", i_params.counterclockwise_icp_tranmat, vector<float>());
  nh_.param<vector<float>>("mypaper/clockwise/icp_init_rotmat", i_params.clockwise_icp_init_rotmat, vector<float>());
  nh_.param<vector<float>>("mypaper/clockwise/icp_init_tranmat", i_params.clockwise_icp_init_tranmat, vector<float>());
  nh_.param<vector<float>>("mypaper/clockwise/icp_rotmat", i_params.clockwise_icp_rotmat, vector<float>());
  nh_.param<vector<float>>("mypaper/clockwise/icp_tranmat", i_params.clockwise_icp_tranmat, vector<float>());

  nh_.param<float>("mypaper/counterclockwise/x_min", i_params.x_min_ccw, 0.0);
  nh_.param<float>("mypaper/counterclockwise/x_max", i_params.x_max_ccw, 0.0);
  nh_.param<float>("mypaper/counterclockwise/y_min", i_params.y_min_ccw, 0.0);
  nh_.param<float>("mypaper/counterclockwise/y_max", i_params.y_max_ccw, 0.0);
  nh_.param<float>("mypaper/counterclockwise/z_min", i_params.z_min_ccw, 0.0);
  nh_.param<float>("mypaper/counterclockwise/z_max", i_params.z_max_ccw, 0.0);
  nh_.param<float>("mypaper/clockwise/x_min", i_params.x_min_cw, 0.0);
  nh_.param<float>("mypaper/clockwise/x_max", i_params.x_max_cw, 0.0);
  nh_.param<float>("mypaper/clockwise/y_min", i_params.y_min_cw, 0.0);
  nh_.param<float>("mypaper/clockwise/y_max", i_params.y_max_cw, 0.0);
  nh_.param<float>("mypaper/clockwise/z_min", i_params.z_min_cw, 0.0);
  nh_.param<float>("mypaper/clockwise/z_max", i_params.z_max_cw, 0.0);

  nh_.param<vector<int>>("mypaper/counterclockwise/r_2d", i_params.ccw_r_2d, vector<int>());
  nh_.param<vector<int>>("mypaper/counterclockwise/r_3d", i_params.ccw_r_3d, vector<int>());
  nh_.param<vector<int>>("mypaper/clockwise/r_2d", i_params.cw_r_2d, vector<int>());
  nh_.param<vector<int>>("mypaper/clockwise/r_3d", i_params.cw_r_3d, vector<int>());
}

void onMouse(int evt, int x, int y, int flags, void* param) {
  if(evt == CV_EVENT_LBUTTONDOWN) {
    cv::Point* ptPtr = (cv::Point*)param;
    ptPtr->x = x;
    ptPtr->y = y;
  }
}

void image_click()
{
  cv::Point2i pt(-1,-1);//assume initial point
  cv::namedWindow("Output Window");
  cv::Mat frame = cv::imread(i_params.image_filename, cv::IMREAD_COLOR);

  cv::setMouseCallback("Output Window", onMouse, (void*)&pt);
  int X=0, Y=0, i=0;
  int temp_X=0, temp_Y=0;
  std::vector<cv::Point2d> click_point;

  while(1)
  {
    cv::imshow("Output Window", frame);
    X=pt.x; 
    Y=pt.y; 
    cout<<"X and Y coordinates are given below"<<endl;
    cout<<X<<'\t'<<Y<<endl; 
    if(cv::waitKey(10)==27)
      break;

    if(temp_X!=X || temp_Y!=Y)
    {
      temp_X = X;
      temp_Y = Y;
      i++;
      click_point.push_back(cv::Point2d(X,Y));
    }
  }

  cout << "click_point:" << click_point << endl;
}

bool read_pcd(pcl::PointCloud<POINT_TYPE>::Ptr &cloud)
{
  if (pcl::io::loadPCDFile<POINT_TYPE>(i_params.pcd_filename, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file pcd\n");
    return 0;
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  publisher_FullPcd.publish(cloud_msg);

  return 1;
}

void passthrough(const pcl::PointCloud<POINT_TYPE>::ConstPtr &input_pc, pcl::PointCloud<POINT_TYPE>::Ptr &output_pc)
{
  pcl::PointCloud<POINT_TYPE>::Ptr x(new pcl::PointCloud<POINT_TYPE>);
  pcl::PointCloud<POINT_TYPE>::Ptr z(new pcl::PointCloud<POINT_TYPE>);
  // Filter out the experimental region
  pcl::PassThrough<POINT_TYPE> pass;
  if(IS_CLOCKWISE)
  {
    pass.setInputCloud(input_pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(i_params.x_min_cw, i_params.x_max_cw);
    pass.filter(*x);
    pass.setInputCloud(x);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(i_params.z_min_cw, i_params.z_max_cw);
    pass.filter(*z);
    pass.setInputCloud(z);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(i_params.y_min_cw, i_params.y_max_cw);
    pass.filter(*output_pc);
  }
  else
  {
    pass.setInputCloud(input_pc);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(i_params.x_min_ccw, i_params.x_max_ccw);
    pass.filter(*x);
    pass.setInputCloud(x);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(i_params.z_min_ccw, i_params.z_max_ccw);
    pass.filter(*z);
    pass.setInputCloud(z);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(i_params.y_min_ccw, i_params.y_max_ccw);
    pass.filter(*output_pc);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*output_pc, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  publisher_PassfilteredPcd.publish(cloud_msg);
}

void ransac(const pcl::PointCloud<POINT_TYPE>::ConstPtr &input_pc, pcl::PointCloud<POINT_TYPE>::Ptr &output_pc)
{
  // Fit a plane through the board point cloud
  // Inliers give the indices of the points that are within the RANSAC threshold
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<POINT_TYPE> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.004);
  pcl::ExtractIndices<POINT_TYPE> extract;
  seg.setInputCloud(input_pc);
  seg.segment(*inliers, *coefficients);

  // Check that segmentation succeeded
  if (coefficients->values.size() < 3)
  {
    ROS_WARN("Chessboard plane segmentation failed");
  }

  // Project the inliers on the fitted plane
  // When it freezes the chessboard after capture, what you see are the inlier points (filtered from the original)
  pcl::ProjectInliers<POINT_TYPE> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(input_pc);
  proj.setModelCoefficients(coefficients);
  proj.filter(*output_pc);

  //only use ransac,and would not project the inliers on the fitted plane
  /*pcl::PointCloud<POINT_TYPE> cloud_ransac;
  POINT_TYPE p;
  for (const auto& idx: inliers->indices)
  {
    p.x = input_pc->points[idx].x;
    p.y = input_pc->points[idx].y;
    p.z = input_pc->points[idx].z;
    p.intensity = input_pc->points[idx].intensity;
    cloud_ransac.push_back(p);
  }
  output_pc = cloud_ransac.makeShared();*/

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*output_pc, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  publisher_RansacPcd.publish(cloud_msg);
}

void bound(const pcl::PointCloud<POINT_TYPE>::ConstPtr &cloud, pcl::PointCloud<POINT_TYPE>::Ptr &boundPoints)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<POINT_TYPE, pcl::Normal, pcl::Boundary> est;
  pcl::search::KdTree<POINT_TYPE>::Ptr tree(new pcl::search::KdTree<POINT_TYPE>());

  pcl::NormalEstimation<POINT_TYPE, pcl::Normal> normEst;  //其中POINT_TYPE表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
  normEst.setInputCloud(cloud);
  normEst.setSearchMethod(tree);
  // normEst.setRadiusSearch(2);  //法向估计的半径
  normEst.setKSearch(500);  //法向估计的点数
  normEst.compute(*normals);

  //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
  est.setInputCloud(cloud);
  est.setInputNormals(normals);
  //  est.setAngleThreshold(90);
  //   est.setSearchMethod (pcl::search::KdTree<POINT_TYPE>::Ptr (new pcl::search::KdTree<POINT_TYPE>));
  est.setSearchMethod(tree);
  est.setKSearch(500);  //一般这里的数值越高，最终边界识别的精度越好
  //est.setRadiusSearch(0.1);  //搜索半径
  est.compute(boundaries);

  pcl::PointCloud<POINT_TYPE> noBoundPoints;
  int countBoundaries = 0;
  for (int i = 0; i < cloud->size(); i++) {
    uint8_t x = (boundaries.points[i].boundary_point);
    int a = static_cast<int>(x); //该函数的功能是强制类型转换
    if (a == 1)
    {
      //  boundPoints.push_back(cloud->points[i]);
      (*boundPoints).push_back(cloud->points[i]);
      countBoundaries++;
    }
    else
      noBoundPoints.push_back(cloud->points[i]);
  }

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*boundPoints, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  publisher_ConvexHullPcd.publish(cloud_msg);
}

void save_point(const pcl::PointCloud<POINT_TYPE>::ConstPtr &input_pc)
{
  pcl::PointCloud<POINT_TYPE> cloud;
  cloud = *input_pc;

  ofstream ofs;

  ofs.open("/home/a123/Documents/catkin_ws/src/mypaper/src/output.dat");
  if (!ofs.is_open())
  {
    cout << "Failed to open file.\n";
  }
  else
  {
    for (pcl::PointCloud<POINT_TYPE>::const_iterator item = cloud.begin(); item != cloud.end(); item++)
    {
      ofs << (double)item->x << "," << (double)item->y << "," << (double)item->z << "\n";
    }
    ofs.close();
  }
}

void calculate_sidelength(visualization_msgs::Marker points)
{
  float length=0;
  for(int i=0;i<4;i++)
  {
    for(int j=i+1;j<4;j++)
    {
      length = sqrt(pow((points.points[i].x-points.points[j].x), 2)+pow((points.points[i].y-points.points[j].y), 2)+pow((points.points[i].z-points.points[j].z), 2));
      cout << i << "," << j << "_length:" << length*100 << endl;
    }
  }
}

std::vector<cv::Point3d> find_crossline(double line_arr[4][6])
{
  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = "/world";
  points.header.stamp = line_strip.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = "find_crossline";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  points.id = 0;
  line_strip.id = 1;
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  points.scale.x = 0.1;
  points.scale.y = 0.1;
  line_strip.scale.x = 0.1;
  points.color.g = 1.0f;
  points.color.a = 1.0;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p;

  std::vector<cv::Point3d> cross_point;
  float lidar_centre_x=0, lidar_centre_y=0, lidar_centre_z=0;
  Eigen::Vector4f corner;
  Eigen::VectorXf top_left(6);
  Eigen::VectorXf top_right(6);
  for (int i = 0; i < 4; i++)
  {
    for (int j = i + 1; j < 4; j++)
    {
      top_left << line_arr[i][0], line_arr[i][1], line_arr[i][2], line_arr[i][3], line_arr[i][4], line_arr[i][5];
      top_right << line_arr[j][0], line_arr[j][1], line_arr[j][2], line_arr[j][3], line_arr[j][4], line_arr[j][5];
      if (pcl::lineWithLineIntersection(top_left, top_right, corner))
      {
        if(IS_CLOCKWISE)
        {
          if(corner[2] < i_params.z_min_cw-1 || corner[2] > i_params.z_max_cw+1)
            continue;
        }
        else
        {
          if(corner[2] < i_params.z_min_ccw-1 || corner[2] > i_params.z_max_ccw+1)
            continue;
        }

        cout << "crossline:" << endl;
        cout << corner[0]*1000 << " " << corner[1]*1000 << " " << corner[2]*1000 << endl;
        lidar_centre_x += (float)corner[0] / 4;
        lidar_centre_y += (float)corner[1] / 4;
        lidar_centre_z += (float)corner[2] / 4;
        cross_point.push_back(cv::Point3d(corner[0],corner[1],corner[2]));

        p.x = corner[0];
        p.y = corner[1];
        p.z = corner[2];
        points.points.push_back(p);
        line_strip.points.push_back(p);
      }
    }
  }
  calculate_sidelength(points);
  cross_point.push_back(cv::Point3d(lidar_centre_x, lidar_centre_y, lidar_centre_z));
  cout << "center:" << endl << lidar_centre_x*1000 << " " << lidar_centre_y*1000 << " " << lidar_centre_z*1000 << endl;
  p.x = lidar_centre_x;
  p.y = lidar_centre_y;
  p.z = lidar_centre_z;
  points.points.push_back(p);
  line_strip.points.push_back(p);

  publisher_marker_pub.publish(points);
  //publisher_marker_pub.publish(line_strip);

  return cross_point;
}

auto chessboardProjection(const std::vector<cv::Point2d> &corners, cv::Mat image_color)
{
  // Find the chessboard in 3D space - in it's own object frame (position is arbitrary, so we place it flat)

  // Location of board frame origin from the bottom left inner corner of the chessboard
  cv::Point3d chessboard_bleft_corner((i_params.chessboard_pattern_size.width - 1) * i_params.square_length / 2, (i_params.chessboard_pattern_size.height - 1) * i_params.square_length / 2, 0);

  std::vector<cv::Point3d> corners_3d;
  for (int x = 0; x < i_params.chessboard_pattern_size.width; x++)
  {
    for (int y = 0; y < i_params.chessboard_pattern_size.height; y++)
    {
      corners_3d.push_back(cv::Point3d(x, y, 0) * i_params.square_length - chessboard_bleft_corner);
    }
  }

  // chessboard corners, middle square corners, board corners and centre
  std::vector<cv::Point3d> board_corners_3d;
  // Board corner coordinates from the centre of the chessboard

  board_corners_3d.push_back(cv::Point3d(-(i_params.board_dimensions.width + i_params.cb_translation_error.x) / 2.0,
                                         (i_params.board_dimensions.height - i_params.cb_translation_error.y) / 2.0, 0.0));

  board_corners_3d.push_back(cv::Point3d(-(i_params.board_dimensions.width + i_params.cb_translation_error.x) / 2.0,
                                         -(i_params.board_dimensions.height + i_params.cb_translation_error.y) / 2.0, 0.0));

  board_corners_3d.push_back(cv::Point3d((i_params.board_dimensions.width - i_params.cb_translation_error.x) / 2.0,
                                         (i_params.board_dimensions.height - i_params.cb_translation_error.y) / 2.0, 0.0));

  board_corners_3d.push_back(cv::Point3d((i_params.board_dimensions.width - i_params.cb_translation_error.x) / 2.0,
                                         -(i_params.board_dimensions.height + i_params.cb_translation_error.y) / 2.0, 0.0));
  // Board centre coordinates from the centre of the chessboard (due to incorrect placement of chessboard on board)
  board_corners_3d.push_back(cv::Point3d(-i_params.cb_translation_error.x / 2.0, -i_params.cb_translation_error.y / 2.0, 0.0));

  std::vector<cv::Point2d> inner_cbcorner_pixels, board_image_pixels;
  cv::Mat rvec(3, 3, cv::DataType<double>::type); // Initialization for pinhole and fisheye cameras
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  bool valid_camera_info = true;
  if (valid_camera_info)
  {
    cout << "3d:" << board_corners_3d << endl;
    cv::solvePnP(corners_3d, corners, i_params.cameramat, i_params.distcoeff, rvec, tvec, 0, 0);                
    cv::projectPoints(corners_3d, rvec, tvec, i_params.cameramat, i_params.distcoeff, inner_cbcorner_pixels);
    cv::projectPoints(board_corners_3d, rvec, tvec, i_params.cameramat, i_params.distcoeff, board_image_pixels);
  }
  else
  {
    ROS_FATAL("No msgs from /camera_info - check camera_info topic in cfg/params.yaml is correct and is being published");
  }

  cout << "board_image_pixels" << board_image_pixels << endl;
  for (int i = 0; i < board_image_pixels.size(); i++)
  {
    if (i == 0)
    {
      cv::circle(image_color, board_image_pixels[i], 5, CV_RGB(255, 0, 0), -1);
    }
    else if (i == 1)
    {
      cv::circle(image_color, board_image_pixels[i], 5, CV_RGB(0, 255, 0), -1);
    }
    else if (i == 2)
    {
      cv::circle(image_color, board_image_pixels[i], 5, CV_RGB(0, 0, 255), -1);
    }
    else if (i == 3)
    {
      cv::circle(image_color, board_image_pixels[i], 5, CV_RGB(255, 255, 0), -1);
    }
    else if (i == 4)
    {
      cv::circle(image_color, board_image_pixels[i], 5, CV_RGB(0, 255, 255), -1);
    }
  }

  for (auto &point : inner_cbcorner_pixels)
  {
    cv::circle(image_color, point, 5, CV_RGB(255, 0, 0), -1);
  }

  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_color).toImageMsg();
  image_publisher.publish(img_msg); //cv::Mat

  // Return all the necessary coefficients
  return std::make_tuple(rvec, tvec, board_image_pixels);
}

auto findImgChessboardCorners()
{
  cv::Mat image_color, image_gray;
  image_color = cv::imread(i_params.image_filename, cv::IMREAD_COLOR);
  cv::cvtColor(image_color, image_gray, CV_BGR2GRAY);

  if(IS_CLOCKWISE)
    cv::rectangle(image_gray,cvPoint(0,0),cvPoint(i_params.camera_size_width/2,i_params.camera_size_height),cv::Scalar(0,0,0),-1,1,0);
  else
    cv::rectangle(image_gray,cvPoint(i_params.camera_size_width/2,0),cvPoint(i_params.camera_size_width,i_params.camera_size_height),cv::Scalar(0,0,0),-1,1,0);

  std::vector<cv::Point2f> cornersf;
  std::vector<cv::Point2d> corners;
  // Find chessboard pattern in the image
  bool pattern_found = findChessboardCorners(image_gray, cv::Size(i_params.chessboard_pattern_size.height, i_params.chessboard_pattern_size.width), cornersf, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

  //指定亚像素计算迭代标注
  cv::TermCriteria criteria = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);
  //亚像素检测
  cv::cornerSubPix(image_gray, cornersf, cv::Size(11, 11), cv::Size(-1, -1), criteria);
  for (auto &corner : cornersf)
  {
    corners.push_back(cv::Point2d(corner));
    //cv::circle(image_color, corner, 3, CV_RGB(255, 0, 0), -1);
  }
  /*cv::imshow("chessboard corners", image_color);
  cv::waitKey(0);*/
  cout << "2D corners:" << corners << endl;
  std::chrono::time_point<std::chrono::system_clock> start_time = std::chrono::system_clock::now();
  auto [rvec, tvec, board_image_pixels] = chessboardProjection(corners, image_color);
  std::chrono::time_point<std::chrono::system_clock> end_time = std::chrono::system_clock::now();
  cout << "-----epnp:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;
  
  //角点绘制
  /*cout << cornersf << endl;
  cv::drawChessboardCorners(image_color, cv::Size(i_params.chessboard_pattern_size.height, i_params.chessboard_pattern_size.width), cornersf, pattern_found);
  cv::imshow("chessboard corners", image_color);
  cv::waitKey(0);*/

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  return std::make_tuple(board_image_pixels, rmat, tvec);
}

void makeImageFromPointCloud(const pcl::PointCloud<POINT_TYPE>::ConstPtr &input_pc, std::vector<cv::Point3d> cross_point, cv::Mat rMat, cv::Mat tMat, cv::Mat extrinsic_matrix)
{
    pcl::PointCloud<POINT_TYPE>::Ptr filtere_pc (new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr transform_pc (new pcl::PointCloud<POINT_TYPE>);
    Eigen::Matrix4d init_mat = Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d icp_mat = Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d ttt = Eigen::Matrix4d::Identity ();
    for (int i=0;i<input_pc->points.size();i++)
    {
      if(input_pc->points[i].x < 0) //過濾掉在後方的點雲
        continue;
      POINT_TYPE  p;
      p.x=input_pc->points[i].x;
      p.y=input_pc->points[i].y;
      p.z=input_pc->points[i].z;
      p.intensity = input_pc->points[i].intensity;  //继承之前点云的intensity
      filtere_pc->points.push_back(p);
    }
    if(IS_CLOCKWISE)
    {
      init_mat(0, 0) = i_params.clockwise_icp_init_rotmat[0];
      init_mat(0, 1) = i_params.clockwise_icp_init_rotmat[1];
      init_mat(0, 2) = i_params.clockwise_icp_init_rotmat[2];
      init_mat(1, 0) = i_params.clockwise_icp_init_rotmat[3];
      init_mat(1, 1) = i_params.clockwise_icp_init_rotmat[4];
      init_mat(1, 2) = i_params.clockwise_icp_init_rotmat[5];
      init_mat(2, 0) = i_params.clockwise_icp_init_rotmat[6];
      init_mat(2, 1) = i_params.clockwise_icp_init_rotmat[7];
      init_mat(2, 2) = i_params.clockwise_icp_init_rotmat[8];
      init_mat(0, 3) = i_params.clockwise_icp_init_tranmat[0];
      init_mat(1, 3) = i_params.clockwise_icp_init_tranmat[1];
      init_mat(2, 3) = i_params.clockwise_icp_init_tranmat[2];

      icp_mat(0, 0) = i_params.clockwise_icp_rotmat[0];
      icp_mat(0, 1) = i_params.clockwise_icp_rotmat[1];
      icp_mat(0, 2) = i_params.clockwise_icp_rotmat[2];
      icp_mat(1, 0) = i_params.clockwise_icp_rotmat[3];
      icp_mat(1, 1) = i_params.clockwise_icp_rotmat[4];
      icp_mat(1, 2) = i_params.clockwise_icp_rotmat[5];
      icp_mat(2, 0) = i_params.clockwise_icp_rotmat[6];
      icp_mat(2, 1) = i_params.clockwise_icp_rotmat[7];
      icp_mat(2, 2) = i_params.clockwise_icp_rotmat[8];
      icp_mat(0, 3) = i_params.clockwise_icp_tranmat[0];
      icp_mat(1, 3) = i_params.clockwise_icp_tranmat[1];
      icp_mat(2, 3) = i_params.clockwise_icp_tranmat[2];
    }
    else
    {
      init_mat(0, 0) = i_params.counterclockwise_icp_init_rotmat[0];
      init_mat(0, 1) = i_params.counterclockwise_icp_init_rotmat[1];
      init_mat(0, 2) = i_params.counterclockwise_icp_init_rotmat[2];
      init_mat(1, 0) = i_params.counterclockwise_icp_init_rotmat[3];
      init_mat(1, 1) = i_params.counterclockwise_icp_init_rotmat[4];
      init_mat(1, 2) = i_params.counterclockwise_icp_init_rotmat[5];
      init_mat(2, 0) = i_params.counterclockwise_icp_init_rotmat[6];
      init_mat(2, 1) = i_params.counterclockwise_icp_init_rotmat[7];
      init_mat(2, 2) = i_params.counterclockwise_icp_init_rotmat[8];
      init_mat(0, 3) = i_params.counterclockwise_icp_init_tranmat[0];
      init_mat(1, 3) = i_params.counterclockwise_icp_init_tranmat[1];
      init_mat(2, 3) = i_params.counterclockwise_icp_init_tranmat[2];

      icp_mat(0, 0) = i_params.counterclockwise_icp_rotmat[0];
      icp_mat(0, 1) = i_params.counterclockwise_icp_rotmat[1];
      icp_mat(0, 2) = i_params.counterclockwise_icp_rotmat[2];
      icp_mat(1, 0) = i_params.counterclockwise_icp_rotmat[3];
      icp_mat(1, 1) = i_params.counterclockwise_icp_rotmat[4];
      icp_mat(1, 2) = i_params.counterclockwise_icp_rotmat[5];
      icp_mat(2, 0) = i_params.counterclockwise_icp_rotmat[6];
      icp_mat(2, 1) = i_params.counterclockwise_icp_rotmat[7];
      icp_mat(2, 2) = i_params.counterclockwise_icp_rotmat[8];
      icp_mat(0, 3) = i_params.counterclockwise_icp_tranmat[0];
      icp_mat(1, 3) = i_params.counterclockwise_icp_tranmat[1];
      icp_mat(2, 3) = i_params.counterclockwise_icp_tranmat[2];
    }
    
    ttt = icp_mat*init_mat;

    pcl::transformPointCloud (*filtere_pc, *transform_pc, ttt);
    cout << "ttt:" << ttt << endl;

    cv::Mat image_color;
    image_color = cv::imread(i_params.image_filename, cv::IMREAD_COLOR);

    // Read 3D points
    pcl::PointCloud<POINT_TYPE> cloud;
    cloud=*transform_pc; 
    
    std::vector<cv::Point3f> objectPoints;
    for (const auto& point : cloud)
    {
      objectPoints.push_back(cv::Point3f(point.x, point.y, point.z));
    }
    cout << "rMat:" << rMat << endl;
    cout << "tMat:" << tMat/1000 << endl;

    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(objectPoints, rMat, tMat/1000, i_params.cameramat, i_params.distcoeff, projectedPoints);
    int count=0;
    for (auto &point : projectedPoints)
    {
      //cv::circle(image_color, point, 2, CV_RGB(cloud[count].intensity, cloud[count].intensity, cloud[count].intensity), -1);  //投影intensity
      cv::circle(image_color, point, 2, CV_RGB(0,255,0), -1);  //投影單一顏色
      count++;
    }

    //reprojetion cross_point
    for (int i = 0 ; i < cross_point.size() ; i++)
    {
      cv::Mat invR = extrinsic_matrix(cv::Rect(0, 0, 3, 3)).t();
      cv::Mat invT = -invR * (extrinsic_matrix(cv::Rect(3, 0, 1, 3)));
      cv::Mat point(1, 3, CV_32F);
      point.at<float>(0) = cross_point[i].x;
      point.at<float>(1) = cross_point[i].y;
      point.at<float>(2) = cross_point[i].z;

      point = point*invR.t()+ invT.t();
      float tmpx = point.at<float>(0)/point.at<float>(2);
      float tmpy = point.at<float>(1)/point.at<float>(2);
      float imagepoint_x = i_params.cameramat.at<double>(0, 0)*tmpx + i_params.cameramat.at<double>(0, 2);
      float imagepoint_y = i_params.cameramat.at<double>(1, 1)*tmpy + i_params.cameramat.at<double>(1, 2);

      cv::circle(image_color, cv::Point(imagepoint_x, imagepoint_y), 5, CV_RGB(255,0,0), -1);
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_color).toImageMsg();
    pcd2image_publisher.publish(img_msg); //cv::Mat
}

cv::Mat output_extrinsic(cv::Mat rMat, cv::Mat tMat)
{
  cv::Mat r_Mat = cv::Mat::eye(4,4, CV_32F);
	rMat.copyTo(r_Mat(cv::Rect_<float>(0,0,3,3)));

  cv::Mat init_mat = cv::Mat::eye(4,4, CV_32F);
  cv::Mat icp_mat = cv::Mat::eye(4,4, CV_32F);
  cv::Mat extrinsic_matrix = cv::Mat::eye(4,4, CV_32F);
  if(IS_CLOCKWISE)
  {
    init_mat.at<float>(0,0) = i_params.clockwise_icp_init_rotmat[0];
    init_mat.at<float>(0,1) = i_params.clockwise_icp_init_rotmat[1];
    init_mat.at<float>(0,2) = i_params.clockwise_icp_init_rotmat[2];
    init_mat.at<float>(1,0) = i_params.clockwise_icp_init_rotmat[3];
    init_mat.at<float>(1,1) = i_params.clockwise_icp_init_rotmat[4];
    init_mat.at<float>(1,2) = i_params.clockwise_icp_init_rotmat[5];
    init_mat.at<float>(2,0) = i_params.clockwise_icp_init_rotmat[6];
    init_mat.at<float>(2,1) = i_params.clockwise_icp_init_rotmat[7];
    init_mat.at<float>(2,2) = i_params.clockwise_icp_init_rotmat[8];
    init_mat.at<float>(0,3) = i_params.clockwise_icp_init_tranmat[0];
    init_mat.at<float>(1,3) = i_params.clockwise_icp_init_tranmat[1];
    init_mat.at<float>(2,3) = i_params.clockwise_icp_init_tranmat[2];

    icp_mat.at<float>(0,0) = i_params.clockwise_icp_rotmat[0];
    icp_mat.at<float>(0,1) = i_params.clockwise_icp_rotmat[1];
    icp_mat.at<float>(0,2) = i_params.clockwise_icp_rotmat[2];
    icp_mat.at<float>(1,0) = i_params.clockwise_icp_rotmat[3];
    icp_mat.at<float>(1,1) = i_params.clockwise_icp_rotmat[4];
    icp_mat.at<float>(1,2) = i_params.clockwise_icp_rotmat[5];
    icp_mat.at<float>(2,0) = i_params.clockwise_icp_rotmat[6];
    icp_mat.at<float>(2,1) = i_params.clockwise_icp_rotmat[7];
    icp_mat.at<float>(2,2) = i_params.clockwise_icp_rotmat[8];
    icp_mat.at<float>(0,3) = i_params.clockwise_icp_tranmat[0];
    icp_mat.at<float>(1,3) = i_params.clockwise_icp_tranmat[1];
    icp_mat.at<float>(2,3) = i_params.clockwise_icp_tranmat[2];
  }
  else
  {
    init_mat.at<float>(0,0) = i_params.counterclockwise_icp_init_rotmat[0];
    init_mat.at<float>(0,1) = i_params.counterclockwise_icp_init_rotmat[1];
    init_mat.at<float>(0,2) = i_params.counterclockwise_icp_init_rotmat[2];
    init_mat.at<float>(1,0) = i_params.counterclockwise_icp_init_rotmat[3];
    init_mat.at<float>(1,1) = i_params.counterclockwise_icp_init_rotmat[4];
    init_mat.at<float>(1,2) = i_params.counterclockwise_icp_init_rotmat[5];
    init_mat.at<float>(2,0) = i_params.counterclockwise_icp_init_rotmat[6];
    init_mat.at<float>(2,1) = i_params.counterclockwise_icp_init_rotmat[7];
    init_mat.at<float>(2,2) = i_params.counterclockwise_icp_init_rotmat[8];
    init_mat.at<float>(0,3) = i_params.counterclockwise_icp_init_tranmat[0];
    init_mat.at<float>(1,3) = i_params.counterclockwise_icp_init_tranmat[1];
    init_mat.at<float>(2,3) = i_params.counterclockwise_icp_init_tranmat[2];

    icp_mat.at<float>(0,0) = i_params.counterclockwise_icp_rotmat[0];
    icp_mat.at<float>(0,1) = i_params.counterclockwise_icp_rotmat[1];
    icp_mat.at<float>(0,2) = i_params.counterclockwise_icp_rotmat[2];
    icp_mat.at<float>(1,0) = i_params.counterclockwise_icp_rotmat[3];
    icp_mat.at<float>(1,1) = i_params.counterclockwise_icp_rotmat[4];
    icp_mat.at<float>(1,2) = i_params.counterclockwise_icp_rotmat[5];
    icp_mat.at<float>(2,0) = i_params.counterclockwise_icp_rotmat[6];
    icp_mat.at<float>(2,1) = i_params.counterclockwise_icp_rotmat[7];
    icp_mat.at<float>(2,2) = i_params.counterclockwise_icp_rotmat[8];
    icp_mat.at<float>(0,3) = i_params.counterclockwise_icp_tranmat[0];
    icp_mat.at<float>(1,3) = i_params.counterclockwise_icp_tranmat[1];
    icp_mat.at<float>(2,3) = i_params.counterclockwise_icp_tranmat[2];
  }
  
  extrinsic_matrix = (r_Mat*icp_mat*init_mat).t();
  extrinsic_matrix.at<float>(0,3) = -(extrinsic_matrix.at<float>(3,2) + tMat.at<double>(2,0)/1000);
  extrinsic_matrix.at<float>(1,3) = extrinsic_matrix.at<float>(3,0) + tMat.at<double>(0,0)/1000;
  extrinsic_matrix.at<float>(2,3) = extrinsic_matrix.at<float>(3,1) + tMat.at<double>(1,0)/1000;
  extrinsic_matrix.at<float>(3,0) = 0;
  extrinsic_matrix.at<float>(3,1) = 0;
  extrinsic_matrix.at<float>(3,2) = 0;
  cout << "extrinsic matrix:" << extrinsic_matrix << endl;

  return extrinsic_matrix;
}

void reprojection_error(cv::Mat extrinsic_matrix, std::vector<cv::Point3d> cross_point, std::vector<cv::Point2d> board_image_pixels)
{
  vector<int> num_3d;
  vector<int> num_2d;

  if(IS_CLOCKWISE)
  {
    num_3d = i_params.cw_r_3d;
    num_2d = i_params.cw_r_2d;

    /*num_3d = i_params.cw_r_3d;
    num_3d.push_back(i_params.ccw_r_3d[0]+5);
    num_3d.push_back(i_params.ccw_r_3d[1]+5);
    num_3d.push_back(i_params.ccw_r_3d[2]+5);
    num_3d.push_back(i_params.ccw_r_3d[3]+5);
    num_2d = i_params.cw_r_2d;
    num_2d.push_back(i_params.ccw_r_2d[0]+5);
    num_2d.push_back(i_params.ccw_r_2d[1]+5);
    num_2d.push_back(i_params.ccw_r_2d[2]+5);
    num_2d.push_back(i_params.ccw_r_2d[3]+5);*/
  }
  else
  {
    num_3d = i_params.ccw_r_3d;
    num_2d = i_params.ccw_r_2d;

    /*num_3d = i_params.cw_r_3d;
    num_3d.push_back(i_params.ccw_r_3d[0]+5);
    num_3d.push_back(i_params.ccw_r_3d[1]+5);
    num_3d.push_back(i_params.ccw_r_3d[2]+5);
    num_3d.push_back(i_params.ccw_r_3d[3]+5);
    num_2d = i_params.cw_r_2d;
    num_2d.push_back(i_params.ccw_r_2d[0]+5);
    num_2d.push_back(i_params.ccw_r_2d[1]+5);
    num_2d.push_back(i_params.ccw_r_2d[2]+5);
    num_2d.push_back(i_params.ccw_r_2d[3]+5);*/
  }

  float error_mean = 0.0;
  cv::Mat invR = extrinsic_matrix(cv::Rect(0, 0, 3, 3)).t();
  cv::Mat invT = -invR * (extrinsic_matrix(cv::Rect(3, 0, 1, 3)));

  for(int i=0; i<4; i++)
  {
    cv::Mat point(1, 3, CV_32F);
    point.at<float>(0) = cross_point[num_3d[i]].x;
    point.at<float>(1) = cross_point[num_3d[i]].y;
    point.at<float>(2) = cross_point[num_3d[i]].z;

    point = point*invR.t()+ invT.t();
    float tmpx = point.at<float>(0)/point.at<float>(2);
    float tmpy = point.at<float>(1)/point.at<float>(2);
    float imagepoint_x = i_params.cameramat.at<double>(0, 0)*tmpx + i_params.cameramat.at<double>(0, 2);
    float imagepoint_y = i_params.cameramat.at<double>(1, 1)*tmpy + i_params.cameramat.at<double>(1, 2);

    float image_x = board_image_pixels[num_2d[i]].x;
    float image_y = board_image_pixels[num_2d[i]].y;

    float error = sqrt(pow(imagepoint_x-image_x,2) + pow(imagepoint_y-image_y,2));
    error_mean += error;
    cout << "error" << i+1 << ":" << error << endl;
    cout << "cross_point_x:" << cross_point[num_3d[i]].x << "; board_image_pixels_x:" << board_image_pixels[num_2d[i]].x << endl;
    cout << "imagepoint_x:" << imagepoint_x << "; imagepoint_y:" << imagepoint_y << endl;
  }
  
  cout << "reprojection_error_mean:" << error_mean/4 << endl;
}

float *mRot2Quat(Eigen::Matrix3f rot, char dir)
{
  float r11,r12,r13,r21,r22,r23,r31,r32,r33;
  if(dir == 'x')
  {
    r11 = 0;//rot(0, 0);
    r12 = 0;//rot(0, 1);
    r13 = 0;//rot(0, 2);
    r21 = rot(1, 0);
    r22 = rot(1, 1);
    r23 = rot(1, 2);
    r31 = 0;//rot(2, 0);
    r32 = 0;//rot(2, 1);
    r33 = 0;//rot(2, 2);
  }
  else if(dir == 'y')
  {
    r11 = rot(0, 0);
    r12 = rot(0, 1);
    r13 = rot(0, 2);
    r21 = 0;//rot(1, 0);
    r22 = 0;//rot(1, 1);
    r23 = 0;//rot(1, 2);
    r31 = 0;//rot(2, 0);
    r32 = 0;//rot(2, 1);
    r33 = 0;//rot(2, 2);
  }
  else if(dir == 'z')
  {
    r11 = 0;//rot(0, 0);
    r12 = 0;//rot(0, 1);
    r13 = 0;//rot(0, 2);
    r21 = 0;//rot(1, 0);
    r22 = 0;//rot(1, 1);
    r23 = 0;//rot(1, 2);
    r31 = rot(2, 0);
    r32 = rot(2, 1);
    r33 = rot(2, 2);
  }
	float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0f) {
		q0 = 0.0f;
	}
	if (q1 < 0.0f) {
		q1 = 0.0f;
	}
	if (q2 < 0.0f) {
		q2 = 0.0f;
	}
	if (q3 < 0.0f) {
		q3 = 0.0f;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= (r32 - r23 >= 0.0f) ? +1.0f : -1.0f;
		q2 *= (r13 - r31 >= 0.0f) ? +1.0f : -1.0f;
		q3 *= (r21 - r12 >= 0.0f) ? +1.0f : -1.0f;
	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q0 *= (r32 - r23 >= 0.0f) ? +1.0f : -1.0f;
		q1 *= +1.0f;
		q2 *= (r21 + r12 >= 0.0f) ? +1.0f : -1.0f;
		q3 *= (r13 + r31 >= 0.0f) ? +1.0f : -1.0f;
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q0 *= (r13 - r31 >= 0.0f) ? +1.0f : -1.0f;
		q1 *= (r21 + r12 >= 0.0f) ? +1.0f : -1.0f;
		q2 *= +1.0f;
		q3 *= (r32 + r23 >= 0.0f) ? +1.0f : -1.0f;
	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q0 *= (r21 - r12 >= 0.0f) ? +1.0f : -1.0f;
		q1 *= (r31 + r13 >= 0.0f) ? +1.0f : -1.0f;
		q2 *= (r32 + r23 >= 0.0f) ? +1.0f : -1.0f;
		q3 *= +1.0f;
	}
	else {
		printf("coding error\n");
	}
	float r = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 /= r;
	q1 /= r;
	q2 /= r;
	q3 /= r;

  float *q;
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;

  return q;
}

/*void aa(const pcl::PointCloud<POINT_TYPE>::ConstPtr &input_pc)
{
  pcl::PointCloud<POINT_TYPE>::Ptr cloud_ptr(new pcl::PointCloud<POINT_TYPE>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>());

  pcl::PointCloud<POINT_TYPE> cloud;
  POINT_TYPE p;
  for (pcl::PointCloud<POINT_TYPE>::const_iterator item = input_pc->begin(); item != input_pc->end(); item++){
    p.x = (double)item->x*1000;
    p.y = (double)item->y*1000;
    p.z = (double)item->z*1000;
    p.intensity = (double)item->intensity;
    cloud.push_back(p);
  }
  cloud_ptr = cloud.makeShared();
 
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_ptr, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_ptr, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	//eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
 
	std::cout << eigenValuesPCA << std::endl;
	std::cout << eigenVectorsPCA << std::endl;

  /////////
  geometry_msgs::PoseStamped msg1; 
  msg1.header.stamp = ros::Time::now();
  msg1.header.frame_id = "/world";
  float *q_x = mRot2Quat(eigenVectorsPCA, 'x');
  msg1.pose.position.x = pcaCentroid(0);
  msg1.pose.position.y = pcaCentroid(1);
  msg1.pose.position.z = pcaCentroid(2);

  msg1.pose.orientation.x = q_x[0];
  msg1.pose.orientation.y = q_x[1];
  msg1.pose.orientation.z = q_x[2];
  msg1.pose.orientation.w = q_x[3];

  publisher_testpca1.publish(msg1);
  /////////
  geometry_msgs::PoseStamped msg2; 
  msg2.header.stamp = ros::Time::now();
  msg2.header.frame_id = "/world";
  float *q_y = mRot2Quat(eigenVectorsPCA, 'y');
  msg2.pose.position.x = pcaCentroid(0);
  msg2.pose.position.y = pcaCentroid(1);
  msg2.pose.position.z = pcaCentroid(2);
    
  msg2.pose.orientation.x = q_y[0];
  msg2.pose.orientation.y = q_y[1];
  msg2.pose.orientation.z = q_y[2];
  msg2.pose.orientation.w = q_y[3];

  publisher_testpca2.publish(msg2);
  /////////
  geometry_msgs::PoseStamped msg3; 
  msg3.header.stamp = ros::Time::now();
  msg3.header.frame_id = "/world";
  float *q_z = mRot2Quat(eigenVectorsPCA, 'z');
  msg3.pose.position.x = pcaCentroid(0);
  msg3.pose.position.y = pcaCentroid(1);
  msg3.pose.position.z = pcaCentroid(2);

  msg3.pose.orientation.x = q_z[0];
  msg3.pose.orientation.y = q_z[1];
  msg3.pose.orientation.z = q_z[2];
  msg3.pose.orientation.w = q_z[3];

  publisher_testpca3.publish(msg3);

  //view
  Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 
 
	pcl::PointCloud<POINT_TYPE>::Ptr transformedCloud(new pcl::PointCloud<POINT_TYPE>);
	pcl::transformPointCloud(*cloud_ptr, *transformedCloud, transform);

  //转换到原点时的主方向
	POINT_TYPE o;
	o.x = 0.0;
	o.y = 0.0;
	o.z = 0.0;
	Eigen::Affine3f tra_aff(transform);
	Eigen::Vector3f pz = eigenVectorsPCA.col(0);
	Eigen::Vector3f py = eigenVectorsPCA.col(1);
	Eigen::Vector3f px = eigenVectorsPCA.col(2);
	pcl::transformVector(pz, pz, tra_aff);
	pcl::transformVector(py, py, tra_aff);
	pcl::transformVector(px, px, tra_aff);
	POINT_TYPE pcaZ;
	pcaZ.x = 1000 * pz(0);
	pcaZ.y = 1000 * pz(1);
	pcaZ.z = 1000 * pz(2);
	POINT_TYPE pcaY;
	pcaY.x = 1000 * py(0);
	pcaY.y = 1000 * py(1);
	pcaY.z = 1000 * py(2);
	POINT_TYPE pcaX;
	pcaX.x = 1000 * px(0);
	pcaX.y = 1000 * px(1);
	pcaX.z = 1000 * px(2);
 
	//初始位置时的主方向
	POINT_TYPE c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	POINT_TYPE pcZ;
	pcZ.x = 1000 * eigenVectorsPCA(0, 0) + c.x;
	pcZ.y = 1000 * eigenVectorsPCA(1, 0) + c.y;
	pcZ.z = 1000 * eigenVectorsPCA(2, 0) + c.z;
	POINT_TYPE pcY;
	pcY.x = 1000 * eigenVectorsPCA(0, 1) + c.x;
	pcY.y = 1000 * eigenVectorsPCA(1, 1) + c.y;
	pcY.z = 1000 * eigenVectorsPCA(2, 1) + c.z;
	POINT_TYPE pcX;
	pcX.x = 1000 * eigenVectorsPCA(0, 2) + c.x;
	pcX.y = 1000 * eigenVectorsPCA(1, 2) + c.y;
	pcX.z = 1000 * eigenVectorsPCA(2, 2) + c.z;
  //visualization
	pcl::visualization::PCLVisualizer viewer;
	pcl::visualization::PointCloudColorHandlerCustom<POINT_TYPE> color_handler(cloud_ptr,255, 0, 0); 
	pcl::visualization::PointCloudColorHandlerCustom<POINT_TYPE> tc_handler(transformedCloud, 0, 255, 0); 
	viewer.addPointCloud(cloud_ptr,color_handler,"cloud");
	viewer.addPointCloud(transformedCloud,tc_handler,"transformCloud");
	
	viewer.addArrow(pcaZ, o, 0.0, 0.0, 1.0, false, "arrow_Z");
	viewer.addArrow(pcaY, o, 0.0, 1.0, 0.0, false, "arrow_Y");
	viewer.addArrow(pcaX, o, 1.0, 0.0, 0.0, false, "arrow_X");
 
	viewer.addArrow(pcZ, c, 0.0, 0.0, 1.0, false, "arrow_z");
	viewer.addArrow(pcY, c, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(pcX, c, 1.0, 0.0, 0.0, false, "arrow_x");
 
	viewer.addCoordinateSystem(100);
	viewer.setBackgroundColor(0.5, 0.5, 0.5);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
 
	}
}*/

int main(int argc, char **argv)
{
  std::chrono::time_point<std::chrono::system_clock> start_time;
  std::chrono::time_point<std::chrono::system_clock> end_time;
  double line_arr[4][6];
  bool loadpcd_done = false;
  bool savefile_done = false;

  ros::init(argc, argv, "mypaper");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it(nh_);
  ros::Rate r(100);
  get_params(nh_);

  publisher_FullPcd = nh_.advertise<sensor_msgs::PointCloud2>("FullPcd", 0);
  publisher_PassfilteredPcd = nh_.advertise<sensor_msgs::PointCloud2>("PassfilteredPcd", 10);
  publisher_RansacPcd = nh_.advertise<sensor_msgs::PointCloud2>("RansacPcd", 10);
  publisher_ConvexHullPcd = nh_.advertise<sensor_msgs::PointCloud2>("ConvexhullPcd", 10);
  publisher_marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  image_publisher = it.advertise("camera_features", 1);
  pcd2image_publisher = it.advertise("pcd2image", 1);
  publisher_testpca1 = nh_.advertise<geometry_msgs::PoseStamped>("testpca1", 10);
  publisher_testpca2 = nh_.advertise<geometry_msgs::PoseStamped>("testpca2", 10);
  publisher_testpca3 = nh_.advertise<geometry_msgs::PoseStamped>("testpca3", 10);
  pcl::PointCloud<POINT_TYPE>::Ptr full_pcd(new pcl::PointCloud<POINT_TYPE>);

  while (ros::ok())
  {
#if DISPLAY_ALL
    pcl::PointCloud<POINT_TYPE>::Ptr passthrough_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr ransac_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr convexhull_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr convexhull_pcd2(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr convexhull_pcd_all(new pcl::PointCloud<POINT_TYPE>);

    if(!loadpcd_done)
      loadpcd_done = read_pcd(full_pcd);

    IS_CLOCKWISE = true;
    std::cout << "-------------right-------------" << std::endl;
    passthrough(full_pcd, passthrough_pcd);
    ransac(passthrough_pcd, ransac_pcd);
    bound(ransac_pcd, convexhull_pcd);
    save_point(convexhull_pcd);
    hough3dlines(line_arr);
    std::vector<cv::Point3d> cross_point = find_crossline(line_arr);
    auto [board_image_pixels, in_rMat, in_tMat] = findImgChessboardCorners();
    cv::Mat extrinsic_matrix = output_extrinsic(in_rMat, in_tMat);
    
    std::cout << "-------------left-------------" << std::endl;
    IS_CLOCKWISE = false;
    passthrough(full_pcd, passthrough_pcd);
    ransac(passthrough_pcd, ransac_pcd);
    bound(ransac_pcd, convexhull_pcd2);
    save_point(convexhull_pcd2);
    hough3dlines(line_arr);
    std::vector<cv::Point3d> cross_point2 = find_crossline(line_arr);
    auto [board_image_pixels2, in_rMat2, in_tMat2] = findImgChessboardCorners();
    cv::Mat extrinsic_matrix2 = output_extrinsic(in_rMat2, in_tMat2);
    *convexhull_pcd_all = *convexhull_pcd + *convexhull_pcd2;
    for(int i=0;i<cross_point2.size();i++)
    {
      cross_point.push_back(cross_point2[i]);
    }
    for(int i=0;i<board_image_pixels2.size();i++)
    {
      board_image_pixels.push_back(board_image_pixels2[i]);
    }
    makeImageFromPointCloud(convexhull_pcd_all, cross_point, in_rMat2, in_tMat2, extrinsic_matrix2);
    reprojection_error(extrinsic_matrix2, cross_point, board_image_pixels);
    IS_CLOCKWISE = true;
    /*makeImageFromPointCloud(convexhull_pcd_all, cross_point, in_rMat, in_tMat, extrinsic_matrix);
    reprojection_error(extrinsic_matrix, cross_point, board_image_pixels);*/
    /*makeImageFromPointCloud(convexhull_pcd_all, cross_point, in_rMat, in_tMat, (extrinsic_matrix+extrinsic_matrix2)/2);
    reprojection_error((extrinsic_matrix+extrinsic_matrix2)/2, cross_point, board_image_pixels);*/

#else
    pcl::PointCloud<POINT_TYPE>::Ptr passthrough_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr ransac_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr convexhull_pcd(new pcl::PointCloud<POINT_TYPE>);
    std::vector<cv::Point3d> cross_point;

    std::cout << "run" << std::endl;
    if(!loadpcd_done)
      loadpcd_done = read_pcd(full_pcd);

    passthrough(full_pcd, passthrough_pcd);
    ransac(passthrough_pcd, ransac_pcd);
    bound(ransac_pcd, convexhull_pcd);
    if (!savefile_done)
    {
      save_point(convexhull_pcd);
      savefile_done = true;
    }
    hough3dlines(line_arr);
    cross_point = find_crossline(line_arr);
    auto [board_image_pixels, in_rMat, in_tMat] = findImgChessboardCorners();
    
    cv::Mat extrinsic_matrix = output_extrinsic(in_rMat, in_tMat);
    makeImageFromPointCloud(convexhull_pcd, cross_point, in_rMat, in_tMat, extrinsic_matrix);
    reprojection_error(extrinsic_matrix, cross_point, board_image_pixels);
#endif

/* //以下用來測pca箭頭
#else
    pcl::PointCloud<POINT_TYPE>::Ptr passthrough_pcd(new pcl::PointCloud<POINT_TYPE>);
    if(!loadpcd_done)
      loadpcd_done = read_pcd(full_pcd);

    passthrough(full_pcd, passthrough_pcd);
    aa(passthrough_pcd);
#endif*/

/* //以下用來測執行時間
#else
    
    pcl::PointCloud<POINT_TYPE>::Ptr passthrough_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr ransac_pcd(new pcl::PointCloud<POINT_TYPE>);
    pcl::PointCloud<POINT_TYPE>::Ptr convexhull_pcd(new pcl::PointCloud<POINT_TYPE>);
    std::vector<cv::Point3d> cross_point;

    std::cout << "run" << std::endl;
    if(!loadpcd_done)
      loadpcd_done = read_pcd(full_pcd);

    start_time = std::chrono::system_clock::now();
    passthrough(full_pcd, passthrough_pcd);
    end_time = std::chrono::system_clock::now();
    cout << "-----passthrough_pcd_time:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

    start_time = std::chrono::system_clock::now();
    ransac(passthrough_pcd, ransac_pcd);
    end_time = std::chrono::system_clock::now();
    cout << "-----ransac_pcd_time:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

    start_time = std::chrono::system_clock::now();
    bound(ransac_pcd, convexhull_pcd);
    end_time = std::chrono::system_clock::now();
    cout << "-----convexhull_pcd_time:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

    if (!savefile_done)
    {
      save_point(convexhull_pcd);
      savefile_done = true;
    }

    start_time = std::chrono::system_clock::now();
    hough3dlines(line_arr);
    end_time = std::chrono::system_clock::now();
    cout << "-----hough3dlines_time:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

    start_time = std::chrono::system_clock::now();
    cross_point = find_crossline(line_arr);
    end_time = std::chrono::system_clock::now();
    cout << "-----find_crossline_time:" << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0 << endl;

    auto [board_image_pixels, in_rMat, in_tMat] = findImgChessboardCorners();
    cv::Mat extrinsic_matrix = output_extrinsic(in_rMat, in_tMat);
    makeImageFromPointCloud(full_pcd, cross_point, in_rMat, in_tMat, extrinsic_matrix);
    reprojection_error(extrinsic_matrix, cross_point, board_image_pixels);
#endif*/
    
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
