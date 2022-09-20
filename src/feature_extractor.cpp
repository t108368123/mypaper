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

#include "hough-3d-lines/hough3dlines.h"

#include <iostream>
#include <cmath>
#include <string>
#include <fstream>
using namespace std;

//360_a1
#define image_filename "/home/a123/Documents/catkin_ws/src/mypaper/data/fusion360/a1.jpg"
#define pcd_filename "/home/a123/Documents/catkin_ws/src/mypaper/data/fusion360/a1.pcd"
#define X_MIN 4.151742
#define X_MAX 4.2686
#define Y_MIN -0.7875
#define Y_MAX 0.1531
#define Z_MIN -0.5896
#define Z_MAX 0.401703

ros::Publisher publisher_FullPcd;
ros::Publisher publisher_PassfilteredPcd;
ros::Publisher publisher_RansacPcd;
ros::Publisher publisher_ConvexHullPcd;
ros::Publisher publisher_marker_pub;
image_transport::Publisher image_publisher;
image_transport::Publisher pcd2image_publisher;

struct initial_parameters_t
{
  cv::Size chessboard_pattern_size;
  int square_length;                // in millimetres
  cv::Size board_dimensions;        // in millimetres
  cv::Point3d cb_translation_error; // in millimetres
  cv::Mat cameramat, distcoeff;
  std::pair<int, int> image_size; // in pixels
  std::string camera_topic, camera_info, lidar_topic;
  float icp_init_rotmat[4][3];
};

initial_parameters_t i_params;

void get_params()
{
  i_params.chessboard_pattern_size.width = 5;
  i_params.chessboard_pattern_size.height = 7;
  
  i_params.board_dimensions.width = 590;
  i_params.board_dimensions.height = 840;
  i_params.square_length = 95;
  i_params.cb_translation_error.x = 0;
  i_params.cb_translation_error.y = 0;
  i_params.cameramat = cv::Mat::zeros(3, 3, CV_64F);
  i_params.cameramat.at<double>(0, 0) = 1164.6233338297;
  i_params.cameramat.at<double>(0, 2) = 950.1242940800;
  i_params.cameramat.at<double>(1, 1) = 1161.1018211652;
  i_params.cameramat.at<double>(1, 2) = 538.5516554830;
  i_params.cameramat.at<double>(2, 2) = 1;
  i_params.distcoeff = cv::Mat::eye(1, 4, CV_64F);
  i_params.distcoeff.at<double>(0) = 0.107542;
  i_params.distcoeff.at<double>(1) = -0.160397;
  i_params.distcoeff.at<double>(2) = 0.000789;
  i_params.distcoeff.at<double>(3) = -0.004844;

  //順時針45度
  /*i_params.icp_init_rotmat[0][0] = 0;
  i_params.icp_init_rotmat[0][1] = -0.7071067;
  i_params.icp_init_rotmat[0][2] = -0.7071067;
  i_params.icp_init_rotmat[1][0] = 0;
  i_params.icp_init_rotmat[1][1] = -0.7071067;
  i_params.icp_init_rotmat[1][2] = 0.7071067;
  i_params.icp_init_rotmat[2][0] = -1;
  i_params.icp_init_rotmat[2][1] = 0;
  i_params.icp_init_rotmat[2][2] = 0;
  i_params.icp_init_rotmat[3][0] = 0.8; //
  i_params.icp_init_rotmat[3][1] = 1.3;
  i_params.icp_init_rotmat[3][2] = 5;*/

  //逆時針45度
  /*i_params.icp_init_rotmat[0][0] = 0;
  i_params.icp_init_rotmat[0][1] = 0.7071067;
  i_params.icp_init_rotmat[0][2] = -0.7071067;
  i_params.icp_init_rotmat[1][0] = 0;
  i_params.icp_init_rotmat[1][1] = -0.7071067;
  i_params.icp_init_rotmat[1][2] = -0.7071067;
  i_params.icp_init_rotmat[2][0] = -1;
  i_params.icp_init_rotmat[2][1] = 0;
  i_params.icp_init_rotmat[2][2] = 0;
  i_params.icp_init_rotmat[3][0] = 0; //
  i_params.icp_init_rotmat[3][1] = -0.3;
  i_params.icp_init_rotmat[3][2] = 4;*/

  //逆時針_360
  i_params.icp_init_rotmat[0][0] = 0;
  i_params.icp_init_rotmat[0][1] = 0.7071067;
  i_params.icp_init_rotmat[0][2] = -0.7071067;
  i_params.icp_init_rotmat[1][0] = 0;
  i_params.icp_init_rotmat[1][1] = -0.7071067;
  i_params.icp_init_rotmat[1][2] = -0.7071067;
  i_params.icp_init_rotmat[2][0] = -1;
  i_params.icp_init_rotmat[2][1] = 0;
  i_params.icp_init_rotmat[2][2] = 0;
  i_params.icp_init_rotmat[3][0] = 0; //
  i_params.icp_init_rotmat[3][1] = -0.3;
  i_params.icp_init_rotmat[3][2] = 4;
}

bool read_pcd(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_filename, *cloud) == -1)
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

void passthrough(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pc)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr x(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr z(new pcl::PointCloud<pcl::PointXYZI>);
  // Filter out the experimental region
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(input_pc);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(X_MIN, X_MAX);
  pass.filter(*x);
  pass.setInputCloud(x);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(Z_MIN, Z_MAX);
  pass.filter(*z);
  pass.setInputCloud(z);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(Y_MIN, Y_MAX);
  pass.filter(*output_pc);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*output_pc, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  publisher_PassfilteredPcd.publish(cloud_msg);
}

auto ransac(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_pc, pcl::PointCloud<pcl::PointXYZI>::Ptr &output_pc)
{
  // Fit a plane through the board point cloud
  // Inliers give the indices of the points that are within the RANSAC threshold
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.004);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  seg.setInputCloud(input_pc);
  seg.segment(*inliers, *coefficients);

  // Check that segmentation succeeded
  if (coefficients->values.size() < 3)
  {
    ROS_WARN("Chessboard plane segmentation failed");
  }

  // Plane normal vector magnitude
  cv::Point3d lidar_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  lidar_normal /= -cv::norm(lidar_normal); // Normalise and flip the direction

  // Project the inliers on the fitted plane
  // When it freezes the chessboard after capture, what you see are the inlier points (filtered from the original)
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(input_pc);
  proj.setModelCoefficients(coefficients);
  proj.filter(*output_pc);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*output_pc, cloud_msg);
  cloud_msg.header.frame_id = "world";
  cloud_msg.header.stamp = ros::Time::now();
  publisher_RansacPcd.publish(cloud_msg);

  cv::Mat lidar_normal_mat(3, 1, CV_64FC1);
  lidar_normal_mat.at<double>(0) = lidar_normal.x;
  lidar_normal_mat.at<double>(1) = lidar_normal.y;
  lidar_normal_mat.at<double>(2) = lidar_normal.z;
  //cout << "lidar_normal" << lidar_normal << endl;
  return lidar_normal_mat;
}

void bound(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &boundPoints)
{
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Boundary> boundaries;
  pcl::BoundaryEstimation<pcl::PointXYZI, pcl::Normal, pcl::Boundary> est;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
  kdtree.setInputCloud(cloud);
  int k =2;
  float everagedistance =0;
  for (int i =0; i < cloud->size()/2;i++)
  {
          vector<int> nnh ;
          vector<float> squaredistance;
          //  pcl::PointXYZI p;
          //   p = cloud->points[i];
          kdtree.nearestKSearch(cloud->points[i],k,nnh,squaredistance);
          everagedistance += sqrt(squaredistance[1]);
          //   cout<<everagedistance<<endl;
  }
  everagedistance = everagedistance/(cloud->size()/2);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normEst;  //其中pcl::PointXYZI表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
  normEst.setInputCloud(cloud);
  normEst.setSearchMethod(tree);
  // normEst.setRadiusSearch(2);  //法向估计的半径
  normEst.setKSearch(500);  //法向估计的点数
  normEst.compute(*normals);

  //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
  est.setInputCloud(cloud);
  est.setInputNormals(normals);
  //  est.setAngleThreshold(90);
  //   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZI>::Ptr (new pcl::search::KdTree<pcl::PointXYZI>));
  est.setSearchMethod(tree);
  est.setKSearch(50);  //一般这里的数值越高，最终边界识别的精度越好
  //  est.setRadiusSearch(everagedistance);  //搜索半径
  est.compute(boundaries);

  pcl::PointCloud<pcl::PointXYZI> noBoundPoints;
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

void save_point(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_pc)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud = *input_pc;

  ofstream ofs;

  ofs.open("/home/a123/Documents/catkin_ws/src/mypaper/src/output.dat");
  if (!ofs.is_open())
  {
    cout << "Failed to open file.\n";
  }
  else
  {
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = cloud.begin(); item != cloud.end(); item++)
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
        if(corner[2] < Z_MIN-1 || corner[2] > Z_MAX+1)
          continue;

        cout << "crossline:" << endl;
        cout << "(" << corner[0]*1000 << " " << corner[1]*1000 << " " << corner[2]*1000 << ")" << endl;
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
  cout << "center:(" << lidar_centre_x*1000 << " " << lidar_centre_y*1000 << " " << lidar_centre_z*1000 << ")" << endl;
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

  double pixdiagonal = sqrt(pow(inner_cbcorner_pixels.front().x - inner_cbcorner_pixels.back().x, 2) + (pow(inner_cbcorner_pixels.front().y - inner_cbcorner_pixels.back().y, 2)));
  double len_diagonal = sqrt(pow(corners_3d.front().x - corners_3d.back().x, 2) + (pow(corners_3d.front().y - corners_3d.back().y, 2)));

  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_color).toImageMsg();
  image_publisher.publish(img_msg); //cv::Mat

  // Return all the necessary coefficients
  return std::make_tuple(rvec, tvec, board_corners_3d, board_image_pixels);
}

auto findImgChessboardCorners()
{
  cv::Mat image_color, image_gray, img;
  image_color = cv::imread(image_filename, cv::IMREAD_COLOR);
  cv::cvtColor(image_color, image_gray, CV_BGR2GRAY);

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
  }
  auto [rvec, tvec, board_corners_3d, board_image_pixels] = chessboardProjection(corners, image_color);
  
  //角点绘制
  /*cout << cornersf << endl;
  cv::drawChessboardCorners(image_color, cv::Size(i_params.chessboard_pattern_size.height, i_params.chessboard_pattern_size.width), cornersf, pattern_found);
  cv::imshow("chessboard corners", image_color);
  cv::waitKey(0);*/
  

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  std::vector<cv::Point3d> corner_vectors;
  for (auto &corner : board_corners_3d)
  {
    cv::Mat m(rmat * cv::Mat(corner).reshape(1) + tvec);
    corner_vectors.push_back(cv::Point3d(m));
  }

  return std::make_tuple(corner_vectors, board_image_pixels, rmat, tvec);
}

void makeImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &input_pc, std::vector<cv::Point3d> cross_point, cv::Mat rMat, cv::Mat tMat, cv::Mat extrinsic_matrix)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr transform_pc (new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Matrix4d init_mat = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d icp_mat = Eigen::Matrix4d::Identity ();
  Eigen::Matrix4d ttt = Eigen::Matrix4d::Identity ();

  init_mat(0, 0) = i_params.icp_init_rotmat[0][0];
  init_mat(0, 1) = i_params.icp_init_rotmat[0][1];
  init_mat(0, 2) = i_params.icp_init_rotmat[0][2];
  init_mat(1, 0) = i_params.icp_init_rotmat[1][0];
  init_mat(1, 1) = i_params.icp_init_rotmat[1][1];
  init_mat(1, 2) = i_params.icp_init_rotmat[1][2];
  init_mat(2, 0) = i_params.icp_init_rotmat[2][0];
  init_mat(2, 1) = i_params.icp_init_rotmat[2][1];
  init_mat(2, 2) = i_params.icp_init_rotmat[2][2];
  init_mat(0, 3) = i_params.icp_init_rotmat[3][0];//
  init_mat(1, 3) = i_params.icp_init_rotmat[3][1];
  init_mat(2, 3) = i_params.icp_init_rotmat[3][2];

  icp_mat(0, 0) = 0.983379;
  icp_mat(0, 1) = -0.178971;
  icp_mat(0, 2) = 0.030626;
  icp_mat(1, 0) = 0.174842;
  icp_mat(1, 1) = 0.978858;
  icp_mat(1, 2) = 0.106163;
  icp_mat(2, 0) = -0.048978;
  icp_mat(2, 1) = -0.099043;
  icp_mat(2, 2) = 0.993877;
  icp_mat(0, 3) = 0.149124; //
  icp_mat(1, 3) = 0.056666;
  icp_mat(2, 3) = 0.195112;


  ttt = icp_mat*init_mat;

  pcl::transformPointCloud (*input_pc, *transform_pc, ttt);
  cout << "ttt:" << ttt << endl;

  cv::Mat image_color;
  image_color = cv::imread(image_filename, cv::IMREAD_COLOR);

  // Read 3D points
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud=*transform_pc;
    
  std::vector<cv::Point3f> objectPoints;
  for (const auto& point : cloud) {
      objectPoints.push_back(cv::Point3f(point.x, point.y, point.z));
  }
  cout << "rMat:" << rMat << endl;
  cout << "tMat:" << tMat/1000 << endl;

  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rMat, tMat/1000, i_params.cameramat, i_params.distcoeff, projectedPoints);
  int count=0;
  for (auto &point : projectedPoints)
  {
    cv::circle(image_color, point, 2, CV_RGB(cloud[count].intensity, cloud[count].intensity, cloud[count].intensity), -1);
    //cv::circle(image_color, point, 2, CV_RGB(0,255,0), -1);
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
  init_mat.at<float>(0,0) = i_params.icp_init_rotmat[0][0];
  init_mat.at<float>(0,1) = i_params.icp_init_rotmat[0][1];
  init_mat.at<float>(0,2) = i_params.icp_init_rotmat[0][2];
  init_mat.at<float>(1,0) = i_params.icp_init_rotmat[1][0];
  init_mat.at<float>(1,1) = i_params.icp_init_rotmat[1][1];
  init_mat.at<float>(1,2) = i_params.icp_init_rotmat[1][2];
  init_mat.at<float>(2,0) = i_params.icp_init_rotmat[2][0];
  init_mat.at<float>(2,1) = i_params.icp_init_rotmat[2][1];
  init_mat.at<float>(2,2) = i_params.icp_init_rotmat[2][2];
  init_mat.at<float>(0,3) = i_params.icp_init_rotmat[3][0];//
  init_mat.at<float>(1,3) = i_params.icp_init_rotmat[3][1];
  init_mat.at<float>(2,3) = i_params.icp_init_rotmat[3][2];

  icp_mat.at<float>(0,0) = 0.983379;
  icp_mat.at<float>(0,1) = -0.178971;
  icp_mat.at<float>(0,2) = 0.030626;
  icp_mat.at<float>(1,0) = 0.174842;
  icp_mat.at<float>(1,1) = 0.978858;
  icp_mat.at<float>(1,2) = 0.106163;
  icp_mat.at<float>(2,0) = -0.048978;
  icp_mat.at<float>(2,1) = -0.099043;
  icp_mat.at<float>(2,2) = 0.993877;
  icp_mat.at<float>(0,3) = 0.149124; //
  icp_mat.at<float>(1,3) = 0.056666;
  icp_mat.at<float>(2,3) = 0.195112;

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
  int num_3d,num_2d;
  float error_mean = 0.0;
  {
    num_3d = 0;
    num_2d = 1;

    cv::Mat invR = extrinsic_matrix(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat invT = -invR * (extrinsic_matrix(cv::Rect(3, 0, 1, 3)));
    cv::Mat point(1, 3, CV_32F);
    point.at<float>(0) = cross_point[num_3d].x;
    point.at<float>(1) = cross_point[num_3d].y;
    point.at<float>(2) = cross_point[num_3d].z;

    point = point*invR.t()+ invT.t();
    float tmpx = point.at<float>(0)/point.at<float>(2);
    float tmpy = point.at<float>(1)/point.at<float>(2);
    float imagepoint_x = i_params.cameramat.at<double>(0, 0)*tmpx + i_params.cameramat.at<double>(0, 2);
    float imagepoint_y = i_params.cameramat.at<double>(1, 1)*tmpy + i_params.cameramat.at<double>(1, 2);

    float image_x = board_image_pixels[num_2d].x;
    float image_y = board_image_pixels[num_2d].y;

    float error = sqrt(pow(imagepoint_x-image_x,2) + pow(imagepoint_y-image_y,2));
    error_mean += error;
    cout << "error:" << error << endl;
    cout << "cross_point:" << cross_point[num_3d].x << "; board_image_pixels:" << board_image_pixels[num_2d].x << endl;
    cout << "imagepoint_x:" << imagepoint_x << "; imagepoint_y:" << imagepoint_y << endl;
  }
  {
    num_3d = 1;
    num_2d = 0;

    cv::Mat invR = extrinsic_matrix(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat invT = -invR * (extrinsic_matrix(cv::Rect(3, 0, 1, 3)));
    cv::Mat point(1, 3, CV_32F);
    point.at<float>(0) = cross_point[num_3d].x;
    point.at<float>(1) = cross_point[num_3d].y;
    point.at<float>(2) = cross_point[num_3d].z;

    point = point*invR.t()+ invT.t();
    float tmpx = point.at<float>(0)/point.at<float>(2);
    float tmpy = point.at<float>(1)/point.at<float>(2);
    float imagepoint_x = i_params.cameramat.at<double>(0, 0)*tmpx + i_params.cameramat.at<double>(0, 2);
    float imagepoint_y = i_params.cameramat.at<double>(1, 1)*tmpy + i_params.cameramat.at<double>(1, 2);

    float image_x = board_image_pixels[num_2d].x;
    float image_y = board_image_pixels[num_2d].y;

    float error = sqrt(pow(imagepoint_x-image_x,2) + pow(imagepoint_y-image_y,2));
    error_mean += error;
    cout << "error:" << error << endl;
    cout << "cross_point:" << cross_point[num_3d].x << "; board_image_pixels:" << board_image_pixels[num_2d].x << endl;
    cout << "imagepoint_x:" << imagepoint_x << "; imagepoint_y:" << imagepoint_y << endl;
  }
  {
    num_3d = 2;
    num_2d = 3;

    cv::Mat invR = extrinsic_matrix(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat invT = -invR * (extrinsic_matrix(cv::Rect(3, 0, 1, 3)));
    cv::Mat point(1, 3, CV_32F);
    point.at<float>(0) = cross_point[num_3d].x;
    point.at<float>(1) = cross_point[num_3d].y;
    point.at<float>(2) = cross_point[num_3d].z;

    point = point*invR.t()+ invT.t();
    float tmpx = point.at<float>(0)/point.at<float>(2);
    float tmpy = point.at<float>(1)/point.at<float>(2);
    float imagepoint_x = i_params.cameramat.at<double>(0, 0)*tmpx + i_params.cameramat.at<double>(0, 2);
    float imagepoint_y = i_params.cameramat.at<double>(1, 1)*tmpy + i_params.cameramat.at<double>(1, 2);

    float image_x = board_image_pixels[num_2d].x;
    float image_y = board_image_pixels[num_2d].y;

    float error = sqrt(pow(imagepoint_x-image_x,2) + pow(imagepoint_y-image_y,2));
    error_mean += error;
    cout << "error:" << error << endl;
    cout << "cross_point:" << cross_point[num_3d].x << "; board_image_pixels:" << board_image_pixels[num_2d].x << endl;
    cout << "imagepoint_x:" << imagepoint_x << "; imagepoint_y:" << imagepoint_y << endl;
  }
  {
    num_3d = 3;
    num_2d = 2;

    cv::Mat invR = extrinsic_matrix(cv::Rect(0, 0, 3, 3)).t();
    cv::Mat invT = -invR * (extrinsic_matrix(cv::Rect(3, 0, 1, 3)));
    cv::Mat point(1, 3, CV_32F);
    point.at<float>(0) = cross_point[num_3d].x;
    point.at<float>(1) = cross_point[num_3d].y;
    point.at<float>(2) = cross_point[num_3d].z;

    point = point*invR.t()+ invT.t();
    float tmpx = point.at<float>(0)/point.at<float>(2);
    float tmpy = point.at<float>(1)/point.at<float>(2);
    float imagepoint_x = i_params.cameramat.at<double>(0, 0)*tmpx + i_params.cameramat.at<double>(0, 2);
    float imagepoint_y = i_params.cameramat.at<double>(1, 1)*tmpy + i_params.cameramat.at<double>(1, 2);

    float image_x = board_image_pixels[num_2d].x;
    float image_y = board_image_pixels[num_2d].y;

    float error = sqrt(pow(imagepoint_x-image_x,2) + pow(imagepoint_y-image_y,2));
    error_mean += error;
    cout << "error:" << error << endl;
    cout << "cross_point:" << cross_point[num_3d].x << "; board_image_pixels:" << board_image_pixels[num_2d].x << endl;
    cout << "imagepoint_x:" << imagepoint_x << "; imagepoint_y:" << imagepoint_y << endl;
  }

  cout << "reprojection_error_mean:" << error_mean/4 << endl;
}

int main(int argc, char **argv)
{
  double line_arr[4][6];
  bool loadpcd_done = false;
  bool savefile_done = false;
  std::vector<cv::Point3d> cross_point;

  ros::init(argc, argv, "mypaper");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it(nh_);
  ros::Rate r(100);
  get_params();

  publisher_FullPcd = nh_.advertise<sensor_msgs::PointCloud2>("FullPcd", 0);
  publisher_PassfilteredPcd = nh_.advertise<sensor_msgs::PointCloud2>("PassfilteredPcd", 10);
  publisher_RansacPcd = nh_.advertise<sensor_msgs::PointCloud2>("RansacPcd", 10);
  publisher_ConvexHullPcd = nh_.advertise<sensor_msgs::PointCloud2>("ConvexhullPcd", 10);
  publisher_marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  image_publisher = it.advertise("camera_features", 1);
  pcd2image_publisher = it.advertise("pcd2image", 1);
  pcl::PointCloud<pcl::PointXYZI>::Ptr full_pcd(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr passthrough_pcd(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ransac_pcd(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr convexhull_pcd(new pcl::PointCloud<pcl::PointXYZI>);

  while (ros::ok())
  {
    if(!loadpcd_done)
      loadpcd_done = read_pcd(full_pcd);

    std::cout << "run" << std::endl;
    passthrough(full_pcd, passthrough_pcd);
    auto lidar_normal = ransac(passthrough_pcd, ransac_pcd);
    bound(ransac_pcd, convexhull_pcd);
    if (!savefile_done)
    {
      save_point(convexhull_pcd);
      savefile_done = true;
    }
    hough3dlines(line_arr);
    cross_point = find_crossline(line_arr);
    auto [corner_vectors, board_image_pixels, in_rMat, in_tMat] = findImgChessboardCorners();
    
    cv::Mat extrinsic_matrix = output_extrinsic(in_rMat, in_tMat);
    makeImageFromPointCloud(full_pcd, cross_point, in_rMat, in_tMat, extrinsic_matrix);
    reprojection_error(extrinsic_matrix, cross_point, board_image_pixels);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
