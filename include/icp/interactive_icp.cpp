#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("%6.6f,%6.6f,%6.6f,%6.6f,%6.6f,%6.6f,%6.6f,%6.6f,%6.6f\n", matrix (0, 0), matrix (0, 1), matrix (0, 2), matrix (1, 0), matrix (1, 1), matrix (1, 2), matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("%6.6f,%6.6f,%6.6f\n", matrix (0, 3)/1000, matrix (1, 3)/1000, matrix (2, 3)/1000);
}

int main (int argc, char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in1 (new PointCloudT);
  PointCloudT::Ptr cloud_in2 (new PointCloudT);
  PointCloudT::Ptr cloud_tr (new PointCloudT);
  PointCloudT::Ptr cloud_in1_filtered (new PointCloudT);  
  PointCloudT::Ptr cloud_in2_filtered (new PointCloudT);
  PointCloudT::Ptr cloud_icp_filtered (new PointCloudT);

  // Checking program arguments
  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    PCL_ERROR ("Provide one ply file.\n");
    return (-1);
  }

  int iterations = 1;  // Default number of ICP iterations
  if (argc > 2)
  {
    // If the user passed the number of iteration as an argument
    iterations = atoi (argv[3]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1\n");
      return (-1);
    }
  }

  pcl::console::TicToc time;
  time.tic ();
  if (pcl::io::loadPCDFile (argv[1], *cloud_in1) < 0 || pcl::io::loadPCDFile (argv[2], *cloud_in2) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in1->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  // voxel filtered
  pcl::VoxelGrid<PointT> voxel;
  voxel.setInputCloud (cloud_in1);
  voxel.setLeafSize (0.1f, 0.1f, 0.1f);
  voxel.filter (*cloud_in1_filtered);
  /*voxel.setInputCloud (cloud_in2);
  voxel.setLeafSize (0.5f, 0.5f, 0.5f);
  voxel.filter (*cloud_in2_filtered);*/

  Eigen::Matrix4d ttt = Eigen::Matrix4d::Identity ();

  //順_mypaper
  ttt (0, 0) = 0;
  ttt (0, 1) = -0.7071067;
  ttt (0, 2) = -0.7071067;
  ttt (1, 0) = 0;
  ttt (1, 1) = -0.7071067;
  ttt (1, 2) = 0.7071067;
  ttt (2, 0) = -1;
  ttt (2, 1) = 0;
  ttt (2, 2) = 0;
  ttt (0, 3) = -500; //
  ttt (1, 3) = -500;
  ttt (2, 3) = 4000;

  //逆_mypaper
  /*ttt (0, 0) = 0;
  ttt (0, 1) = 0.7071067;
  ttt (0, 2) = -0.7071067;
  ttt (1, 0) = 0;
  ttt (1, 1) = -0.7071067;
  ttt (1, 2) = -0.7071067;
  ttt (2, 0) = -1;
  ttt (2, 1) = 0;
  ttt (2, 2) = 0;
  ttt (0, 3) = -1000; //
  ttt (1, 3) = 500;
  ttt (2, 3) = 3800;*/

  //逆_360_a1
  /*ttt (0, 0) = 0;
  ttt (0, 1) = 0.7071067;
  ttt (0, 2) = -0.7071067;
  ttt (1, 0) = 0;
  ttt (1, 1) = -0.7071067;
  ttt (1, 2) = -0.7071067;
  ttt (2, 0) = -1;
  ttt (2, 1) = 0;
  ttt (2, 2) = 0;
  ttt (0, 3) = 0; //
  ttt (1, 3) = -300;
  ttt (2, 3) = 4000;*/

  //逆_360_a2
  /*ttt (0, 0) = 0.6124;
  ttt (0, 1) = 0.3536;
  ttt (0, 2) = -0.7071;
  ttt (1, 0) = -0.6124;
  ttt (1, 1) = -0.3536;
  ttt (1, 2) = -0.7071;
  ttt (2, 0) = -0.5000;
  ttt (2, 1) = 0.8660;
  ttt (2, 2) = 0;
  ttt (0, 3) = 0; //
  ttt (1, 3) = -300;
  ttt (2, 3) = 4000;*/

  //逆_360_a3
  /*ttt (0, 0) = 0.7071067;
  ttt (0, 1) = 0;
  ttt (0, 2) = -0.7071067;
  ttt (1, 0) = -0.7071067;
  ttt (1, 1) = 0;
  ttt (1, 2) = -0.7071067;
  ttt (2, 0) = 0;
  ttt (2, 1) = 1;
  ttt (2, 2) = 0;
  ttt (0, 3) = 1000; //
  ttt (1, 3) = -1000;
  ttt (2, 3) = 3500;*/

  //逆_360_a4
  /*ttt (0, 0) = 0.0000;
  ttt (0, 1) = -0.7071;
  ttt (0, 2) = -0.7071;
  ttt (1, 0) = -0.0000;
  ttt (1, 1) = 0.7071;
  ttt (1, 2) = -0.7071;
  ttt (2, 0) = 1.000;
  ttt (2, 1) = 0.000;
  ttt (2, 2) = 0;
  ttt (0, 3) = -300; //
  ttt (1, 3) = 300;
  ttt (2, 3) = 4000;*/

  //逆_360_a5
  /*ttt (0, 0) = -0.6124;
  ttt (0, 1) = -0.3536;
  ttt (0, 2) = -0.7071;
  ttt (1, 0) = 0.6124;
  ttt (1, 1) = 0.3536;
  ttt (1, 2) = -0.7071;
  ttt (2, 0) = 0.5000;
  ttt (2, 1) = -0.8660;
  ttt (2, 2) = 0;
  ttt (0, 3) = 0; //
  ttt (1, 3) = -300;
  ttt (2, 3) = 4000;*/

  //逆_360_a6
  /*ttt (0, 0) = -0.6124;
  ttt (0, 1) = 0.3536;
  ttt (0, 2) = -0.7071;
  ttt (1, 0) = 0.6124;
  ttt (1, 1) = -0.3536;
  ttt (1, 2) = -0.7071;
  ttt (2, 0) = -0.5000;
  ttt (2, 1) = -0.8660;
  ttt (2, 2) = 0;
  ttt (0, 3) = -300; //
  ttt (1, 3) = 0;
  ttt (2, 3) = 4000;*/

  pcl::transformPointCloud (*cloud_in2, *cloud_in2_filtered, ttt);

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // The Iterative Closest Point algorithm
  time.tic ();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  //icp.setTransformationEpsilon(1e-6);
  //icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setInputSource (cloud_in2_filtered);
  icp.setInputTarget (cloud_in1_filtered);
  icp.align (*cloud_icp_filtered);
  //icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");

  // Create two vertically separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer.addCoordinateSystem(1000,v1);   
  viewer.addCoordinateSystem(1000,v2);   

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in1_filtered, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  viewer.addPointCloud (cloud_in1_filtered, cloud_in_color_h, "cloud_in_v1", v1);
  viewer.addPointCloud (cloud_in1_filtered, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_icp_filtered, 20, 180, 20);
  viewer.addPointCloud (cloud_icp_filtered, cloud_tr_color_h, "cloud_tr_v1", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_in2_filtered, 180, 20, 20);
  viewer.addPointCloud (cloud_in2_filtered, cloud_icp_color_h, "cloud_icp_v2", v2);

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_in_v1");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_in_v2");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_tr_v1");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud_icp_v2");

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

  // Set background color
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5000.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
  }

  return (0);
}
