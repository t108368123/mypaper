#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>		  
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
 
using namespace std;
#define PointType pcl::PointXYZI
typedef pcl::Normal NormalType;
 
int main(int argc,char **argv)
{
	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr cloud_1000(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr cloud_normal(new pcl::PointCloud<NormalType>());
 
	pcl::io::loadPCDFile(argv[1], *cloud);

	pcl::PointCloud<PointType> cb;
	PointType p;
	for (pcl::PointCloud<PointType>::const_iterator item = cloud->begin(); item != cloud->end(); item++){
	p.x = (double)item->x*1000;
	p.y = (double)item->y*1000;
	p.z = (double)item->z*1000;
	p.intensity = (double)item->intensity;
	cb.push_back(p);
	}
	cloud_1000 = cb.makeShared();
 
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_1000, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud_1000, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	//eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
 
	Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
	transform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
	transform.block<3, 1>(0, 3) = -1.0f * (transform.block<3,3>(0,0)) * (pcaCentroid.head<3>());// 
 
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud_1000, *transformedCloud, transform);
 
	std::cout << eigenValuesPCA << std::endl;
	std::cout << eigenVectorsPCA << std::endl;
	std::cout << "center:" << pcaCentroid << std::endl;
 
	//转换到原点时的主方向
	PointType o;
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
	PointType pcaZ;
	pcaZ.x = 1000 * pz(0);
	pcaZ.y = 1000 * pz(1);
	pcaZ.z = 1000 * pz(2);
	PointType pcaY;
	pcaY.x = 1000 * py(0);
	pcaY.y = 1000 * py(1);
	pcaY.z = 1000 * py(2);
	PointType pcaX;
	pcaX.x = 1000 * px(0);
	pcaX.y = 1000 * px(1);
	pcaX.z = 1000 * px(2);
 
	//初始位置时的主方向
	PointType c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	PointType pcZ;
	pcZ.x = 1000 * eigenVectorsPCA(0, 0) + c.x;
	pcZ.y = 1000 * eigenVectorsPCA(1, 0) + c.y;
	pcZ.z = 1000 * eigenVectorsPCA(2, 0) + c.z;
	PointType pcY;
	pcY.x = 1000 * eigenVectorsPCA(0, 1) + c.x;
	pcY.y = 1000 * eigenVectorsPCA(1, 1) + c.y;
	pcY.z = 1000 * eigenVectorsPCA(2, 1) + c.z;
	PointType pcX;
	pcX.x = 1000 * eigenVectorsPCA(0, 2) + c.x;
	pcX.y = 1000 * eigenVectorsPCA(1, 2) + c.y;
	pcX.z = 1000 * eigenVectorsPCA(2, 2) + c.z;

	std::cout << "vx:" << eigenVectorsPCA(0, 0) << " vy:" << eigenVectorsPCA(1, 0) << " vz:" << eigenVectorsPCA(2, 0) << std::endl;

       //visualization
	pcl::visualization::PCLVisualizer viewer;
	pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud_1000,255, 0, 0); 
	pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); 
	viewer.addPointCloud(cloud_1000,color_handler,"cloud");
	viewer.addPointCloud(transformedCloud,tc_handler,"transformCloud");
	
	viewer.addArrow(pcaZ, o, 0.0, 0.0, 1.0, false, "arrow_Z");
	viewer.addArrow(pcaY, o, 0.0, 1.0, 0.0, false, "arrow_Y");
	viewer.addArrow(pcaX, o, 1.0, 0.0, 0.0, false, "arrow_X");
 
	viewer.addArrow(pcZ, c, 0.0, 0.0, 1.0, false, "arrow_z");
	viewer.addArrow(pcY, c, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(pcX, c, 1.0, 0.0, 0.0, false, "arrow_x");
 
 
	viewer.addCoordinateSystem(100);
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
 
	}
 
	return 0;
}
