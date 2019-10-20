

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>


int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target2 (new pcl::PointCloud<pcl::PointXYZ>);

  // Read Cloud SourceIn data
pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("source.pcd", *cloud_source);
  reader.read<pcl::PointXYZ> ("target.pcd", *cloud_target);
if(pcl::io::loadPCDFile<pcl::PointXYZ> ("source.pcd", *cloud_source) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
if(pcl::io::loadPCDFile<pcl::PointXYZ> ("target.pcd", *cloud_target) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }
std::cerr << "Source Cloud: " << std::endl;
  std::cerr << *cloud_source << std::endl;
std::cerr << "Target Cloud: " << std::endl;
  std::cerr << *cloud_target << std::endl;
  

//Voxel Grid
// Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vox;
  vox.setInputCloud (cloud_source);
  vox.setLeafSize (0.2f, 0.2f, 0.2f);
  vox.filter (*cloud_source2);
  vox.setInputCloud (cloud_target);
  vox.setLeafSize (0.2f, 0.2f, 0.2f);
  vox.filter (*cloud_target2);

 

//ICP
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_source2);
  icp.setInputTarget(cloud_target2);
  icp.setMaxCorrespondenceDistance (1);
  icp.setMaximumIterations (100);
  icp.setTransformationEpsilon (1e-5);
  icp.setEuclideanFitnessEpsilon (0.001);
  //icp.setRANSACOutlierRejectionThreshold(0.01);	

 pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
//Transform Parameters
  Eigen::Affine3f transform;
    transform = icp.getFinalTransformation();
  pcl::transformPointCloud(*cloud_source, Final, transform);
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::io::savePCDFile ("Final.pcd", Final, true);


 return (0);
}
