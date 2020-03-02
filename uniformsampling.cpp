//load library

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "No Point Cloud Data shoed. Please show source and target PCL data!" << std::endl;
    return (-1);
  }

  // Input
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile (argv[1], *cloud) < 0) // Soruce data is first
  {
    std::cerr << "Failed to read file." << std::endl;
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;

  return (0);
}

 
  
