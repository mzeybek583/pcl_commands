// Load library

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/harris_3d.h>

int
main(int argc, char** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Keypoints indices example application.\n");
    pcl::console::print_info ("Syntax is: %s <source-pcd-file>\n", argv[0]);
    return (1);
  }

  pcl::console::print_info ("Reading %s\n", argv[1]);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) // load the file
  {
    pcl::console::print_error ("Couldn't read file %s!\n", argv[1]);
    return (-1);
  }

  pcl::HarrisKeypoint3D <pcl::PointXYZRGB, pcl::PointXYZI> harris;
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
  harris.setNonMaxSupression (true);
  harris.setInputCloud (cloud);
  //detector.setThreshold (1);
  harris.setRadius (0.5);
  
pcl::StopWatch watch;
  harris.compute (*keypoints);
  pcl::console::print_highlight ("Detected %zd points in %lfs\n", keypoints->size (), watch.getTimeSeconds ());
  pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices ();
  if (!keypoints_indices->indices.empty ())
  {
    pcl::io::savePCDFile ("harris3d.pcd", *cloud, keypoints_indices->indices, true);
    pcl::console::print_info ("Saved keypoints to harris3d.pcd\n");
  }
  else
    pcl::console::print_warn ("Keypoints indices are empty!\n");
}
