#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

void
VoxelGridFilter (string input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (input.c_str(), *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")."<<endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZI> vox;
  vox.setInputCloud (cloud);
  vox.setLeafSize (0.05f, 0.05f, 0.05f);
  vox.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<<endl;

  pcl::PCDWriter writer;
  writer.write(input.c_str(),*cloud_filtered);

}