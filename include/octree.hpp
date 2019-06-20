#ifndef Octree_H
#define Octree_H

#include <pcl/io/auto_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/common/common.h>

#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/filter.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkCubeSource.h>
#include <vtkCleanPolyData.h>

class OctreeViewer
{
public:
  OctreeViewer(std::string &filename, double reso);
  ~OctreeViewer();
  void run();
  void clearView();
  void clearOctree();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //displayed_cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr displayCloud;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viz;
  pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::Ptr octree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
  float voxelSize;
  bool key;

private:

  //pcl::PointCloud<pcl::PointXYZ>::Ptr xyz;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudVoxel;
  //level
  int displayedDepth;
  //bool to decide what should be display
  bool wireframe;
  bool show_cubes_, show_centroids_, show_original_points_;
  float point_size_;
  double resolution;

  // \brief Callback to interact with the keyboard
  void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void *);

  bool loadCloud(std::string &filename);
  // \brief Helper function that draw info for the user on the viewer
  void showLegend();
  // \brief Visual update. Create visualizations and add them to the viewer
  void update();
  // \brief display octree cubes via vtk-functions
  void showCubes(double voxelSideLen);
  // \brief Extracts all the points of depth = level from the octree
  void extractPointsAtLevel(int depth);
  // \brief Helper function to increase the octree display level by one
  bool IncrementLevel();
  // \brief Helper function to decrease the octree display level by one
  bool DecrementLevel();
};

#endif //FOO_H