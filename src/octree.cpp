#include <vector>
#include <pcl/common/time.h>
#include "octree.hpp"

OctreeViewer::OctreeViewer (std::string &filename, double reso) :
    viz (new pcl::visualization::PCLVisualizer ("Octree visualizator")),
    cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    displayCloud (new pcl::PointCloud<pcl::PointXYZ>()),
    cloudVoxel (new pcl::PointCloud<pcl::PointXYZ>()),
    octree (new pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> (reso)),
    resolution (reso),
    wireframe (true),
    show_cubes_ (true),
    show_centroids_ (false),
    show_original_points_ (false),
    point_size_ (1.0),
    key(false)
{
}

OctreeViewer::~OctreeViewer(){
//    clearOctree();
}

/* \brief Graphic loop for the viewer
 *
 */
void OctreeViewer::run(){ 

    pcl::StopWatch time;
    time.reset();
    octree->setInputCloud(cloud);
    //update bounding box automatically
    octree->defineBoundingBox();
    // add points from cloud to octree
    octree->addPointsFromInputCloud();
    //show octree at default depth

    //set current level to half the maximum one
    displayedDepth = static_cast<int> (floor (octree->getTreeDepth() / 2.0));
    std::cout << "octree : " << displayedDepth << std::endl;
    if (displayedDepth == 0){
        displayedDepth = 1;
    }

    extractPointsAtLevel(displayedDepth);
    std::cout << "time of octree : " << time.getTimeSeconds() << std::endl;
}

void OctreeViewer::clearOctree(){

    octree->deleteTree();
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

/* \brief Extracts all the points of depth = level from the octree
 */
void OctreeViewer::extractPointsAtLevel(int depth){

    cloudVoxel.reset (new pcl::PointCloud<pcl::PointXYZ>);
    displayCloud.reset (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ pt_voxel_center;
    pcl::PointXYZ pt_centroid;
    std::cout << "===== Extracting data at depth " << depth << "... " << std::endl;
    double start = pcl::getTime ();

    for (pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::FixedDepthIterator tree_it = octree->fixed_depth_begin(depth);
         tree_it != octree->fixed_depth_end();
         ++tree_it)
    {
      // Compute the point at the center of the voxel which represents the current OctreeNode
      Eigen::Vector3f voxel_min, voxel_max;
      octree->getVoxelBounds(tree_it, voxel_min, voxel_max);

      pt_voxel_center.x = (voxel_min.x () + voxel_max.x ()) / 2.0f;
      pt_voxel_center.y = (voxel_min.y () + voxel_max.y ()) / 2.0f;
      pt_voxel_center.z = (voxel_min.z () + voxel_max.z ()) / 2.0f;
      cloudVoxel->points.push_back (pt_voxel_center);

      // If the asked depth is the depth of the octree, retrieve the centroid at this LeafNode
      if (octree->getTreeDepth () == (unsigned int) depth)
      {
        pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode* container = static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::LeafNode*> (tree_it.getCurrentOctreeNode ());

        container->getContainer ().getCentroid (pt_centroid);
      }
      // Else, compute the centroid of the LeafNode under the current BranchNode
      else {
        // Retrieve every centroid under the current BranchNode
        pcl::octree::OctreeKey dummy_key;
        pcl::PointCloud<pcl::PointXYZ>::VectorType voxelCentroids;
        octree->getVoxelCentroidsRecursive(static_cast<pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ>::BranchNode*> (*tree_it), dummy_key, voxelCentroids);

        // Iterate over the leafs to compute the centroid of all of them
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        for (size_t j = 0; j < voxelCentroids.size (); ++j){
            centroid.add(voxelCentroids[j]);
        }
        centroid.get(pt_centroid);
      }

      displayCloud->points.push_back(pt_centroid);
    }

    //ボクセル1辺の長さを算出
    Eigen::Vector3f voxel_min, voxel_max;
    octree->getVoxelBounds(octree->fixed_depth_begin(depth), voxel_min, voxel_max);
    voxelSize =  voxel_max.x()-voxel_min.x(); 
    std::cout << "voxelSize : " << voxelSize << std::endl;


    double end = pcl::getTime ();
    printf("%lu pts, %.4gs. %.4gs./pt. =====\n", displayCloud->points.size (), end - start,
           (end - start) / static_cast<double> (displayCloud->points.size ()));
}