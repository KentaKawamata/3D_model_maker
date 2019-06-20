#ifndef GET_FRAME_H
#define GET_FRAME_H

#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "editCloud.hpp"
#include "octree.hpp"
#include "calcAverage.hpp"

class GetFrame {

public:
    GetFrame();
    ~GetFrame();
    void run();

private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sumCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aveCloud;
    pcl::PointXYZ avePoint;
    double reso;
    double resolution;
    int number;
    float voxelSize;
    //double sum;

    std::string filename;
    std::string output_names;
    std::string rote;
    std::string saveDir;

    void averagePointCloud();
    
    EditCloud *edit;
    OctreeViewer *octree;
    CalcAverage *calcAve;
};

#endif //GET_FRAME_H