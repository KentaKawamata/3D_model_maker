#ifndef CALC_AVERAGE_H
#define CALC_AVERAGE_H
#include <iostream>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "editCloud.hpp"
#include "octree.hpp"

class CalcAverage {

public:
    CalcAverage();
    ~CalcAverage();
    void run();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr sumCentroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aveCloud;
    float voxelSize;

private:

    pcl::PointCloud<pcl::PointXYZ>::Ptr samePoints;
    pcl::PointXYZ avePoint;
    double sum;

    double min_x;
    double min_y;
    double min_z;
    double max_x;
    double max_y;
    double max_z;

    double averageFilter(std::vector<int> list);
    void setList();
    void getMaxMin();
    void calcAverage();
    inline bool findIndex(int num, std::vector<int> index);

};

#endif //CALC_AVERAGE_H