#ifndef EDIT_CLOUD_H
#define EDIT_CLOUD_H

#include <pcl/point_types.h>
#include <pcl/io/auto_io.h>
#include <Eigen/Core>
#include <Eigen/LU>

class EditCloud
{
public:
    EditCloud();
    ~EditCloud();
    void filter();

    pcl::PointCloud<pcl::PointXYZ>::Ptr over_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr under_cloud;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

private:

    void smooth(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void rangeFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void outline(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
};

#endif //EDIT_CLOUD_H