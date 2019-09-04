#ifndef EDIT_CLOUD_H
#define EDIT_CLOUD_H

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

class EditCloud
{
public:
    EditCloud();
    ~EditCloud();
    void filter();
    void out_filter();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

private:

    void smooth();
    void rangeFilter();
    void outline();
};

#endif //EDIT_CLOUD_H