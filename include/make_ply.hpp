#ifndef MAKE_PLY_H
#define MAKE_PLY_H

#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "./../include/getRotationVector.hpp"
#include "./../include/editCloud.hpp"

class ROStoPCL {

private:

    double RrosX;
    double RrosY;
    double RrosZ;

    int count;
    int over_pc_num;
    int under_pc_num;

    uint32_t confidence;

    char character;

    std::string lis_header_id;
    std::string lis_child_id;

    std::string lis_conf;
    std::string over_cloud_frame;
    std::string under_cloud_frame;

    tf2::Vector3 translation;
    tf2::Quaternion rotation;

    ros::Subscriber conf_sub;
    ros::Subscriber over_cloud_sub;
    ros::Subscriber under_cloud_sub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr over_cloud_pcl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr under_cloud_pcl;

    Eigen::Matrix3d R3d;
    Eigen::Matrix3d under_R3d;
    Eigen::Matrix4d R;
    Eigen::Matrix4d under_R;

    void check_params();
    void set_params();
    void get_params();

    void getOverPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs);
    void getUnderPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs);
    void getConfidence_callback(const std_msgs::UInt32& msg);

    void getCharacter();

    void addPointCloud();
    void savePointcloud();

    void transformPointCloud();

    void quaternion_to_euler(geometry_msgs::TransformStamped &ts); 
    void quaternion_to_vector(geometry_msgs::TransformStamped &ts);

public:

    ROStoPCL(ros::NodeHandle &nh);
    ~ROStoPCL();
    void run();

    GetRotationVector *rotevec;
    EditCloud *edit; 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //MAKE_PLY_H