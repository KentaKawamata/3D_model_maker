#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "./../include/make_ply.hpp"

ROStoPCL::ROStoPCL(ros::NodeHandle &nh) : 
    cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
    R (Eigen::Matrix4d::Identity()),
    cloud_frame ("/cam_1/depth/color/points"),
    count (0),
    filename ("/root/datas/model_for_RT/3d_model.ply"),
    cloud_sub(nh.subscribe(cloud_frame, 1, &ROStoPCL::getOverPointCloud_callback, this))
{
    get_params();
    rotevec = new GetRotationVector();
    edit = new EditCloud();
}

ROStoPCL::~ROStoPCL() {
    delete rotevec; 
    delete edit;
}

void ROStoPCL::set_params() {

    ros::param::set("/pc_tf/lis_header_id", "track_odom_frame");
    ros::param::set("/pc_tf/lis_child_id", "track_pose_frame");

    ros::param::get("/pc_tf/lis_header_id", lis_header_id);
    ros::param::get("/pc_tf/lis_child_id", lis_child_id);
}

void ROStoPCL::check_params() {
    
    if(!ros::param::has("/pc_tf/lis_header_id")){
        throw std::runtime_error("COULD NOT FIND PARAMS OF FRAME");
    
    } else if(!ros::param::has("/pc_tf/lis_child_id")){
        throw std::runtime_error("COULD NOT FIND PARAMS OF FRAME");
    }    
}

void ROStoPCL::get_params() {

    try
    {
        check_params();
        ros::param::get("/pc_tf/lis_header_id", lis_header_id);
        ros::param::get("/pc_tf/lis_child_id", lis_child_id);

    }
    catch(std::exception& ex)
    {
        ROS_ERROR_STREAM("=== " << ex.what() << " ===");
        set_params();
    } 
}

void ROStoPCL::getOverPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs) {

    pcl::fromROSMsg(*cloud_msgs, *cloud_pcl);
    ROS_INFO("GET POINTCLOUD ");
}

void ROStoPCL::savePointcloud() {

    if (count > 0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::io::loadPLYFile(filename, *total_cloud);

        *total_cloud += *cloud_pcl;
    
        pcl::copyPointCloud(*total_cloud, *(edit->cloud));
        edit->out_filter();
        pcl::copyPointCloud(*(edit->cloud), *total_cloud);

        ROS_INFO_STREAM("SAVE FILE NAME : " + filename);
        pcl::io::savePLYFileASCII(filename, *total_cloud);
    }
    else 
    {
        ROS_INFO_STREAM("SAVE FILE NAME : " + filename);
        pcl::io::savePLYFileASCII(filename, *cloud_pcl);
    }
}

void ROStoPCL::transformPointCloud() {

    pcl::copyPointCloud(*cloud_pcl, *(edit->cloud));
    edit->filter();
    pcl::copyPointCloud(*(edit->cloud), *cloud_pcl);
    pcl::transformPointCloud(*cloud_pcl, *cloud_pcl, R); 

    savePointcloud();
}

void ROStoPCL::quaternion_to_euler(geometry_msgs::TransformStamped &ts) {

    translation.setValue(ts.transform.translation.x,
                         ts.transform.translation.y,
                         ts.transform.translation.z);

    rotevec->tpclZ =  translation.getX();
    rotevec->tpclY = -translation.getZ();
    rotevec->tpclX = -translation.getY();

    rotation.setValue(ts.transform.rotation.x,
                      ts.transform.rotation.y,
                      ts.transform.rotation.z,
                      ts.transform.rotation.w);

    tf2::Matrix3x3 m(rotation);
    m.getRPY(RrosX, RrosY, RrosZ);

    rotevec->roll  = RrosX;
    rotevec->pitch = -RrosY;
    rotevec->yaw   = -RrosZ;

    ROS_INFO("ROLL : %f", (float)-RrosX);
    ROS_INFO("PITCH : %f", (float)RrosY);
    ROS_INFO("YAW : %f", (float)RrosZ);
}

void ROStoPCL::quaternion_to_vector(geometry_msgs::TransformStamped &ts) {
    
    quaternion_to_euler(ts);
    rotevec->transformPointCloud();

    R = rotevec->R;
}

void ROStoPCL::run() {

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(0.5);
        
    while(ros::ok()){

        geometry_msgs::TransformStamped ts;
        try{
            ts = tfBuffer.lookupTransform(lis_header_id, lis_child_id, ros::Time(0));

        } catch(tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            ros::spinOnce();
            continue;
        }


        if(cloud_pcl->size() > 0) 
        {
            quaternion_to_vector(ts);
            transformPointCloud();
            count++;
        } 
        else 
        {
            ROS_INFO("NO POINTCLOUD DATA");
        }

        rate.sleep();
        ros::spinOnce();
    }    
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "model_maker");
    ros::NodeHandle nh;

    ROStoPCL *get_pcl;
    get_pcl = new ROStoPCL(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
