#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>

#include "./../include/make_ply.hpp"

ROStoPCL::ROStoPCL(ros::NodeHandle &nh) : 
    over_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
    under_cloud_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
    R (Eigen::Matrix4d::Identity()),
    under_R (Eigen::Matrix4d::Identity()),
    over_cloud_frame ("/cam_1/depth/color/points"),
    under_cloud_frame ("/cam_2/depth/color/points"),
    count (0),
    over_cloud_sub(nh.subscribe(over_cloud_frame, 1, &ROStoPCL::getOverPointCloud_callback, this)),
    under_cloud_sub(nh.subscribe(under_cloud_frame, 1, &ROStoPCL::getUnderPointCloud_callback, this))
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
    ros::param::set("/pc_tf/pub_header_id", "track_pose_frame");
    ros::param::set("/pc_tf/pub_child_id", "cam_1_link");

    ros::param::get("/pc_tf/lis_header_id", lis_header_id);
    ros::param::get("/pc_tf/lis_child_id", lis_child_id);
    ros::param::get("/pc_tf/pub_header_id", pub_header_id);
    ros::param::get("/pc_tf/pub_child_id", pub_child_id);
}

void ROStoPCL::check_params() {
    
    if(!ros::param::has("/pc_tf/lis_header_id")){
        throw std::runtime_error("COULD NOT FIND PARAMS OF FRAME");
    
    } else if(!ros::param::has("/pc_tf/lis_child_id")){
        throw std::runtime_error("COULD NOT FIND PARAMS OF FRAME");
    
    } else if(!ros::param::has("/pc_tf/pub_header_id")){
        throw std::runtime_error("COULD NOT FIND PARAMS OF FRAME");
    
    } else if(!ros::param::has("/pc_tf/pub_child_id")){
        throw std::runtime_error("COULD NOT FIND PARAMS OF FRAME");
    }
}

void ROStoPCL::get_params() {

    try {

        check_params();
        ros::param::get("/pc_tf/lis_header_id", lis_header_id);
        ros::param::get("/pc_tf/lis_child_id", lis_child_id);
        ros::param::get("/pc_tf/pub_header_id", pub_header_id);
        ros::param::get("/pc_tf/pub_child_id", pub_child_id);

    } catch(std::exception& ex) {
        ROS_ERROR_STREAM("=== " << ex.what() << " ===");
        set_params();
    } 
}

void ROStoPCL::getOverPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs) {

    pcl::fromROSMsg(*cloud_msgs, *over_cloud_pcl);
    ROS_INFO("GET POINTCLOUD   === OVER ===");
}

void ROStoPCL::getUnderPointCloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msgs) {

    pcl::fromROSMsg(*cloud_msgs, *under_cloud_pcl);
    ROS_INFO("GET POINTCLOUD   === UNDER ===");
}

void ROStoPCL::addPointCloud() {

    *over_cloud_pcl += *under_cloud_pcl;
}

void ROStoPCL::savePointcloud() {

    std::string num = std::to_string(count);
    std::string filename = "/root/ply_data/" + num + ".ply"; 
    ROS_INFO_STREAM("FILE NAME : " + filename);
    pcl::io::savePLYFileASCII(filename, *over_cloud_pcl);
}

void ROStoPCL::transformPointCloud() {

    pcl::copyPointCloud(*over_cloud_pcl, *(edit->over_cloud));
    pcl::copyPointCloud(*under_cloud_pcl, *(edit->under_cloud));
    edit->filter();

    pcl::copyPointCloud(*(edit->over_cloud), *over_cloud_pcl);
    pcl::copyPointCloud(*(edit->under_cloud), *under_cloud_pcl);

    pcl::transformPointCloud(*over_cloud_pcl, *over_cloud_pcl, R); 
    pcl::transformPointCloud(*under_cloud_pcl, *under_cloud_pcl, under_R); 

    addPointCloud();
    savePointcloud();
}

void ROStoPCL::quaternion_to_euler(geometry_msgs::TransformStamped &ts) {

    translation.setValue(ts.transform.translation.x,
                         ts.transform.translation.y,
                         ts.transform.translation.z);

    /*************************************************************************
     * < Must not toutch tpclZ, tpclY and tpclX !!!!!      >
     * < tpclZ, tpclY and tpclX are completely correct !!! >
     *         \  ^__^
     *          \ (oo)\______
     *           (__)\    )\/\
     *              ||----w |
     *              ||    ||
     ***********************************************************************/
    //////////////////////////////////
    rotevec->tpclZ =  translation.getX();
    rotevec->tpclY = -translation.getZ();
    rotevec->tpclX = -translation.getY();
    //////////////////////////////////

    rotation.setValue(ts.transform.rotation.x,
                      ts.transform.rotation.y,
                      ts.transform.rotation.z,
                      ts.transform.rotation.w);

    tf2::Matrix3x3 m(rotation);
    m.getRPY(RrosX, RrosY, RrosZ);

    /*************************************************************************
     * Must not toutch roll, pitch and yaw !!!!!
     * Roll, pitch and yaw are completely correct !!!
     * 
     *    　　　　　　　_,ｨ￣￣￣￣ 7ｰ ､
     *　　 　 _,. -'" .| ' ''　　'　　''|l　　｀ ｰｫ､_
     *　　　/ , 　,,　|　'　'　　''　'|l　'　　'' '|;;;;;;l 　＿＿
     *　　 /　　　　| ::　　　　　 |l 　 　 　.|;;;;;;|￣r--ｧ￣､＼__
     *　　/___ , , , ,| ::　　　　　 |　　,　　,,|;;;;;{ --'-=（=　乂 rﾄ}
     *　 ﾑ==ｧ--､ｰ､- - - - ァ=======ｲ{ZZZZZZｲ三ｴﾆｲ==-
     *　　 {Yヽ0ﾉ} } ＼!!!＿/;;;;;{：{0} }};;;;;ｲ}￣
     *　　　 | | | |-t==t-''￣￣~ | | | l|i:::i:l|
     *　　　.|_U_l|　|:i::i:l| 　 　 　　ﾞ|_U_l|::::::l|
     *　 　｜l　ｌ|　 | ::::l|　 　 　 　 | l　l|::::::l|
     *　 　 |r-､l| 　 Y=Y}　 　 　　 Y^ 米o;}}
     *　 　 ﾄ-.イ 　　ﾄ-ィ| 　 　 　 ｀ﾄ-ィ|Y;/
     *　 　,|　 ｌ| 　 　 |　 l|　　 　 　 ,|　i, |/
     *　　(0)(0) 　　　!n人9､　　／ｨ(0)(0)}
     *　/ri￣|^}　　　{ T￣|^}／Ｘ:;; {T==ｲｺ
     *　|二=--{､　　|三三三}＼/三r-----{
     *　｀ーー‐'　　　￣￣￣　　｀~└─―┘
     ***********************************************************************/
    ////////////////////////////////////////////////////////////
    rotevec->roll  = -RrosX;
    rotevec->pitch =  RrosY;
    rotevec->yaw   =  RrosZ;
    rotevec->under_pitch = rotevec->pitch + (40.0*M_PI/180);
    ////////////////////////////////////////////////////////////

    //ROS_INFO("ROLL : %f", (float)roll/M_PI*180);
}

void ROStoPCL::quaternion_to_vector(geometry_msgs::TransformStamped &ts) {
    
    quaternion_to_euler(ts);
    rotevec->transformPointCloud();

    R = rotevec->R;
    under_R = rotevec->under_R;
}

void ROStoPCL::run() {

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(5.0);
        
    geometry_msgs::TransformStamped ts;

    while(ros::ok()){

        try{
            ts = tfBuffer.lookupTransform(lis_header_id, lis_child_id, ros::Time());

        } catch(tf2::TransformException &ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        over_pc_num = over_cloud_pcl->size();
        under_pc_num = under_cloud_pcl->size();

        if(over_pc_num>0 && under_pc_num>0) {
            
            quaternion_to_vector(ts);
            transformPointCloud();
            ROS_INFO("=====  GET TF  =====");
            count++;

        } else {
            ROS_INFO("NO POINTCLOUD DATA");
        }
        ros::spinOnce();
        rate.sleep();
    }    
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "get_pointcloud");
    ros::NodeHandle nh;

    ROStoPCL *get_pcl;
    get_pcl = new ROStoPCL(nh);
    get_pcl->run();

    delete get_pcl; 
   
    return 0;
}
