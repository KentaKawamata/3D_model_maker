#ifndef REGI_PC_H
#define REGI_PC_H

#include <string>
#include <tuple>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

namespace registration {

    class registration_pc {

    private:

        using pc_fpfh_feature = std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr>;
        
        float voxel_size;
	    std::string modelname;
	    std::string cloudname;
	    std::string saveName;

        pcl::PointCloud<pcl::PointXYZ>::Ptr scene1_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene2_pc;
        pcl::visualization::PCLVisualizer::Ptr view_origin;
        pcl::visualization::PCLVisualizer::Ptr view_ICP;

        pc_fpfh_feature scene1_kp_fpfh;
        pc_fpfh_feature scene2_kp_fpfh;

        Eigen::Matrix4f RANSAC_R;
        Eigen::Matrix4f ICP_R_1;
        Eigen::Matrix4f ICP_R_2;

        void read_pointcloud(
            const std::string& filename,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud );
        boost::shared_ptr<pcl::visualization::PCLVisualizer> draw_pointcloud(
	        const std::string& window_name,
	        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene1,
	        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene2,
	        const Eigen::Matrix4f& trans );
        pc_fpfh_feature fpfh_search_fuature(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud );
        Eigen::Matrix4f RANSAC_global_registration(
            const pc_fpfh_feature& scene1,
            const pc_fpfh_feature& scene2 );
        Eigen::Matrix4f ICP_registration(
        	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene1,
        	const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene2,
        	const Eigen::Matrix4f& trans );
        void save_data(std::string& filename, Eigen::Matrix4f R);
        void print4x4Matrix(const Eigen::Matrix4f& matrix);
        void range_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud );
        void voxelization_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud );
        void filter();
        void registrate();

    public:
        
        int count;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        
        registration_pc(/* args */);
        ~registration_pc();
        void run();
        void registrate_for_ROS_to_PCL();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif //REGI_PC_H