#include <string>
#include <tuple>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>

#include <pcl/filters/passthrough.h>

#include "loadVoxelSize.hpp"
#include "./../include/registration_pc.hpp"

namespace registration {

    using pc_fpfh_feature = std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::FPFHSignature33>::Ptr>;
	
	registration_pc::registration_pc() :
		scene1_pc (new pcl::PointCloud<pcl::PointXYZ>() ),
		scene2_pc (new pcl::PointCloud<pcl::PointXYZ>() ),
		RANSAC_R (Eigen::Matrix4f::Identity() ),
		ICP_R_1 (Eigen::Matrix4f::Identity() ),
		ICP_R_2 (Eigen::Matrix4f::Identity() )
		//view_origin (new pcl::visualization::PCLVisualizer() ),
		//view_ICP (new pcl::visualization::PCLVisualizer() )
	{
		registration::loadParams(&voxel_size, cloudname, modelname, saveName);
	}

	registration_pc::~registration_pc()
	{

	}

    void registration_pc::read_pointcloud(
		const std::string& filename,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
	{
		const int retval = pcl::io::loadPLYFile(filename, *cloud);
		if (retval == -1 || cloud->size() <= 0)
		{
			PCL_ERROR("File load error.");
			exit(-1);
		}	
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> registration_pc::draw_pointcloud(
		const std::string& window_name,
	    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene1,
	    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene2,
	    const Eigen::Matrix4f& trans )
	{
	
		const pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer(window_name));
		viewer->setBackgroundColor(0.0, 0.0, 0.0);

		const auto tmp1_pc = scene1->makeShared();
		const auto tmp2_pc = scene2->makeShared();

		pcl::transformPointCloud(*tmp1_pc, *tmp1_pc, trans);

		const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color1(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(tmp1_pc, 1.0*255, 0.706 * 255, 0.0 * 255));
		const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color2(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(tmp2_pc, 0.0 * 255, 0.651 * 255, 0.929 * 255));
		viewer->addPointCloud<pcl::PointXYZ>(tmp1_pc, *color1, "scene1");
		viewer->addPointCloud<pcl::PointXYZ>(tmp2_pc, *color2, "scene2");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "scene1");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "scene2");

		return viewer;
	}

	pc_fpfh_feature registration_pc::fpfh_search_fuature(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
	{

		const pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		const boost::shared_ptr<pcl::VoxelGrid<pcl::PointXYZ>> sor(new pcl::VoxelGrid<pcl::PointXYZ>); // なぜかPtrがprotectedなので
		sor->setInputCloud(pointcloud);
		sor->setLeafSize(voxel_size, voxel_size, voxel_size);
		sor->filter(*keypoints);

		const float radius_normal = voxel_size * 2.0;
		const auto view_point = pcl::PointXYZ(0.0, 5.0, 5.0);

		const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		const pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr ne(new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>);
		const pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		ne->setInputCloud(pointcloud);
		ne->setRadiusSearch(radius_normal);
		ne->setSearchMethod(kdtree);
		ne->setViewPoint(view_point.x, view_point.y, view_point.z);
		ne->compute(*normals);

		const float radius_feature = voxel_size * 5.0;

		const pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);
		const pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr fpfhe(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
		fpfhe->setInputCloud(keypoints);
		fpfhe->setSearchSurface(pointcloud);
		fpfhe->setInputNormals(normals);
		fpfhe->setSearchMethod(kdtree);
		fpfhe->setRadiusSearch(radius_feature);
		fpfhe->compute(*fpfh);
	
		return std::make_pair(keypoints, fpfh);
	}

 	Eigen::Matrix4f registration_pc::RANSAC_global_registration(
		const pc_fpfh_feature& scene1,
		const pc_fpfh_feature& scene2 )
	{
		const auto& kp1 = scene1.first;
		const auto& kp2 = scene2.first;
		const auto& fpfh1 = scene1.second;
		const auto& fpfh2 = scene2.second;

	    Eigen::VectorXf p(3);
	    p[0]=0;
	    p[1]=1;
	    p[2]=0;

		const float distance_threshold = voxel_size * 2.5;
	    boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> >  warp_fcn(new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
	    warp_fcn->setParam(p);
	    boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
	    te->setWarpFunction (warp_fcn);
		const pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		const pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>::Ptr align(new pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>);
		align->setInputSource(kp1);
		align->setSourceFeatures(fpfh1);
		align->setInputTarget(kp2);
		align->setTargetFeatures(fpfh2);
	    align->setTransformationEstimation(te);
	    align->setMaximumIterations(500000);
		align->setNumberOfSamples(4);
		align->setCorrespondenceRandomness(2);
		align->setSimilarityThreshold(0.9f);
		align->setMaxCorrespondenceDistance(distance_threshold);
		align->setInlierFraction(0.25f);
		align->align(*output);

		return align->getFinalTransformation();
	}

	Eigen::Matrix4f registration_pc::ICP_registration(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene1,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene2,
        const Eigen::Matrix4f& trans )
	{
		/*
	    Eigen::VectorXf p(3);
	    p[0]=0;
	    p[1]=1;
	    p[2]=0;
		*/

	    boost::shared_ptr<pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ> >  warp_fcn(new pcl::registration::WarpPointRigid3D<pcl::PointXYZ, pcl::PointXYZ>);
	    //warp_fcn->setParam(p);
	    boost::shared_ptr<pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ> > te (new pcl::registration::TransformationEstimationLM<pcl::PointXYZ, pcl::PointXYZ>);
	    te->setWarpFunction (warp_fcn);
		
		const pcl::PointCloud<pcl::PointXYZ>::Ptr scene1_temp = scene1->makeShared();
		pcl::transformPointCloud(*scene1_temp, *scene1_temp, trans);

		const float distance_threshold = voxel_size * 0.4;

		const pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
		const pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp(new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
		// 最大対応距離
		icp->setMaxCorrespondenceDistance(0.5);
	    icp->setMaximumIterations(500000);
		icp->setInputSource(scene1_temp);
		icp->setInputTarget(scene2);
		icp->setTransformationEstimation(te);
		icp->setMaxCorrespondenceDistance(distance_threshold);
		icp->align(*output);

		return icp->getFinalTransformation();
	}

	void registration_pc::save_data(std::string& filename, Eigen::Matrix4f R)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		read_pointcloud(filename, cloud);
		
		pcl::transformPointCloud(*cloud, *cloud, R);
		pcl::io::savePLYFileASCII(saveName, *cloud);
	}
	
	void registration_pc::print4x4Matrix(const Eigen::Matrix4f& matrix)
	{
  		printf ("Rotation matrix :\n");
  		printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    	printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    	printf ("Translation vector :\n");
    	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
	}

	void registration_pc::range_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
	{
	    pcl::PassThrough<pcl::PointXYZ> passZ;
	    passZ.setInputCloud(cloud);
	    passZ.setFilterFieldName("z");
	    passZ.setFilterLimits(0.0, 2.0);
	    passZ.setFilterLimitsNegative (true);
	    passZ.filter(*cloud);

	    pcl::PassThrough<pcl::PointXYZ> passY;
	    passY.setInputCloud(cloud);
	    passY.setFilterFieldName("y");
	    passY.setFilterLimits(0.0, 3.0);
	    passY.setFilterLimitsNegative (false);
	    passY.filter(*cloud);
	}

	void registration_pc::voxelization_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
	{
		pcl::VoxelGrid<pcl::PointXYZ> sor;
  		sor.setInputCloud(cloud);
  		sor.setLeafSize (voxel_size, voxel_size, voxel_size);
  		sor.filter (*cloud);
	}

	void registration_pc::filter()
	{
		range_filter(scene1_pc);
		range_filter(scene2_pc);

		voxelization_filter(scene1_pc);
		voxelization_filter(scene2_pc);
	}

	void registration_pc::registrate()
	{	
		read_pointcloud(modelname, scene1_pc);
		read_pointcloud(cloudname, scene2_pc);

		filter();

		view_origin = draw_pointcloud("origin", scene1_pc, scene2_pc, Eigen::Matrix4f::Identity());

		scene1_kp_fpfh = fpfh_search_fuature(scene1_pc);
		scene2_kp_fpfh = fpfh_search_fuature(scene2_pc);

		RANSAC_R = RANSAC_global_registration(scene1_kp_fpfh, scene2_kp_fpfh);

		ICP_R_1 = ICP_registration(scene1_pc, scene2_pc, RANSAC_R);
		ICP_R_2 = ICP_registration(scene1_pc, scene2_pc, RANSAC_R*ICP_R_1);

		Eigen::Matrix4f R = ICP_R_2 * RANSAC_R;

		print4x4Matrix(R);

		view_ICP = draw_pointcloud("ICP", scene1_pc, scene2_pc, R);
		save_data(modelname, R);
		
		while (!view_origin->wasStopped() && !view_ICP->wasStopped())
		{
			view_origin->spinOnce();
			view_ICP->spinOnce(100);
		}
	}

	void registration_pc::registrate_for_ROS_to_PCL()
	{	
		pcl::copyPointCloud(*cloud, *scene1_pc);

		int num = count--;
    	cloudname = "/root/ply_data/" + std::to_string(num) + ".ply"; 
		read_pointcloud(cloudname, scene2_pc);

		filter();

		scene1_kp_fpfh = fpfh_search_fuature(scene1_pc);
		scene2_kp_fpfh = fpfh_search_fuature(scene2_pc);

		RANSAC_R = RANSAC_global_registration(scene1_kp_fpfh, scene2_kp_fpfh);

		ICP_R_1 = ICP_registration(scene1_pc, scene2_pc, RANSAC_R);
		ICP_R_2 = ICP_registration(scene1_pc, scene2_pc, RANSAC_R*ICP_R_1);

		Eigen::Matrix4f R = ICP_R_2 * RANSAC_R;

		print4x4Matrix(R);
		pcl::transformPointCloud(*cloud, *cloud, R);
	}

	void registration_pc::run()
	{
		registrate();
	}

}