#include <pcl/filters/passthrough.h>
//#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <cmath>
#include <iostream>

#include "./../include/editCloud.hpp"

EditCloud::EditCloud() : 
    cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    over_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    under_cloud (new pcl::PointCloud<pcl::PointXYZ>())
{
}

EditCloud::~EditCloud(){

}

void EditCloud::smooth() {
   
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);//Kdtreeの作成

    pcl::PointCloud<pcl::PointNormal> mls_points;//出力する点群の格納場所を作成

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals (true);//法線の計算を行うかどうか

    // 各パラメーターの設定
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    mls.process (mls_points);//再構築

    pcl::copyPointCloud(mls_points, *cloud);
}

void EditCloud::rangeFilter() {

    pcl::PassThrough<pcl::PointXYZ> passX;
    passX.setInputCloud(cloud);
    passX.setFilterFieldName("x");
    passX.setFilterLimits(-2.0, 2.0);
    passX.setFilterLimitsNegative(false);
    passX.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> passY;
    passY.setInputCloud(cloud);
    passY.setFilterFieldName("y");
    passY.setFilterLimits(-2.0, 3.0);
    passY.setFilterLimitsNegative(false);
    passY.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> passZ;
    passZ.setInputCloud(cloud);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits(0.3, 3.0);
    passZ.setFilterLimitsNegative(false);
    passZ.filter(*cloud);
}

void EditCloud::outline(){

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    /*     
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
    sor1.setInputCloud(cloud);
    sor1.setMeanK(50);
    sor1.setStddevMulThresh(0.01);
    sor1.filter(*cloud);
    */
    //pcl::PLYWriter writer;
    //writer.write<pcl::PointXYZ> ("/home/kawa/program/calc3D/data/outline.ply", *newCloud, false);
}

void EditCloud::filter() {

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    //over_cloud
    pcl::copyPointCloud(*over_cloud, *cloud);
    rangeFilter();
    outline();
    pcl::copyPointCloud(*cloud, *over_cloud);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    //under_cloud
    pcl::copyPointCloud(*under_cloud, *cloud);
    rangeFilter();
    outline();
    pcl::copyPointCloud(*cloud, *under_cloud);
}