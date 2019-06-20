#include <iostream>
#include <string>

#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>

#include "calcAverage.hpp"
#include "loadParams.hpp"

CalcAverage::CalcAverage() : 
    sumCentroid (new pcl::PointCloud<pcl::PointXYZ>()),
    aveCloud (new pcl::PointCloud<pcl::PointXYZ>()),
    samePoints (new pcl::PointCloud<pcl::PointXYZ>())
{
}

CalcAverage::~CalcAverage(){
}

void CalcAverage::getMaxMin(){
    
    max_x = 0;
    max_y = 0;
    max_z = 0;
    min_x = 0;
    min_y = 0;
    min_z = 0;

    for(auto point : sumCentroid->points){
        if(max_z > point.z)
            max_z = max_z;
        else
            max_z = point.z;

        if(max_y > point.y)
            max_y = max_y;
        else
            max_y = point.y;

        if(max_x > point.x)
            max_x = max_x;
        else
            max_x = point.x;

        if(min_z < point.z)
            min_z = min_z;
        else
            min_z = point.z;

        if(min_y < point.y)
            min_y = min_y;
        else
            min_y = point.y;

        if(min_x < point.x)
            min_x = min_x;
        else
            min_x = point.x;
    }
    std::cout << "end min-max PointCloud" << std::endl;
}

double CalcAverage::averageFilter(std::vector<int> list){

    sum = 0;
    for(auto i : list){
       sum += sumCentroid->points[i].y; 
    }
    return sum / list.size();
}

void CalcAverage::setList(){

    std::cout << "start avreage PointCloud" << std::endl;
    getMaxMin();

    for(int i=min_x; i<=max_x; i+=voxelSize){
        for(int j=min_z; j<=max_z; j+=voxelSize){
    
            std::vector<int> list;
            for(int num=0; num<sumCentroid->points.size(); num++){
                if(sumCentroid->points[num].x==i && sumCentroid->points[num].z==j){
                    list.push_back(num);
                }
            }
            double ave = averageFilter(list);
            avePoint.x = i;
            avePoint.y = ave;
            avePoint.z = j;
            aveCloud->points.push_back(avePoint);
        }
    }
}

/*
 * すでに計算済みのxy座標に存在する点群を探す
 */ 
inline bool CalcAverage::findIndex(int num, std::vector<int> index){

    auto sameNum = std::find(index.begin(), index.end(), num);
    if (sameNum != index.end()){
        // 点群が使用済みだった場合
        return false;
    } else {
        // 点群が使用前だった場合
        return true;
    }
}

void CalcAverage::calcAverage() {

    std::vector<float> diffZ = {};
    // 一度計算に使用した点のインデックスjを格納していく
    std::vector<int> index = {};

    for(int i=0; i<sumCentroid->points.size(); i++){

        if(findIndex(i, index) == false){
            continue;
        }

        float x = sumCentroid->points[i].x;
        float z = sumCentroid->points[i].z;

        samePoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> list;
        for(int j=0; j<sumCentroid->points.size(); j++){

            if( (sumCentroid->points[j].x==x && sumCentroid->points[j].z==z) && j!=i){
                samePoints->points.push_back(sumCentroid->points[j]);
                index.push_back(j);
            }
        }
        
        index.push_back(i);
        if(samePoints->points.size()>=1){
            double ave = averageFilter(list);
            avePoint.x = x;
            avePoint.y = ave;
            avePoint.z = z;
            aveCloud->points.push_back(avePoint);
        } else {
            aveCloud->points.push_back(sumCentroid->points[i]);
        }
    }
}

void CalcAverage::run(){

    calcAverage();
}