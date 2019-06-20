#include <iostream>
#include <string>

#include <pcl/common/time.h>
#include <pcl/io/ply_io.h>

#include "getFrame.hpp"
#include "loadParams.hpp"

GetFrame::GetFrame() : 
    cloud (new pcl::PointCloud<pcl::PointXYZ>()),
    sumCloud (new pcl::PointCloud<pcl::PointXYZ>()),
    aveCloud (new pcl::PointCloud<pcl::PointXYZ>()),
    reso(0.001),
    resolution(0.0001),
    number(0)
{
    std::string ghoost("Not Use");
    octree = new OctreeViewer(ghoost, resolution);
    edit = new EditCloud(ghoost, rote, ghoost);
    calcAve = new CalcAverage();
}

GetFrame::~GetFrame(){
    delete octree;
    delete edit;
    delete calcAve;
}

void GetFrame::averagePointCloud(){

    int count = 0;

    /**
     * Conbine multiple PointCloud data
     * 
     **/ 
    while(count<5) {

        output_names = "./data/" + std::to_string(number) + ".ply";
        number++;

        pcl::io::loadPLYFile (output_names, *cloud);
                
        *sumCloud += *cloud;
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

        count++;                
    }

    /**
     *  Voxelize sumCoud for average PointCloud data
     **/ 
    pcl::copyPointCloud(*sumCloud, *(octree->cloud));
    octree->run();
    std::cout << "voxelSize : " << octree->voxelSize << std::endl;

    /**
     *  Calc average of PointCloud data.
     **/
    pcl::copyPointCloud(*(octree->displayCloud), *(calcAve->sumCentroid));
    calcAve->voxelSize = octree->voxelSize;     
    calcAve->run();
    pcl::copyPointCloud(*(calcAve->aveCloud), *aveCloud);

    pcl::io::savePLYFileASCII(saveDir, *aveCloud);

}

void GetFrame::run(){

    //averagePointCloud();
}