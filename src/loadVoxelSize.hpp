#include <iostream>
#include <string>
#include <boost/property_tree/xml_parser.hpp>

namespace registration {
    
    void loadParams(float *size, std::string &standard, 
                    std::string &cloud, std::string &save){

        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml("root/catkin_ws/src/make_ply/include/voxel_size.xml", pt);

        //resolutionを取得
        *size = pt.get<double>("datas.voxelSize");
        //出来形点群データファイルの取得
        standard = pt.get<std::string>("datas.standard");
        //点群の回転行列データファイルの取得
        cloud = pt.get<std::string>("datas.cloud");
        //voxelの保存先の取得
        save = pt.get<std::string>("datas.saveDir");

        std::cout << *size << std::endl;
        std::cout << standard << std::endl;
        std::cout << cloud << std::endl;
        std::cout << save << std::endl;
    }
}
