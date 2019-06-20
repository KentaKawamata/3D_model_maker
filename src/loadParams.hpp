#include <iostream>
#include <string>
#include <boost/property_tree/xml_parser.hpp>

void loadBag(double *reso, std::string &bag, 
                    std::string &rote, std::string &save){

    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml("./src/setting.xml", pt);

    //resolutionを取得
    *reso = pt.get<double>("datas.resolution");
    //出来形点群データファイルの取得
    bag = pt.get<std::string>("datas.bagData");
    //点群の回転行列データファイルの取得
    rote = pt.get<std::string>("datas.rotationData");
    //voxelの保存先の取得
    save = pt.get<std::string>("datas.saveDir");

    std::cout << *reso << std::endl;
    std::cout << bag << std::endl;
    std::cout << rote << std::endl;
    std::cout << save << std::endl;
}

void loadPLY(double *reso, std::string &ply, 
                    std::string &rote, std::string &save){

    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml("./src/setting.xml", pt);

    //resolutionを取得
    *reso = pt.get<double>("datas.resolution");
    //出来形点群データファイルの取得
    ply = pt.get<std::string>("datas.plyData");
    //点群の回転行列データファイルの取得
    rote = pt.get<std::string>("datas.rotationData");
    //voxelの保存先の取得
    save = pt.get<std::string>("datas.saveDir");

    std::cout << *reso << std::endl;
    std::cout << ply << std::endl;
    std::cout << rote << std::endl;
    std::cout << save << std::endl;
}