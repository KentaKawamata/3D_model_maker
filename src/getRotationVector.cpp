#include "./getRotationVector.hpp"

GetRotationVector::GetRotationVector() : 
    R3d (Eigen::Matrix3d::Identity()),
    under_R3d (Eigen::Matrix3d::Identity()),
    R (Eigen::Matrix4d::Identity()),
    under_R (Eigen::Matrix4d::Identity()),
    tpclX(0.0),
    tpclY(0.0),
    tpclZ(0.0),
    pitch(0.0),
    under_pitch(0.0),
    roll(0.0),
    yaw(0.0)
{
}

GetRotationVector::~GetRotationVector() {
    
}

void GetRotationVector::setOverRotate4(){

    R(0,0) = R3d(0,0);
    R(0,1) = R3d(0,1);
    R(0,2) = R3d(0,2);
 
    R(1,0) = R3d(1,0);
    R(1,1) = R3d(1,1);
    R(1,2) = R3d(1,2);
 
    R(2,0) = R3d(2,0);
    R(2,1) = R3d(2,1);
    R(2,2) = R3d(2,2);

    R(0,3) = tpclX; 
    R(1,3) = tpclY; 
    R(2,3) = tpclZ;
    R(3,3) = 1.0;
}

void GetRotationVector::setUnderRotate4(){

    under_R(0,0) = under_R3d(0,0);
    under_R(0,1) = under_R3d(0,1);
    under_R(0,2) = under_R3d(0,2);
 
    under_R(1,0) = under_R3d(1,0);
    under_R(1,1) = under_R3d(1,1);
    under_R(1,2) = under_R3d(1,2);
 
    under_R(2,0) = under_R3d(2,0);
    under_R(2,1) = under_R3d(2,1);
    under_R(2,2) = under_R3d(2,2);

    under_R(0,3) = tpclX; 
    under_R(1,3) = tpclY; 
    under_R(2,3) = tpclZ;
    under_R(3,3) = 1.0;
}

void GetRotationVector::eulerToRote(){

    Eigen::Matrix3d Rpi;
    Eigen::Matrix3d Ryo;
    Eigen::Matrix3d Rro;
    Eigen::Matrix3d under_Rpi;

    Rpi << 1, 0, 0,  
           0, cos(pitch), sin(pitch),  
           0, -sin(pitch), cos(pitch); 

    Ryo << cos(yaw), 0, -sin(yaw), 
           0, 1, 0, 
           sin(yaw), 0, cos(yaw);

    Rro << cos(roll), sin(roll), 0, 
           -sin(roll), cos(roll), 0, 
           0, 0, 1; 

    under_Rpi << 1, 0, 0,  
           0, cos(under_pitch), sin(under_pitch),  
           0, -sin(under_pitch), cos(under_pitch); 

    R3d = Rpi * Ryo * Rro;
    under_R3d = under_Rpi * Ryo * Rro;
}

void GetRotationVector::transformPointCloud() {

    eulerToRote();
    setOverRotate4();
    setUnderRotate4();
}