#ifndef GET_ROTATION_VECTOR_H
#define GET_ROTATION_VECTOR_H

#include <Eigen/Core>
#include <Eigen/LU>

class GetRotationVector {

private:

    Eigen::Matrix3d R3d;
    Eigen::Matrix3d under_R3d;

    void eulerToRote();
    void setOverRotate4();
    void setUnderRotate4();

public:

    double roll;
    double pitch;
    double yaw;
    double under_pitch;

    double tpclX;
    double tpclY;
    double tpclZ;
    
    Eigen::Matrix4d R;
    Eigen::Matrix4d under_R;

    GetRotationVector();
    ~GetRotationVector();
    void transformPointCloud();
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //GET_ROTATION_VECTOR_H