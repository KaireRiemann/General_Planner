#ifndef CONTROLLER_UTILS_H
#define CONTROLLER_UTILS_H

#include<tf2/LinearMath/Quaternion.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<geometry_msgs/Quaternion.h>
#include<Eigen/Eigen>

namespace controller
{
    const double PI = 3.1415926;
    //欧拉角转四元数
    inline geometry_msgs::Quaternion quaternion_from_rpy(double roll, double pitch, double yaw)
    {
        if(yaw > PI)
        {
            yaw -= 2 * PI;
        }

        tf2::Quaternion quaternion_tf2;
        //通过欧拉角转化为四元数
        quaternion_tf2.setRPY(roll,pitch,yaw);
        //转化为ros消息
        geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
        return quaternion;
    }

    //从四元数中获得yaw
    inline double yaw_from_quaternion(const geometry_msgs::Quaternion &quat)
    {
        tf2::Quaternion tf_quat;
        tf2::convert(quat,tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll,pitch,yaw);
        return yaw;
    }

    //获取欧拉角
    inline void rpy_from_quaternion(const geometry_msgs::Quaternion &quat, double &roll, double &pitch, double &yaw)
    {
        tf2::Quaternion tf_quat;
        tf2::convert(quat,tf_quat);
        tf2::Matrix3x3(tf_quat).getRPY(roll,pitch,yaw);
    }

    // | w² + x² - y² - z²   2xy - 2wz           2xz + 2wy         |
    // | 2xy + 2wz           w² - x² + y² - z²   2yz - 2wx         |
    // | 2xz - 2wy           2yz + 2wx           w² - x² - y² + z² |
    //四元数转化为旋转矩阵
    inline Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q)
    {
        Eigen::Matrix3d rot_mat;
        rot_mat<<q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2*q(1)*q(2) - 2*q(0)*q(3),2*q(1)*q(3) + 2*q(0)*q(2),
                2*q(1)*q(2) + 2*q(0)*q(3), q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2 * q(2) * q(3) - 2 * q(0) * q(1),
                2*q(1)*q(3) - 2*q(0)*q(2), 2*q(2)*q(3)+2*q(0)*q(1), q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);
        return rot_mat;
    }

    inline Eigen::Vector4d rotmat2Quaternion(const Eigen::Matrix3d &R)
    {
        Eigen::Vector4d quat;
        double tr = R.trace();
        if (tr > 0.0){
            double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
            quat(0) = 0.25 * S;
            quat(1) = (R(2, 1) - R(1, 2)) / S;
            quat(2) = (R(0, 2) - R(2, 0)) / S;
            quat(3) = (R(1, 0) - R(0, 1)) / S;
        } 
        else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))){
            double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
            quat(0) = (R(2, 1) - R(1, 2)) / S;
            quat(1) = 0.25 * S;
            quat(2) = (R(0, 1) + R(1, 0)) / S;
            quat(3) = (R(0, 2) + R(2, 0)) / S;
        } 
        else if (R(1, 1) > R(2, 2)){
            double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
            quat(0) = (R(0, 2) - R(2, 0)) / S;
            quat(1) = (R(0, 1) + R(1, 0)) / S;
            quat(2) = 0.25 * S;
            quat(3) = (R(1, 2) + R(2, 1)) / S;
        } 
        else{
            double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
            quat(0) = (R(1, 0) - R(0, 1)) / S;
            quat(1) = (R(0, 2) + R(2, 0)) / S;
            quat(2) = (R(1, 2) + R(2, 1)) / S;
            quat(3) = 0.25 * S;
        }
        return quat;
    }

    //四元数乘法, q*p表示先使用p的旋转再使用q的旋转
    // w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z
    // x = p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y
    // y = p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x
    // z = p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w
    inline Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p)
    {
        Eigen::Vector4d quat;
        quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
        p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);

        return quat;
    }



} //namespace controller
#endif  