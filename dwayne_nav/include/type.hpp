/*
 * @Author       : dwayne
 * @Date         : 2023-04-06
 * @LastEditTime : 2023-04-06
 * @Description  : 关于矩阵及向量的定义
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include <Eigen/Core>
#include <vector>

enum NODE_STATUS
{
    NOT_VISITED = 0,
    IN_OPENSET  = 1,
    IN_CLOSESET = 2
};

enum DIRECTION
{
    FORWARD  = 0,
    BACKWARD = 1,
    NO       = 3
};

template<int dim>
using TypeVectorVecd =
    typename std::vector<Eigen::Matrix<double, dim, 1>,
                         Eigen::aligned_allocator<Eigen::Matrix<double, dim, 1>>>;

typedef TypeVectorVecd<2> VectorVec2d;
typedef TypeVectorVecd<3> VectorVec3d;
typedef TypeVectorVecd<4> VectorVec4d;

typedef Eigen::Vector2d Vec2d;
typedef Eigen::Vector3d Vec3d;
typedef Eigen::Vector4d Vec4d;

typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector4i Vec4i;