#pragma once

#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

template<typename T, int N>
using Vector = Eigen::Matrix<T, N, 1>;

template <typename RealType=float>
struct Transform
{
    using Vec3 = Vector<RealType, 3>;
    using Vec4 = Vector<RealType, 4>;
    using Mat3= Eigen::Matrix<RealType, 3, 3>;
    using Mat4 =  Eigen::Matrix<RealType, 4, 4>;
    using Quaternion = Eigen::Quaternion<RealType> ;

    typedef Vec3 Translation;
    typedef Quaternion Orientation;
    typedef Vec3 Scale;

    Translation pos = Translation(0., 0., 0.);
    Orientation rot = Orientation(0., 0., 0., 0.);
    Scale s = Scale(1.0, 1.0, 1.0);

    Transform()
    {
    }

    // decomposition -- assumes input is composed of TRS transformations
    Transform(const Mat4& mat);


    Transform(const Mat3& mat);
    

    void setOrientation(const Mat3& f);
    Mat4 matrix() const;
    Vec3 origin() const;

    void setOrigin(Vec3 newPosition);
};

template <typename RealType = float>
struct TransformStack
{
    Transform<RealType> transform;
    Transform<RealType>* parentTransform;
};


using Transformf = Transform<float>;
using Transformd = Transform<double>;


