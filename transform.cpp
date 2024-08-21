#include "transform.h"


// decomposition -- assumes input is composed of TRS transformations

template<class RealType>
Transform<RealType>::Transform(const Transform<RealType>::Mat4& mat)
{
    pos = mat.template topLeftCorner<3, 4>().col(3);

    Mat3 topleft = mat.template topLeftCorner<3, 3>();

    RealType len0 = topleft.col(0).norm();
    RealType len1 = topleft.col(1).norm();
    RealType len2 = topleft.col(2).norm();

    topleft.col(0) /= len0;
    topleft.col(1) /= len1;
    topleft.col(2) /= len2;

    rot = Orientation(topleft);
    s = Vec3(len0, len1, len2);
}

template<class RealType>
Transform<RealType>::Transform(const Mat3& mat)
{
    RealType len0 = mat.col(0).norm();
    RealType len1 = mat.col(1).norm();
    RealType len2 = mat.col(2).norm();

    Mat3 nmat=mat;

    nmat.col(0) /= len0;
    nmat.col(1) /= len1;
    nmat.col(2) /= len2;

    rot = Orientation(nmat);
    s = Vec3(len0, len1, len2);
}

template<class RealType>
void Transform<RealType>::setOrientation(const Mat3& f)
{
    rot = Orientation(f);
}

template<class RealType>
Transform<RealType>::Mat4 Transform<RealType>::matrix() const
{
    Mat4 rval;
    rval.template topLeftCorner<3, 3>() = rot.toRotationMatrix() * Eigen::DiagonalMatrix<RealType, 3>(s[0], s[1], s[2]);
    rval.template topLeftCorner<3, 4>().col(3) = pos;
    rval.row(3) = Vec4(0., 0., 0., 1.);
    return rval;
}

template<class RealType>
Transform<RealType>::Vec3 Transform<RealType>::origin() const
{
    Vec3 invScale(1.0 / s[0], 1.0 / s[1], 1.0 / s[2]);
    return invScale.cwiseProduct(rot.inverse() * -pos);
}

template<class RealType>
void Transform<RealType>::setOrigin(Transform<RealType>::Vec3 newPosition)
{
    Vec3 invScale(1.0 / s[0], 1.0 / s[1], 1.0 / s[2]);
    pos = -(rot * newPosition);
}

template class Transform<float>;
template class Transform<double>;

