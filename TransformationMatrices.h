///\todo do non float versions

//#if !defined(TRANSFORMATIONS_H_INCLUDED) && !defined(VIRTUOSO_TRANSFORMATIONSLIB_IMPLEMENTATION)
#ifndef TRANSFORMATIONS_H_INCLUDED
#define TRANSFORMATIONS_H_INCLUDED

#include <Eigen/Core>
#include <cmath>

///\file These functions create transformation matrices using the Eigen library.  What these particular transforms are and what their
/// arguments do should all hopefully be obvious.  All angles are in radians and sane default arguments are provided where possible

///lookat()
///orthoProjection()

Eigen::Vector4f extractCameraPlane(const Eigen::Matrix4f& modelviewProjection);

Eigen::Matrix4f cameraFrameMatrix(const Eigen::Vector3f& lookVector, const Eigen::Vector3f& upVecIn, const Eigen::Vector3f& position = Eigen::Vector3f(0, 0, 0));

Eigen::Matrix4f perspectiveProjectionInfinite(float fov = 60, float aspect = 1280/720.0, float zNear = .01);
Eigen::Matrix4f perspectiveProjectionInfinite(float left, float right, float bottom, float top, float near);

Eigen::Matrix4f perspectiveProjection(float fov = 60, float aspect = 1280/720.0, float zNear = .01, float zFar = 1000.0 );

Eigen::Matrix4f perspectiveProjectionWithFOVAngles(float fovRadsX, float fovRadsY, const float near, const float far);

Eigen::Matrix4f perspectiveProjectionInfiniteWithFOVAngles(float fovRadsX, float fovRadsY, const float near);

Eigen::Matrix4f perspectiveProjection(float left, float right, float top, float bottom, float near, float far);

Eigen::Matrix4f rotationMatrixY(float theta);

Eigen::Matrix3f rotationMatrixY_3x3(float theta);

Eigen::Matrix4f rotationMatrixZ(float theta);

Eigen::Matrix4f rotationMatrixX(float theta);

Eigen::Matrix4f rotationMatrixXYZ(float rotX, float rotY, float rotZ);

Eigen::Matrix4f rotateAxisAngle(Eigen::Vector3f axis, float angle);

Eigen::Matrix4f scalingMatrix(float x, float y, float z);
Eigen::Matrix4f scalingMatrix(const Eigen::Vector3f& vec);

Eigen::Matrix4f translationMatrix(float x, float y, float z);

Eigen::Matrix4f translationMatrix(const Eigen::Vector3f& trans);

///creates a matrix to transform normals from object space to eye space while maintaining their perpindicularity to the surface
Eigen::Matrix3f normalMatrix(const Eigen::Matrix4f& modelViewMatrix);



#endif
