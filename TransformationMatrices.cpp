#include "TransformationMatrices.h"
#include <Eigen/Geometry>


Eigen::Vector4f extractCameraPlane(const Eigen::Matrix4f& modelviewProjection)
{
    Eigen::Vector4f cameraplane = modelviewProjection.row(3) + (modelviewProjection.row(4));
    float tmp = cameraplane[3];
    cameraplane[3] = 0.0f;
    cameraplane.normalize();
    cameraplane[3] = tmp;
    return cameraplane;
    
   /* Eigen::Vector4f cameraPlane = -modelview.col(2);
    cameraPlane[3] = -(playerState.position[0] * cameraPlane[0] + playerState.position[1] * cameraPlane[1] + playerState.position[2] * cameraPlane[2]);*/
    ///return -modelview.row(2);
}



#include <stdexcept>
#include <cmath>

Eigen::Matrix4f perspectiveProjectionInfinite(float fovDegrees, float aspect, float zNear)
{
    const float pi = (float)M_PI;
    const float fovRads = fovDegrees * pi / 180.0f;

    const float epsilon = 2.4e-7f;

    Eigen::Matrix4f temp;

    float f = 1.0f / tan(fovRads * .5f);

    temp.col(0) = Eigen::Vector4f(f / aspect, 0.0f, 0.0f, 0.0f);
    temp.col(1) = Eigen::Vector4f(0.0f, f, 0.0f, 0.0f);
    temp.col(2) = Eigen::Vector4f(0.0f, 0.0f, epsilon-1.0f, -1.0f);
    temp.col(3) = Eigen::Vector4f(0.0f, 0.0f, (epsilon - 2.0f) *zNear, 0.0f);

    return temp;
}

Eigen::Matrix4f perspectiveProjection(float fovDegrees, float aspect, float zNear, float zFar )
{
    const float pi = (float)M_PI;
    const float fovRads = fovDegrees * pi / 180.0f;

    Eigen::Matrix4f temp;

    float f = 1.0f / tan(  fovRads * .5f );

    temp.col(0) = Eigen::Vector4f(f / aspect, 0.0f, 0.0f , 0.0f);
    temp.col(1) = Eigen::Vector4f(0.0f,  f , 0.0f  ,0.0f);
    temp.col(2) = Eigen::Vector4f(0.0f, 0.0f, (zFar + zNear) / (zNear - zFar) ,-1.0f );
    temp.col(3) = Eigen::Vector4f(0.0f, 0.0f, 2.0f*zFar * zNear / (zNear-zFar) ,0.0f);

    return temp;
}


Eigen::Matrix4f perspectiveProjection(float left, float right, float bottom, float top, float nearPlane, float farPlane)
{
    Eigen::Matrix4f rval;
    
    float n_2 = 2.0f * nearPlane;
    float width = right - left;
    float height = top-bottom;
    
    rval.col(0) = Eigen::Vector4f(n_2 / width, 0.0f, 0.0f, 0.0f);
    rval.col(1) = Eigen::Vector4f(0.0f, n_2 / height, 0.0f, 0.0f);
    rval.col(2) = Eigen::Vector4f((right + left) / width, (top + bottom) / height, -(farPlane + nearPlane) / (farPlane - nearPlane), -1.0f);
    rval.col(3) = Eigen::Vector4f(0.0f, 0.0f, -n_2 * farPlane / (farPlane - nearPlane), 0.0);
    
    return rval;
}


Eigen::Matrix4f perspectiveProjectionInfinite(float left, float right, float bottom, float top, float nearPlane)
{
    Eigen::Matrix4f rval;
    
    float n_2 = 2.0f * nearPlane;
    float width = right - left;
    float height = top-bottom;
    
    rval.col(0) = Eigen::Vector4f(n_2 / width, 0.0f, 0.0f, 0.0f);
    rval.col(1) = Eigen::Vector4f(0.0f, n_2 / height, 0.0f, 0.0f);
    rval.col(2) = Eigen::Vector4f((right + left) / width,
                                  (top + bottom) / height,
                                  -1.0f, //
                                  -1.0f);
    rval.col(3) = Eigen::Vector4f(0.0f,
                                  0.0f,
                                  -n_2,
                                  0.0);
    
    return rval;
}

// Returns a projection matrix based on the given FOV.
Eigen::Matrix4f perspectiveProjectionWithFOVAngles(float fovRadsX, float fovRadsY, const float nearPlane, const float farPlane)
{
    const float halfWidth = nearPlane * tanf(.5f * fovRadsX);
    const float halfHeight = nearPlane * tanf(.5f * fovRadsY);
    
    const float minX = -halfWidth;
    const float maxX = halfWidth;
    
    const float minY = -halfHeight;
    const float maxY = halfHeight;
    
    return perspectiveProjection( minX, maxX, minY, maxY, nearPlane, farPlane);
}

// Returns a projection matrix based on the given FOV.
Eigen::Matrix4f perspectiveProjectionInfiniteWithFOVAngles(float fovRadsX, float fovRadsY, const float nearPlane)
{
    const float halfWidth = nearPlane * tanf(.5f * fovRadsX);
    const float halfHeight = nearPlane * tanf(.5f * fovRadsY);
    
    const float minX = -halfWidth;
    const float maxX = halfWidth;
    
    const float minY = -halfHeight;
    const float maxY = halfHeight;
    
    return perspectiveProjectionInfinite( minX, maxX, minY, maxY, nearPlane);
}




Eigen::Matrix4f rotationMatrixY(float theta)
{
    Eigen::Matrix4f temp;

    ///\todo
    temp.col(0) = Eigen::Vector4f(cos(theta), 0.0f, -sin(theta), 0.0f);
    temp.col(1) = Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.0f);
    temp.col(2) = Eigen::Vector4f(sin(theta), 0.0f, cos(theta), 0.0f);
    temp.col(3) = Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f);

    return temp;
}


Eigen::Matrix3f rotationMatrixY_3x3(float theta)
{
    Eigen::Matrix3f temp;

    temp.col(0) = Eigen::Vector3f(cos(theta), 0.0f, -sin(theta));
    temp.col(1) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    temp.col(2) = Eigen::Vector3f(sin(theta), 0.0f, cos(theta));

    return temp;
}

Eigen::Matrix4f rotationMatrixZ(float theta)
{
    Eigen::Matrix4f temp;

    temp.col(0) = Eigen::Vector4f(cos(theta), sin(theta), 0.0f, 0.0f);
    temp.col(1) = Eigen::Vector4f(-sin(theta), cos(theta), 0,0);
    temp.col(2) = Eigen::Vector4f(0.0f,0.0f,1.0f,0.0f);
    temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0f);

    return temp;
}

Eigen::Matrix4f rotationMatrixX(float theta)
{
    Eigen::Matrix4f temp;

    temp.col(0) = Eigen::Vector4f(1.0f,0.0f,0.0f,0.0f);
    temp.col(1) = Eigen::Vector4f(0.0f,cos(theta), sin(theta), 0.0f);
    temp.col(2) = Eigen::Vector4f(0.0f,-sin(theta), cos(theta), 0.0f);
    temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1);

    return temp;
}



Eigen::Matrix4f scalingMatrix(const Eigen::Vector3f& vec)
{
    Eigen::Matrix4f temp;
    
    temp.col(0) = Eigen::Vector4f(vec[0],0.0f,0.0f,0.0f);
    temp.col(1) = Eigen::Vector4f(0.0f,vec[1],0.0f,0.0f);
    temp.col(2) = Eigen::Vector4f(0.0f,0.0f,vec[2],0.0f);
    temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0);
    
    return temp;
}


Eigen::Matrix4f scalingMatrix(float x, float y, float z)
{
    Eigen::Matrix4f temp;
    
    temp.col(0) = Eigen::Vector4f(x,0.0f,0.0f,0.0f);
    temp.col(1) = Eigen::Vector4f(0.0f,y,0.0f,0.0f);
    temp.col(2) = Eigen::Vector4f(0.0f,0.0f,z,0.0f);
    temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0);

    return temp;
}


Eigen::Matrix4f translationMatrix(const Eigen::Vector3f& trans)
{
    Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
    
    temp.col(3) = Eigen::Vector4f(trans[0],trans[1],trans[2],1.0f);
    
    return temp;
}

Eigen::Matrix4f translationMatrix(float x, float y, float z)
{
    
    Eigen::Matrix4f temp = Eigen::Matrix4f::Identity();
    
    temp.col(3) = Eigen::Vector4f(x,y,z,1.0f);
    
    return temp;
}

Eigen::Matrix3f normalMatrix( Eigen::Matrix4f& modelViewMatrix)
{
    return modelViewMatrix.topLeftCorner<3,3>().inverse().transpose();
}

Eigen::Matrix4f rotationMatrixXYZ(float rotX, float rotY, float rotZ)
{
    float sinZ = sin(rotZ);
    float sinX = sin(rotX);
    float sinY = sin(rotY);
    
    float cosZ = cos(rotZ);
    float cosX = cos(rotX);
    float cosY = cos(rotY);

    Eigen::Matrix4f temp;

    temp.col(0) = Eigen::Vector4f(cosY * cosZ,
                                  -cosY * sinZ,
                                  sinY,
                                  0.0f);
    
    temp.col(1) = Eigen::Vector4f(cosX*sinZ + sinX * sinY * cosZ,
                                  cosX*cosZ + sinX * sinY * sinZ,
                                  -sinX * cosY,
                                  0.0
                                  );
    
    temp.col(2) = Eigen::Vector4f(sinX*sinZ - cosX * sinY * cosZ,
                                  sinX*cosZ + cosX * sinY * sinZ,
                                  cosX * sinY,
                                  0.0
                                  );

    temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0);

    return temp;
}


///\todo http://eigen.tuxfamily.org/dox/classEigen_1_1AngleAxis.html
/// Eigen does this for us already, as well as conversions to / from matrices, quats, etc
///  http://eigen.tuxfamily.org/dox/group__TutorialGeometry.html
Eigen::Matrix4f rotateAxisAngle(Eigen::Vector3f axis, float angle)
{
    
    float cosTheta = cos(angle);
    float sinTheta = sin(angle);
    float oneMinusCosTheta = 1.0f - cosTheta;
    
    Eigen::Matrix4f temp;
    
    temp.col(0) = Eigen::Vector4f(axis[0] * axis[0] * oneMinusCosTheta + cosTheta,
                                  axis[1] * axis[0] * oneMinusCosTheta + axis[2] * sinTheta,
                                  axis[2] * axis[0] * oneMinusCosTheta - axis[1] * sinTheta,
                                  0.0);
    
    temp.col(1) = Eigen::Vector4f(axis[1] * axis[0] * oneMinusCosTheta - axis[2] * sinTheta,
                                  axis[1] * axis[1] * oneMinusCosTheta + cosTheta,
                                  axis[2] * axis[1] * oneMinusCosTheta + axis[0] * sinTheta,
                                  0.0
                                );
    
    temp.col(2) = Eigen::Vector4f(axis[2] * axis[0] * oneMinusCosTheta + axis[1] * sinTheta,
                                  axis[1] * axis[2] * oneMinusCosTheta - axis[0] * sinTheta,
                                  axis[2] * axis[2] * oneMinusCosTheta + cosTheta,
                                  0.0
    );
    
    
    temp.col(3) = Eigen::Vector4f(0.0f,0.0f,0.0f,1.0);
    
    return temp;
}


Eigen::Matrix4f cameraFrameMatrix(const Eigen::Vector3f& lookVector, const Eigen::Vector3f& upVecIn, const Eigen::Vector3f& position)
{
    Eigen::Matrix4f temp;
    Eigen::Matrix3f rotPortion;

    //we want the camera to look down the negative z axis (rhs)
    rotPortion.row(2) = -(lookVector);
    rotPortion.row(2).normalize();

    Eigen::Vector3f upVector = upVecIn;
    upVector.normalize();

    rotPortion.row(0) = upVector.cross(rotPortion.row(2));

    // guarantee orthogonality by recalculating the up
    rotPortion.row(1) = rotPortion.row(2).cross(rotPortion.row(0));
    
    
    ///\todo
    rotPortion.row(0).normalize();
    rotPortion.row(1).normalize();
    rotPortion.row(2).normalize();
    
    
    Eigen::Vector3f transPortion = -(rotPortion * position);

    
    temp.topLeftCorner<3, 3>() = rotPortion;
    temp.row(3) = Eigen::Vector4f(0, 0, 0, 1);
    temp.topLeftCorner<3, 4>().col(3) = transPortion;

    return temp;
}