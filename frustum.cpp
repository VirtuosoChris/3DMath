#include "frustum.h"

using namespace Virtuoso::Math;

PlaneTest Virtuoso::Math::spherePlaneIntersection(const Eigen::Vector4f& planeNormal,
    const Eigen::Vector4f& centroid,
    const float& radius)
{
    float signed_distance_plane = planeNormal.dot(centroid); 

    //its completely outside one plane thus is outside the frustum
    if (signed_distance_plane >= radius)
    {
        return ABOVE_PLANE;
    }

    if (signed_distance_plane <= -radius)
    {
        return BELOW_PLANE;
    }
    else
    {
        return INTERSECTS_PLANE;
    }
}


FrustumTest Virtuoso::Math::sphereFrustumIntersection(const std::array<Eigen::Vector4f, 6>& planeNormals,
    const Eigen::Vector4f& centroid,
    const float& radius,
    std::uint8_t& insideCache,
    std::uint8_t& startPlane
)
{
    bool intersection = false;

    for (unsigned int ctr = 0, q = startPlane; ctr < 6; ctr++, q = (startPlane + ctr) % 6)
    {
        unsigned int i = 1 << q;

        if (insideCache & i) continue; //a previous call indicated that we are fully inside this plane so no need to test

        PlaneTest result = spherePlaneIntersection(planeNormals[q], centroid, radius);

        switch (result)
        {
            case ABOVE_PLANE:
            {
                startPlane = q;
                return OUTSIDE_FRUSTUM;
                break;
            }
            case INTERSECTS_PLANE:
            {
                intersection = true;
                break;
            }
            case BELOW_PLANE:
            {
                insideCache |= i; //cache which frustum planes we are completely inside so on recursive traversal calls we don't have to test them
                break;
            }
        }
    }

    return intersection ? INTERSECTS_FRUSTUM : INSIDE_FRUSTUM;
}


///takes in a 4 component vector of the form x,y,z,d that satisfies the equation ax + by + cz + d = 0
///and an axis aligned bounding box.  Returns whether the box is above, below, or intersecting the plane
PlaneTest Virtuoso::Math::AABBPlaneIntersectionTest(const Eigen::Vector4f& planeNormal, const AABB& box)
{
    Eigen::Vector3f centroid = box.center();
    Eigen::Vector3f half_vector = box.max() - centroid;

    Eigen::Vector3f temp(fabs(planeNormal[0]), fabs(planeNormal[1]), fabs(planeNormal[2]));
    float extents = half_vector.dot(temp);
    
    Eigen::Vector4f centroidT(centroid[0], centroid[1], centroid[2], 1.0);
    float centerDistance = planeNormal.dot(centroidT);

    return centerDistance - extents > 0 ?  ABOVE_PLANE : (centerDistance + extents < 0 ? BELOW_PLANE : INTERSECTS_PLANE);
}


FrustumTest Virtuoso::Math::AABBFrustumIntersection(const std::array<Eigen::Vector4f, 6>& planeNormals,
                                  const AABB& box, 
                                  unsigned short& start_plane, //the plane we want to start testing with.  We will loop around
                                  unsigned short& insideCache //the planes we don't have to test because we know we're already inside them
                                  ,unsigned short& intersectCache
                                  )
{
    for(unsigned int counter = 0; counter < 5; counter++)
    {
        unsigned int plane_to_test = (start_plane + counter)%5;
        
        if(insideCache & (1 << plane_to_test) )continue; //if we already know we're inside this plane (from a parent node of a hierarchy, etc), then we don't need to test again
        
        PlaneTest result = AABBPlaneIntersectionTest(planeNormals[plane_to_test], box);

        switch(result)
        {
            //if the AABB is outside ANY of the planes of the frustum completely, then the box is outside the frustum, and testing can stop
            case ABOVE_PLANE:

                start_plane = plane_to_test;
                return OUTSIDE_FRUSTUM;

            case INTERSECTS_PLANE:

                intersectCache |= 1<<plane_to_test;
                break;

            case BELOW_PLANE:

                insideCache |= 1<<plane_to_test;
                break;
        }
    }

    //always test the near plane separately

    PlaneTest result;

    if (insideCache & (1 << NEAR_PLANE) )
    {
        result = BELOW_PLANE;
    }
    else
    {
        result = AABBPlaneIntersectionTest(planeNormals[NEAR_PLANE], box);
    }

    switch(result)
    {
        //if the AABB is outside ANY of the planes of the frustum completely, then the box is outside the frustum, and testing can stop
        case ABOVE_PLANE:
            return OUTSIDE_FRUSTUM;
        case INTERSECTS_PLANE:
            intersectCache |= 1<<NEAR_PLANE;
            break;
        case BELOW_PLANE:
            insideCache |= 1<<NEAR_PLANE;
            break;
    }

    //if we were outside we would have already broken out of the function.  Thus, we return intersection if any of the planes
    //intersect the box, otherwise we must be completely inside
    return intersectCache ? INTERSECTS_FRUSTUM: INSIDE_FRUSTUM;
}


FrustumTest Virtuoso::Math::AABBFrustumIntersection(const Eigen::Matrix4f& modelviewProjectionMatrix, const AABB& box)
{
    auto planeNormals = getFrustumPlaneNormals(modelviewProjectionMatrix);

    unsigned short start_plane = 0; //the plane we want to start testing with.  We will loop around
    unsigned short insideCache = 0; //the planes we don't have to test because we know we're already inside them
    unsigned short intersectCache = 0;

    return AABBFrustumIntersection(planeNormals, box, start_plane, insideCache, intersectCache);
}


///a method which takes in a modelview projection matrix and extracts the frustum normals from them, 
///in the form of <x,y,z,d> where x,y,z is the outward facing normal to the frustum plane and these values
///satisfy the plane equation ax + by + cz + d = 0
std::array<Eigen::Vector4f, 6> Virtuoso::Math::getFrustumPlaneNormals(const Eigen::Matrix4f& modelviewProjectionMatrix)
{
    std::array<Eigen::Vector4f, 6> returnVal;
 
    Eigen::Vector4f row3 = modelviewProjectionMatrix.row(3);

    returnVal[RIGHT_PLANE] = -(row3 - Eigen::Vector4f(modelviewProjectionMatrix.row(0)));
    returnVal[RIGHT_PLANE].normalize();

    returnVal[BOTTOM_PLANE] = -(row3 + Eigen::Vector4f(modelviewProjectionMatrix.row(1)));
    returnVal[BOTTOM_PLANE].normalize();

    returnVal[TOP_PLANE] = -(row3 - Eigen::Vector4f(modelviewProjectionMatrix.row(1)));
    returnVal[TOP_PLANE].normalize();

    returnVal[NEAR_PLANE] = -(row3 + Eigen::Vector4f(modelviewProjectionMatrix.row(2)));
    returnVal[NEAR_PLANE].normalize();

    returnVal[FAR_PLANE] = -(row3 - Eigen::Vector4f(modelviewProjectionMatrix.row(2)));
    returnVal[FAR_PLANE].normalize();

    returnVal[LEFT_PLANE] = -(row3 + Eigen::Vector4f(modelviewProjectionMatrix.row(0)));
    returnVal[LEFT_PLANE].normalize();

    return returnVal;
}
 