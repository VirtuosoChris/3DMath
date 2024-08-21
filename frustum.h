#ifndef _FRUSTUM_H_INCLUDED
#define _FRUSTUM_H_INCLUDED

#include <Eigen/Core>

typedef Eigen::AlignedBox<float, 3> AABB;

namespace Virtuoso
{
    namespace Math
    {
        ///identifier enum for the various planes of the viewing volume
        enum FrustumPlane { LEFT_PLANE = 0, RIGHT_PLANE = 1, BOTTOM_PLANE = 2, TOP_PLANE = 3, FAR_PLANE = 4, NEAR_PLANE = 5 };

        ///when testing a volume against a plane, we can have 3 cases.  We can intersect the plane, or be on either side of it
        enum PlaneTest { ABOVE_PLANE, INTERSECTS_PLANE, BELOW_PLANE };
        enum FrustumTest { OUTSIDE_FRUSTUM = ABOVE_PLANE, INTERSECTS_FRUSTUM = INTERSECTS_PLANE, INSIDE_FRUSTUM = BELOW_PLANE };

        ///takes in a 4 component vector of the form x,y,z,d that satisfies the equation ax + by + cz + d = 0
        ///and an axis aligned bounding box.  Returns whether the box is above, below, or intersecting the plane
        PlaneTest AABBPlaneIntersectionTest(const Eigen::Vector4f& planeNormal, const AABB& box);

        FrustumTest AABBFrustumIntersection(const Eigen::Matrix4f& modelviewProjectionMatrix, const AABB& box);

        FrustumTest AABBFrustumIntersection(const std::array<Eigen::Vector4f, 6>& planeNormals,
            const AABB& box,
            unsigned short& start_plane, //the plane we want to start testing with.  We will loop around
            unsigned short& insideCache, //the planes we don't have to test because we know we're already inside them
            unsigned short& intersectCache
        );

        inline bool AABBFrustumIntersectionTest(const Eigen::Matrix4f& modelviewProjectionMatrix, const AABB& box)
        {
            return OUTSIDE_FRUSTUM != AABBFrustumIntersection(modelviewProjectionMatrix, box);
        }

        ///a method which takes in a modelview projection matrix and extracts the frustum normals from them, 
        ///in the form of <x,y,z,d> where x,y,z is the outward facing normal to the frustum plane and these values
        ///satisfy the plane equation ax + by + cz + d = 0
        std::array<Eigen::Vector4f, 6> getFrustumPlaneNormals(const Eigen::Matrix4f& modelviewProjectionMatrix);

        inline PlaneTest spherePlaneIntersection(const Eigen::Vector4f& planeNormal,
            const Eigen::Vector4f& centroid,
            const float& radius);

        inline FrustumTest sphereFrustumIntersection(const std::array<Eigen::Vector4f, 6>& planeNormals,
            const Eigen::Vector4f& centroid,
            const float& radius,
            std::uint8_t& insideCache,
            std::uint8_t& startPlane
        );

    } // namespace Math
} // namespace Virtuoso
#endif