#ifndef BOUNDARY_H
#define BOUNDARY_H

#include <cmath>
#include "Ray.h"

/**
 * @brief The Boundary class
 * Spherical Boundary class
 */
class Boundary {

/**
 * @brief dotProduct
 * @param point1
 * @param point2
 * @return Dot Product
 */
__device__ float dot(Point point1, Point point2);

/**
 * @brief swap
 * @param num1
 * @param num2
 * Swaps values by reference
 */
__device__ void swap(float &num1, float &num2);

public:
/**
 * @brief Constructor
 * @param r
 * @param c
 * r sets the radius of the spherical boundary
 * c sets the center of the spherical boundary
 */
 __device__ __host__ Boundary(float r, Point c);

/**
 * @brief setRadius
 * @param r
 * sets the radius of the spherical boundary
 */
__device__ void setRadius(float r);

/**
 * @brief getRadius
 * @return the radius of the spherical boundary
 */
__device__ float getRadius() const;

/**
 * @brief setCenter
 * @param c
 * sets the center of the spherical boundary
 */
__device__ void setCenter(Point c);

/**
 * @brief getCenter
 * @return the center of the spherical boundary 
 */
__device__ Point getCenter() const;

/**
 * @brief isCrossed
 * @param ray
 * @return whether the Boundary was crossed by the given ray or not
 */
__device__ bool isCrossed(Ray ray);

/**
 * @brief getIntersectionPoint
 * @param ray
 * @return the intersection Point between the ray and the spherical boundary
 */
__device__ Point getIntersectionPoint(Ray ray);


private:
/**
 * @brief _radius
 * the radius of the spherical boundary
 */
float _radius;

/**
 * @brief _center
 * the center of the spherical boundary
 */
Point _center;

};
#endif




/**
 *  R0 = Ray's origin (Prev Position)
 *  Rd = The direction of the ray (Ray's Direction)
 *  t = this is the intersection distance from the ray origin to the hit point 
 *  t = dot((CenterOfTheSphere - R0), Rd)
 *  x = sqrt(r^2 - y^2)
 *  y = length(S - P)
 *  where S is the center of the sphere, P is anypoint on the Ray
 *  t1 = t+x
 *  t2 = t-x
 *  pointOfIntersection = RayOrigin + direction*t
 */
