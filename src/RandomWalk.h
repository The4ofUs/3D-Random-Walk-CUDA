#ifndef RANDOMWALK_H
#define RANDOMWALK_H

#include "Boundary.h"
#include "RNG.h"

#define BOUNDARY_RADIUS 5.0

/**
 * @brief randomWalk
 * keeps wandering around with the photon in the 3D space
 * @return The Point where the PHoton hits the Boundary
 */
__device__ Point randomWalk(curandState_t *states)
{
    RNG rng;
    Point origin = Point(0.f, 0.f, 0.f);
    Ray ray = Ray(origin, origin);
    Boundary boundary = Boundary(BOUNDARY_RADIUS, origin);
    long iterator = 0;
    
    while (!boundary.isCrossed(ray))
    {
        ray.setDirection(rng.getRandomPoint(states, iterator));

        ray.setStep(rng.getRandomStep(states, iterator));

        ray.move();

        iterator++;
    }

    return boundary.getIntersectionPoint(ray);
}

#endif