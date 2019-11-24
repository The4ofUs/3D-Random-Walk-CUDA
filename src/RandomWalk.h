#ifndef RANDOMWALK_H
#define RANDOMWALK_H

#include "Boundary.h"
#include "RNG.h"



/**
 * @brief randomWalk
 * keeps wandering around with the photon in the 3D space
 * @return The Point where the Photon hits the Boundary
 */
__device__ Point randomWalk(curandState_t *states, int idx, Boundary boundary, RNG rng)
{
    Ray ray = Ray(Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f));

    while (!boundary.isCrossed(ray))
    {
        ray.move(rng.getRandomPoint(states, idx), rng.getRandomStep(states, idx));
    }
    return boundary.getIntersectionPoint(ray);
}

#endif