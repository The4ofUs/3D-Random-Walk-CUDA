#ifndef RNG_H
#define RNG_H

#include <curand.h>
#include <curand_kernel.h>
#include "Point.h"
#include "Vector.h"
#include <stdio.h>  /* printf, scanf, puts, NULL */
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
/**
 * @brief The RNG class
 * Pseudo Random Number Generator
 */
class RNG
{
public:
    /**
     * @brief generate
     * Simple random number generator function, generates a float between 0.0 and 1.0
     * @param states
     * @param i
     * @return
     */
    __device__ float generate(curandState *states, int i);
    /**
     * @brief getRandomStep
     * Gets a random step.
     * @param states
     * @param i
     * @return
     */
    __device__ float getRandomStep(curandState *states, int i);
    /**
     * @brief getRandomPoint
     * Returns a random triplet of floats (x,y,z) as an instance of Point.
     * @param states
     * @param i
     * @return
     */
    __device__ Vector getRandomDirection(curandState *states, int i);

    __device__ Point getRandomPoint(curandState *states, int i);

    __host__ Point getRandomPoint();
};
#endif