#include "header.h"
#define NUMBER_OF_ITERATIONS 500 // Number of steps, just for the demo, this number should be decided or taken later on by the user


// Returns the final position after a series of random wandering around in the 3D-space
__device__ Point randomWalk()
{
    // Creating an instance of our RandomnessGenerator class
    RandomnessGenerator randomnessGenerator;
    Ray ray;
    Point origin;
    origin.setCoordinates(0.f, 0.f, 0.f);
    ray.startFrom(origin);

    // This for loop simulates each step the photon takes
    for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
    {
        // Setting ray direction
        ray.setDirection(randomnessGenerator.getRandomPoint());

        // Setting ray step
        ray.setStep(randomnessGenerator.getRandomStep());

        // Move!
        ray.move();

    }

    return ray.getCurrentPos();
}