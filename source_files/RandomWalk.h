#ifndef RANDOMWALK_H
#define RANDOMWALK_H

#include "header.h"
#include "barrierConstructor.h"
#define NUMBER_OF_ITERATIONS 500 // Number of steps, just for the demo, this number should be decided or taken later on by the user

// Returns the final position after a series of random wandering around in the 3D-space
__device__ Point randomWalk(curandState_t *states)
{
    // Creating an instance of our RandomnessGenerator class
    RandomnessGenerator randomnessGenerator;
    Ray ray;
    Point origin;
    Point currentPoint;
    Point checkPoint;
    Barrier sphere;
    int i=0;
    sphere.setRadius(10); //sphere Radius
    int Radius= sphere.getRadius();
    origin.setCoordinates(0.f, 0.f, 0.f);
    currentPoint.setCoordinates(.0f,0.f,0.f);
    checkPoint.setCoordinates(Radius, Radius, Radius);
    ray.startFrom(origin);
    while (currentPoint.getX()<Radius&& currentPoint.getY()<Radius &&currentPoint.getZ()<Radius&&currentPoint.getX()>(-Radius)&& currentPoint.getY()>(-Radius) &&currentPoint.getZ()>(-Radius)){
        
    // This for loop simulates each step the photon takes
        // Setting ray direction
        // Passing "states" and "i" as arguments for the sake of changing the seed and the state of the generator
        // for each iteration
        ray.setDirection(randomnessGenerator.getRandomPoint(states, i));

        // Setting ray step
        // Passing "states" and "i" as arguments for the sake of changing the seed and the state of the generator
        // for each iteration
        ray.setStep(randomnessGenerator.getRandomStep(states, i));

        // Move!
        ray.move();
        currentPoint=ray.getCurrentPos();
        i++;
    }
    Point proj =  sphere.getProjection(currentPoint,Radius);
    return proj;
}

#endif