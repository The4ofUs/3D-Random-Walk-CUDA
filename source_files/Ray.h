#ifndef Ray_H
#define Ray_H
#include "RandomnessGenerator.h"

class Ray
{

public:
    __device__ void startFrom(Point startingPoint); // Sets getYour starting point
    __device__ void setDirection(Point direction);  // Sets the direction of the ragetY
    __device__ void setStep(float step);            // Sets the step of movement
    __device__ Point getCurrentPos() const;
    __device__ Point getDirection() const;
    __device__ float getStep() const;
    __device__ void move(); // The point moves in the specified direction with the given step -The function relies on member attributes that getYou should set first-

private:
    Point _currentPos;
    Point _direction;
    float _step;
};
#endif