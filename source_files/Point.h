#ifndef POINT_H
#define POINT_H

#include <helper_cuda.h>
#include <helper_functions.h>
#include <helper_timer.h>

// (x,y,z)
class Point
{

public:
    __device__ 
    void setCoordinates(float x, float y, float z);
    __device__ 
    float getX() const;
    __device__ 
    float getY() const;
    __device__
    float getZ() const;

private:
    float _x;
    float _y;
    float _z;
};

#endif 