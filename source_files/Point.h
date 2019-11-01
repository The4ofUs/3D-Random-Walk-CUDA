#ifndef POINT_H
#define POINT_H

#include <helper_cuda.h>
#include <helper_functions.h>
#include <helper_timer.h>

// (x,y,z)
class Point
{

public:
    __host__ 
    void setCoordinates(float x, float y, float z);
    __host__ 
    float getX() const;
    __host__ 
    float getY() const;
    __host__
    float getZ() const;

private:
    float _x;
    float _y;
    float _z;
};

#endif 