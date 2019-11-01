#include "Point.h"
#include <iostream>
#include <vector>


__device__ void Point::setCoordinates(float x, float y, float z)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
}
__device__ float Point::getX() const { return this->_x; }

__device__ float Point::getY() const { return this->_y; }

__device__ float Point::getZ() const { return this->_z; }

