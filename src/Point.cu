#include "Point.h"

__device__
 void Point::setCoordinates(float x, float y, float z)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
}
__device__ float Point::getX() const { return this->_x; }

__device__  float Point::getY() const { return this->_y; }

__device__  float Point::getZ() const { return this->_z; }

__device__ __host__
    Point Point::add(Point point){
        float result_x = this->_x + point.getX();
        float result_y = this->_y + point.getY();
        float result_z = this->_z + point.getZ();
        return Point( result_x, result_y, result_z );
    }

    __device__ __host__
    Point Point::subtract(Point point){
        float result_x = this->_x - point.getX();
        float result_y = this->_y - point.getY();
        float result_z = this->_z - point.getZ();
        return Point( result_x, result_y, result_z );
    }

