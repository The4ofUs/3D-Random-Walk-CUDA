#ifndef POINT_H
#define POINT_H

#include <helper_cuda.h>
#include <helper_functions.h>
#include <helper_timer.h>

/**
 * @brief The Point class
 * A point in the space defined by the cartesian coordinates (X, Y, Z)
 */
class Point
{

public:
__device__ __host__ Point(float x, float y, float z){
    setCoordinates(x, y, z);
}

__device__ __host__ Point(){
    setCoordinates(0.f, 0.f, 0.f);
}

    /**
     * @brief setCoordinates
     * Updates the coordinates of the point to the given ones.
     * @param x
     * @param y
     * @param z
     */
__device__ __host__
    void setCoordinates(float x, float y, float z);


    /**
     * @brief getX
     * @return
     * The X coordinate of the point.
     */
__device__ __host__
    float getX() const;


    /**
     * @brief getY
     * @return
     * The Y coordinate of the point.
     */
__device__ __host__
    float getY() const;


    /**
     * @brief getZ
     * @return
     * The Z coordinate of the point.
     */
__device__ __host__
    float getZ() const;

    /**
     * @brief add
     * @return
     * The summation of the 2 points
     */
__device__ __host__
    Point add(Point point);

    /**
     * @brief subtract
     * @return
     * The subtraction of the 2 points
     */
    __device__ __host__
    Point subtract(Point point);



private:

    /**
     * @brief _x
     * The X coordinate of the point.
     */
    float _x;

    /**
     * @brief _y
     * The Y coordinate of the point.
     */
    float _y;

    /**
     * @brief _z
     * The Z coordinate of the point.
     */
    float _z;
};


#endif 