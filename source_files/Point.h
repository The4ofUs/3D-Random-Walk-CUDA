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

    /**
     * @brief setCoordinates
     * Updates the coordinates of the point to the given ones.
     * @param x
     * @param y
     * @param z
     */
    __device__
    void setCoordinates(float x, float y, float z);

    /**
     * @brief getX
     * @return
     * The X corrdinate of the point.
     */
    __device__
    float getX() const;

    /**
     * @brief getY
     * @return
     * The Y corrdinate of the point.
     */
    __device__
    float getY() const;

    /**
     * @brief getZ
     * @return
     * The Z corrdinate of the point.
     */
    __device__
    float getZ() const;

private:

    /**
     * @brief _x
     * The X corrdinate of the point.
     */
    float _x;

    /**
     * @brief _y
     * The Y corrdinate of the point.
     */
    float _y;

    /**
     * @brief _z
     * The Z corrdinate of the point.
     */
    float _z;
};

#endif  