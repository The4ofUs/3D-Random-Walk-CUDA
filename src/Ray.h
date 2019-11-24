#ifndef Ray_H
#define Ray_H
#include "Point.h"

/**
 * @brief The Ray class
 */
class Ray
{

public:
    /**
     * @brief Constructor 
     * @param startingPoint
     * Starting origin of the ray
     * @param direction
     * Direction of the ray
     */
__device__ Ray(Point startingPoint, Point direction);

    /**
     * @brief setDirection
     * Sets the direction of the ray
     * @param direction
     * The direction of the ray.
     */
__device__ void setDirection(Point direction);

    /**
     * @brief setStep
     * Sets the step along the ray.
     * @param step
     * Step value.
     */
__device__ void setStep(float step);

    /**
     * @brief getCurrentPos
     * @return
     * The current position of the ray.
     */
__device__ Point getCurrentPos() const;

    /**
     * @brief getDirection
     * @return
     * The current direction of the ray.
     */
__device__ Point getDirection() const;

    /**
     * @brief getPrevPos
     * @return
     * The previous position of the ray.
     */
__device__ Point getPrevPos() const;

    /**
     * @brief getStep
     * @return
     * The current step along the ray.
     */
__device__ float getStep() const;

    /**
     * @brief move
     * The point moves in the specified direction with the given step.
     * @param direction
     * The vector that the ray should move on
     * @param step
     * The magnitude of the movement
     */
__device__ void move(Point direction, float step);

private:
    /**
     * @brief _prevPos
     * The previous position of the ray.
     */
    Point _prevPos;
    /**
     * @brief _currentPos
     * The current position of the ray.
     */
    Point _currentPos;
    /**
     * @brief _direction
     * The current direction unit vector of the ray.
     */
    Point _direction;
    /**
     * @brief _step
     * The step along the ray.
     */
    float _step;
};
#endif