#include "Ray.h"
using namespace std;

__device__ Ray::Ray(Point startingPoint, Point direction){
    this->_currentPos.setCoordinates(startingPoint.getX(), startingPoint.getY(), startingPoint.getZ());
    this->_direction.setCoordinates(direction.getX(), direction.getY(), direction.getZ());
}

__device__ void Ray::setDirection(Point direction) { this->_direction.setCoordinates(direction.getX(), direction.getY(), direction.getZ()); }

__device__ void Ray::setStep(float step) { this->_step = step; }

__device__ Point Ray::getCurrentPos() const { return this->_currentPos; }

__device__ Point Ray::getDirection() const { return this->_direction; }

__device__ Point Ray::getPrevPos() const { return this->_prevPos; }

__device__ float Ray::getStep() const { return this->_step; }

__device__ void Ray::move(Point direction, float step) // The point moves in the specified direction with the given step
{
    this->_prevPos = this->_currentPos;
    this->_direction = direction;
    this->_step = step;
    float newX = this->_currentPos.getX() + (direction.getX() * step);
    float newY = this->_currentPos.getY() + (direction.getY() * step);
    float newZ = this->_currentPos.getZ() + (direction.getZ() * step);
    this->_currentPos.setCoordinates(newX, newY, newZ);
}
