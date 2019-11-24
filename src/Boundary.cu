#include "Boundary.h"
using namespace std;

__device__
float Boundary::dot(Point point1, Point point2){return point1.getX()*point2.getX() + point1.getY()*point2.getY() + point1.getZ()*point2.getZ();}


__device__ __host__ Boundary::Boundary(float r, Point c){
    _radius = r;
    _center = c;
}

__device__ void Boundary::setRadius(float r){_radius = r;}

__device__ float Boundary::getRadius() const {return _radius;}

__device__ void Boundary::setCenter(Point c){_center = c;}

__device__ Point Boundary::getCenter() const {return _center;}

__device__ bool Boundary::isCrossed(Ray ray){
    float absDistance = (float) sqrtf((float) powf(ray.getCurrentPos().getX(),2)
                        + (float) powf(ray.getCurrentPos().getY(),2) 
                        + (float) powf(ray.getCurrentPos().getZ(),2));
    if(absDistance >= _radius){
        return true;
    } else {
        return false;
    }
}


__device__ Point Boundary::getIntersectionPoint(Ray ray){
        /**
            P(t) = A + tB
            P(t) is a point on the ray 
            A is the ray origin
            B is the ray direction
            t is a parameter used to move away from ray origin
            S = P - Center
            ||S||^2 = r^2
            Sphere: dot(S,S) = r^2
            Ray: P(t) = A + tB
            Combined: dot((A + tB - Center),(A + tB - Center)) = r^2
            in Quadratic form: t^2.dot(B,B) + 2t.dot(B, A - C) + dot(A - C, A - C) - r^2 = 0
            let a = dot(B,B)
                b = 2.dot(B, A - C)
                c = dot(A - C, A - C) - r^2
            t1, t2 = (-b (+/-) sqrt(b^2 - 4ac) / 2a)
        */
        Point A = ray.getPrevPos();
        Point B = ray.getDirection();
        Point S = A.add(_center);
        Point A_C = A.subtract(_center);
        float a = dot(B, B);
        float b = 2.0 * dot(B, A_C);
        float c = dot(A_C, A_C) - _radius*_radius;
        float discriminant = b*b - 4*a*c;
        float t1 = (-b + sqrtf(discriminant)) / (2.0*a);
        float t2 = (-b - sqrtf(discriminant)) / (2.0*a);
        float t;

        if(t1 < 0){
            t = t2;
        } else {
            t = t1;
        }

        return Point((A.getX()+B.getX()*t),(A.getY()+B.getY()*t),(A.getZ()+B.getZ()*t));
}


