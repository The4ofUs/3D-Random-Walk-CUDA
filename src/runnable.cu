#include <curand.h>
#include <curand_kernel.h>
#include <iostream>

#define N 256                           // Number of photons
#define THREADS_PER_BLOCK 256           // Threads per Block
#define BOUNDARY_RADIUS 5.0

class Point{
private:
    float _x;
    float _y;
    float _z;
public:
    __device__ __host__ Point(float x, float y, float z){
        setCoordinates(x, y, z);
    }
    
    __device__ __host__  Point(){
        setCoordinates(0.f, 0.f, 0.f);
    }

    __device__ __host__
 void setCoordinates(float x, float y, float z)
{
    this->_x = x;
    this->_y = y;
    this->_z = z;
}
__device__ __host__  float getX() const { return this->_x; }

__device__ __host__   float getY() const { return this->_y; }

__device__ __host__   float getZ() const { return this->_z; }

__device__ __host__  
    Point add(Point point){
        float result_x = this->_x + point.getX();
        float result_y = this->_y + point.getY();
        float result_z = this->_z + point.getZ();
        return Point( result_x, result_y, result_z );
    }

    __device__ __host__  
    Point subtract(Point point){
        float result_x = this->_x - point.getX();
        float result_y = this->_y - point.getY();
        float result_z = this->_z - point.getZ();
        return Point( result_x, result_y, result_z );
    }
};

class RNG{
private:
__device__  float generate( curandState* globalState, int i) 
{
    curandState localState = globalState[i];
    float random = curand_uniform( &localState );
    globalState[i] = localState;
    return random;
}
public:
__device__   float getRandomStep( curandState* globalState , int i) { 
    float step = 0.f;       // Intialize for step value
    step = generate (globalState, i);
    return step;
 } 

__device__  Point getRandomPoint( curandState* globalState , int i)
{

    float u = generate (globalState , i);
    float v = generate (globalState, i);
    
    float theta = 2 * M_PI * u;
    float phi = acos(1 - 2 * v);

    // Transforming into the cartesian space
    float x = sin(phi) * cos(theta);
    float y = sin(phi) * sin(theta);
    float z = cos(phi);

    return Point(x,y,z);
}
};

class Ray{
private:
    Point _prevPos;
    Point _currentPos;
    Point _direction;
    float _step;
public:
    __device__ Ray(Point startingPoint, Point direction){
        this->_currentPos.setCoordinates(startingPoint.getX(), startingPoint.getY(), startingPoint.getZ());
        this->_direction.setCoordinates(direction.getX(), direction.getY(), direction.getZ());
    }
    
    __device__ void setDirection(Point direction) { this->_direction.setCoordinates(direction.getX(), direction.getY(), direction.getZ()); }
    
    __device__ void setStep(float step) { this->_step = step; }
    
    __device__ Point getCurrentPos() const { return this->_currentPos; }
    
    __device__ Point getDirection() const { return this->_direction; }
    
    __device__ Point getPrevPos() const { return this->_prevPos; }
    
    __device__ float getStep() const { return this->_step; }
    
    __device__ void move(Point direction, float step) // The point moves in the specified direction with the given step
    {
        this->_prevPos = this->_currentPos;
        this->_direction = direction;
        this->_step = step;
        float newX = this->_currentPos.getX() + (direction.getX() * step);
        float newY = this->_currentPos.getY() + (direction.getY() * step);
        float newZ = this->_currentPos.getZ() + (direction.getZ() * step);
        this->_currentPos.setCoordinates(newX, newY, newZ);
    }
};

class Boundary{
private:
    float _radius;
    Point _center;

    __device__
    float dot(Point point1, Point point2){return point1.getX()*point2.getX() + point1.getY()*point2.getY() + point1.getZ()*point2.getZ();}
    
public:
    __device__ __host__ Boundary(float r, Point c){
        _radius = r;
        _center = c;
    }
    
    __device__ bool isCrossed(Ray ray){
        float absDistance = (float) sqrtf((float) powf(ray.getCurrentPos().getX(),2)
                            + (float) powf(ray.getCurrentPos().getY(),2) 
                            + (float) powf(ray.getCurrentPos().getZ(),2));
        if(absDistance >= _radius){
            return true;
        } else {
            return false;
        }
    };
    
    
    __device__ Point getIntersectionPoint(Ray ray){
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
};



/**
 * @brief randomWalk
 * keeps wandering around with the photon in the 3D space
 * @return The Point where the Photon hits the Boundary
 */
 __device__ Point randomWalk(curandState_t *states, int idx, Boundary boundary, RNG rng)
 {
     Ray ray = Ray(Point(0.f, 0.f, 0.f), Point(0.f, 0.f, 0.f));
 
     while (!boundary.isCrossed(ray))
     {
         ray.move(rng.getRandomPoint(states, idx), rng.getRandomStep(states, idx));
     }
     return boundary.getIntersectionPoint(ray);
 }





void streamOut(Point* _cpuPoints);

__global__ void finalPosition(unsigned int seed, curandState_t* states, Point* _gpuPoints,Boundary boundary,RNG rng) {
    int idx = blockIdx.x*blockDim.x+threadIdx.x;
    curand_init(seed, idx, 0, &states[idx]);
    Point finalPos;
    finalPos = randomWalk(states, idx, boundary, rng);
    _gpuPoints[idx] = finalPos;
}

  int main() {

    int nBlocks = N/THREADS_PER_BLOCK + 1;
 
    curandState_t* states;
    cudaMalloc((void**) &states, N * sizeof(curandState_t));

// Allocate host memory for final positions
    Point * _cpuPoints= (Point*)malloc(sizeof(Point) * N);

// Allocate device  memory for final positions
    Point* _gpuPoints = nullptr;
    cudaMalloc((void**) &_gpuPoints, N * sizeof(Point));

// Initializing the Boundary and the Random Number Generator
    Boundary boundary = Boundary(BOUNDARY_RADIUS, Point(0.f, 0.f, 0.f));
    RNG rng;
  
// Call Kernel
    finalPosition<<<nBlocks,THREADS_PER_BLOCK>>>(time(0), states , _gpuPoints, boundary, rng);

// Copy device data to host memory to stream them out
    cudaMemcpy(_cpuPoints, _gpuPoints, N* sizeof( Point), cudaMemcpyDeviceToHost);


    streamOut (&_cpuPoints[0]);

    free(_cpuPoints);
    cudaFree(_gpuPoints);


    return 0;

}

void streamOut(Point* _cpuPoints)  
{
    FILE *output;
    output = fopen("output.csv", "w");

    for (int i = 0; i < N; i++)
    {
        // Streaming out my output in a log file
        float absDistance = (float) sqrtf((float) powf(_cpuPoints[i].getX(), 2) 
                            + (float) powf(_cpuPoints[i].getY(), 2) 
                            + (float) powf(_cpuPoints[i].getZ(), 2));
        fprintf(output, "%f,%f,%f,%f\n", _cpuPoints[i].getX(), _cpuPoints[i].getY(), _cpuPoints[i].getZ(), absDistance);
    }
}
