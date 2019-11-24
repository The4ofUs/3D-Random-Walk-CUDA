#include "RandomWalk.h"
#define N 1000          // Number of photons
#define THREADS_PER_BLOCK 256         // Threads per Block
#define BOUNDARY_RADIUS 10.0


void streamOut(Point* _cpuPoints);

__global__ void finalPosition(unsigned int seed, curandState_t* states, Point* _gpuPoints,Boundary boundary,RNG rng, int n) {
    int idx = blockIdx.x*blockDim.x+threadIdx.x;
    if(idx < n){
    curand_init(seed, idx, 0, &states[idx]);
    Point finalPos;
    finalPos = randomWalk(states, idx, boundary, rng);
    _gpuPoints[idx] = finalPos;
    }
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
    finalPosition<<<nBlocks,THREADS_PER_BLOCK>>>(time(0), states , _gpuPoints, boundary, rng, N);

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
