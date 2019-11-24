#include "RNG.h"

__device__  float RNG::generate( curandState* globalState, int i) 
{
    curandState localState = globalState[i];
    float random = curand_uniform( &localState );
    globalState[i] = localState;
    return random;
}

__device__   float RNG::getRandomStep( curandState* globalState , int i) { 
    float step = 0.f;       // Intialize for step value
    step = generate (globalState, i);
    return step;
 } 

__device__  Point RNG::getRandomPoint( curandState* globalState , int i)
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