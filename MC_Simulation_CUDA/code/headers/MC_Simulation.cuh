//
// Created by mostafa on ٢‏/٨‏/٢٠٢٠.
//

#ifndef MC_SIMULATION_MC_SIMULATION_CUH
#define MC_SIMULATION_MC_SIMULATION_CUH


#include "MC_RNG.cuh"
#include "MC_Detector.cuh"
#include "MC_MLTissue.cuh"

#define NUMBER_OF_PHOTONS 500
#define THREADS_PER_BLOCK 1024
#define DETECTOR_RADIUS 1.f
#define DETECTOR_POSITION MC_Point(0.f, 0.f, 1.f)
#define DETECTOR_LOOK_AT MC_Vector(0.f, 0.f, -1.f)
#define TISSUE_RADIUS 100.f
#define TISSUE_CENTER_1 MC_Point(0.f, 0.f, 1.f)
#define TISSUE_CENTER_2 MC_Point(0.f, 0.f, 0.f)
#define A_COEFFICIENTS std::vector<float> {1.f, 6.f, 4.f, 2.f}
#define S_COEFFICIENTS std::vector<float> {100.f, 30.f, 12.f, 20.f}
#define R_INDICES std::vector<float> {0.5f, 0.3f, 0.2f, 0.8f};

class MC_Simulation {
public:
    MC_Simulation();
    void start();
private:
    MC_Detector detector;
    MC_MLTissue mlTissue;
};


#endif //MC_SIMULATION_MC_SIMULATION_CUH
