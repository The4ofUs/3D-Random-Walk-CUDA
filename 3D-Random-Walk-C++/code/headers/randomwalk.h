#ifndef RANDOMWALK_H
#define RANDOMWALK_H

#include "RNG.h"
#include "Detector.h"
#include "Tissue.h"
#define WEIGHT_THRESHOLD 0.0001f
#define ROULETTE_CHANCE 0.1f

/**
 * @brief randomWalk
 * keeps wandering around with the photon in the 3D space
 * @return The final state of the photon
 */
Photon randomWalk(Detector detector, RNG rng, Tissue tissue, unsigned seed)
{
    Photon photon = Photon(detector.getCenter());
    bool first_step = true;
    Ray path;

    while (photon.getState() == photon.ROAMING)
    {
        if (first_step)
        {
            path = Ray(photon.getPosition(), detector.getNormal(), rng.getRandomStep(seed));
            first_step = false;
        }
        else
        {
            path = Ray(photon.getPosition(), rng.getRandomDirection(seed), rng.getRandomStep(seed));
        }

        photon.moveAlong(path);
        tissue.attenuate(photon);
        if (detector.isHit(photon, path))
        {
            photon.setState(photon.DETECTED);
            break;
        }

        if (!tissue.escaped(photon.getPosition()))
        {
            if (photon.getWeight() < WEIGHT_THRESHOLD)
            {   std::cout <<"here"<<'\n';
                rng.roulette(photon, ROULETTE_CHANCE, seed);
            }
        }
        else
        {
            photon.setState(photon.ESCAPED);
            break;
        }
    }

    return photon;
}

#endif