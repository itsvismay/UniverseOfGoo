#include "simparameters.h"

SimParameters::SimParameters()
{
    simRunning = false;
    integrator = TI_EXPLICIT_EULER;
    timeStep = 0.001;
    NewtonMaxIters = 20;
    NewtonTolerance = 1e-8;

    activeForces = F_GRAVITY | F_SPRINGS | F_FLOOR | F_DAMPING;
    gravityG = -9.8;
    springStiffness = 100;
    maxSpringStrain = 0.2;
    dampingStiffness = 1.0;

    clickMode = CM_ADDPARTICLE, CM_ADDSAW;
    particleMass = 1.0;
    maxSpringDist = 0.25;
    particleFixed = false;

    sawRadius= 0.01;
}
