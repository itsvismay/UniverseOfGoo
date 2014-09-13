#ifndef SIMPARAMETERS_H
#define SIMPARAMETERS_H

struct SimParameters
{
    SimParameters();

    enum TimeIntegrator {TI_EXPLICIT_EULER, TI_IMPLICIT_EULER, TI_IMPLICIT_MIDPOINT, TI_VELOCITY_VERLET};
    enum ClickMode {CM_ADDPARTICLE, CM_ADDSAW};

    const static int F_GRAVITY = 1;
    const static int F_SPRINGS = 2;
    const static int F_FLOOR   = 4;
    const static int F_DAMPING = 8;


    bool simRunning;
    TimeIntegrator integrator;
    double timeStep;
    double NewtonTolerance;
    int NewtonMaxIters;

    int activeForces;
    double gravityG;
    double springStiffness;
    double maxSpringStrain;
    double dampingStiffness;

    ClickMode clickMode;
    double particleMass;
    double maxSpringDist;
    bool particleFixed;
    double sawRadius;
};

#endif // SIMPARAMETERS_H
