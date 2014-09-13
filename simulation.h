#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>
#include <QMutex>

typedef Eigen::Triplet<double> Tr;

class SimParameters;

struct Particle
{
public:
    Particle(Eigen::Vector2d pos, double mass, bool isFixed) : pos(pos), mass(mass), fixed(isFixed)
    {
        vel.setZero();
    }

    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double mass;
    bool fixed;
};

class Simulation
{
public:
    Simulation(const SimParameters &params);

    void addParticle(double x, double y);
    void addSaw(double x, double y);

    void takeSimulationStep();
    void render();
    void clearScene();

private:
    const SimParameters &params_;
    QMutex renderLock_;

    double time_;
    std::vector<Particle> particles_;
};

#endif // SIMULATION_H
