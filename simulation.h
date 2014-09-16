#ifndef SIMULATION_H
#define SIMULATION_H

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <set>
#include <QMutex>

typedef Eigen::Triplet<double> Tr;

class SimParameters;

static int idGenerator = 0;
struct Particle
{
public:
    Particle(Eigen::Vector2d pos, double mass, bool isFixed) : pos(pos), mass(mass), fixed(isFixed)
    {
        vel.setZero();
        id = idGenerator;
        idGenerator++;
    }
    int id;
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double mass;
    bool fixed;
};

struct Saw
{
public:
    Saw(Eigen::Vector2d pos, double sawRadius) : pos(pos), radius(sawRadius)
    {
        vel.setZero();
    }
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double radius;
    bool fixed;
};

struct SpringComponent
{
public:
    SpringComponent(int p1, int p2, double restLength): p1Id(p1), p2Id(p2), restLength(restLength)
    {
    }
    int p1Id;
    int p2Id;
    double restLength;

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

    void updateConfigVectorq();
    void updateConfigVectorVel();
    void updateParticlePosFromQ();
    void updateParticleVelFromV();

    void generateGrForce();
    void generateSpringForce();
    void combineForces();

    void explicitEuler();


private:
    const SimParameters &params_;
    QMutex renderLock_;

    double time_;
    const double floorHeight_;
    std::vector< std::vector< Particle > > ConnectedParticles_;
    std::vector<SpringComponent> springs_;
    std::vector<Particle> particles_;
    std::vector<Saw> saws_;
    Eigen::VectorXd qVector_;
    Eigen::VectorXd velocityVector_;
    Eigen::VectorXd forceVector_;
    Eigen::VectorXd gravityForceVector_;
    Eigen::VectorXd springForceVector_;
};

#endif // SIMULATION_H
