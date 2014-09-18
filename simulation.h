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
    Particle(Eigen::Vector2d pos, double mass, bool isFixed, double radius) : pos(pos), mass(mass), fixed(isFixed), radius(radius)
    {
        vel.setZero();
        id = idGenerator;
        idGenerator++;
    }
    int id;
    int posInVec;
    Eigen::Vector2d pos;
    Eigen::Vector2d vel;
    double mass;
    double radius;
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
    SpringComponent(int p1, int p2, double restLength, int posP1, int posP2): p1Id(p1), p2Id(p2), restLength(restLength), p1PosInQVector(posP1), p2PosInQVector(posP2)
    {
    }
    int p1Id;
    int p2Id;
    int p1PosInQVector;
    int p2PosInQVector;
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
    void updateConfigVectorqPrev(Eigen::VectorXd qConfig);
    void updateConfigVectorVel();
    void updateParticlePosFromQ(Eigen::VectorXd qConfig);
    void updateParticleVelFromV(Eigen::VectorXd vConfig);

    double euclideanDistanceFormula(double x1, double y1, double x2, double y2);
    double distanceFromInfiniteLine(Eigen::Vector2d q1, Eigen::Vector2d q2, Eigen::Vector2d p);
    double distanceFromFiniteLine(Eigen::Vector2d q1, Eigen::Vector2d q2, Eigen::Vector2d p);

    Eigen::SparseMatrix<double> getMassInverseMatrix();

    Eigen::SparseMatrix<double> generateAllGradients(Eigen::VectorXd xTildaVector, Eigen::VectorXd qPrev_);
    Eigen::VectorXd generateAllForces(Eigen::VectorXd qConfig, Eigen::VectorXd qPrevConfig);

    Eigen::VectorXd generateGrForce(Eigen::VectorXd qConfig);
    Eigen::SparseMatrix<double> generateGravityForceGradient(Eigen::VectorXd qConfig);
    Eigen::VectorXd generateSpringForce(Eigen::VectorXd qConfig);
    Eigen::SparseMatrix<double> generateSpringForceGradient(Eigen::VectorXd qConfig);
    Eigen::VectorXd generateViscousDampingForce(Eigen::VectorXd qConfig, Eigen::VectorXd qPrevConfig);
    Eigen::SparseMatrix<double> generateViscousDampingGradient(Eigen::VectorXd qConfig, Eigen::VectorXd qPrevConfig);

    void snapSprings();
    void checkSawCollisions();
    void removeOutsideParticles();
    void removeSpringsWithParticle(int particleId);

    void reAlignParticlesAndSprings();
    void reAlignPrevQVector(int index);


private:
    const SimParameters &params_;
    QMutex renderLock_;

    int flip;
    double time_;
    const double floorHeight_;
    std::vector< std::vector< Particle > > ConnectedParticles_;
    std::vector<SpringComponent> springs_;
    std::vector<Particle> particles_;
    std::vector<Saw> saws_;
    Eigen::VectorXd qVector_;
    Eigen::VectorXd qPrevVector_;
    Eigen::VectorXd velocityVector_;
};

#endif // SIMULATION_H
