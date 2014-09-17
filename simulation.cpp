
#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include <iostream>

const double PI = 3.1415926535898;

using namespace Eigen;
using namespace std;

Simulation::Simulation(const SimParameters &params) : params_(params), time_(0), floorHeight_(-0.5)
{
}

void Simulation::render()
{

    double baseradius = 0.02;
    double pulsefactor = 0.1;
    double pulsespeed = 50.0;

    int numcirclewedges = 20;

    renderLock_.lock();
    {
        if((params_.activeForces & 4)==4)
        {
            //draw floor
            glColor3f(0, 1, 0);
            glBegin(GL_QUADS);
            {
                glVertex2f(-1,floorHeight_);
                glVertex2f(1,floorHeight_);
                glVertex2f(1,-1);
                glVertex2f(-1,-1);
            }
            glEnd();
        }


        for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            double radius = baseradius*sqrt(it->mass);
            radius *= (1.0 + pulsefactor*sin(pulsespeed*time_));

            glColor3f(0,0,0);
            if (it->fixed)
            {
                glColor3f(1,0,0);
            }
            glBegin(GL_TRIANGLE_FAN);
            {
                glVertex2f(it->pos[0], it->pos[1]);
                for(int i=0; i<=numcirclewedges; i++)
                {
                    glVertex2f(it->pos[0] + radius * cos(2*PI*i/numcirclewedges),
                               it->pos[1] + radius * sin(2*PI*i/numcirclewedges));
                }
            }
            glEnd();
        }

        for(vector<Saw>::iterator it = saws_.begin(); it != saws_.end(); ++it)
        {
            double radius = it->radius;
            glColor3f(0.5, 0.5, 0.5);
            glBegin(GL_TRIANGLE_FAN);
            {
                glVertex2f(it->pos[0], it->pos[1]);
                for(int i=0; i<=numcirclewedges; i++)
                {
                    glVertex2f(it->pos[0]+ radius * cos(2*PI*i/numcirclewedges),
                               it->pos[1] + radius * sin(2*PI*i/numcirclewedges));
                }
            }
            glEnd();
        }

        for(vector<SpringComponent>::iterator it = springs_.begin(); it!=springs_.end(); ++it)
        {
            glColor3f(0,0,1);
            glBegin(GL_LINES);
            {
                glVertex2f(particles_[it->p1Id].pos[0],particles_[it->p1Id].pos[1]);
                glVertex2f(particles_[it->p2Id].pos[0], particles_[it->p2Id].pos[1]);
            }
            glEnd();
        }
    }
    renderLock_.unlock();
}

void Simulation::takeSimulationStep()
{
    time_ += params_.timeStep;
    updateConfigVectorq();
    updateConfigVectorVel();
    if (params_.integrator == params_.TI_EXPLICIT_EULER)
    {
        int particleId = 0;
        Eigen::VectorXd totalForceVector = generateAllForces(qVector_, qPrevVector_);
        Eigen::VectorXd massInvForceVector = getMassInverseMatrix()*totalForceVector;
        updateConfigVectorqPrev(qVector_);
        for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            particleId = it->id;
            qVector_[particleId*2] = qVector_[particleId*2] + params_.timeStep*velocityVector_[particleId*2];
            qVector_[(particleId*2)+1] = qVector_[(particleId*2)+1] + params_.timeStep*velocityVector_[(particleId*2)+1];
            velocityVector_[particleId*2] = velocityVector_[particleId*2] + params_.timeStep*massInvForceVector[particleId*2];
            velocityVector_[(particleId*2)+1] = velocityVector_[(particleId*2)+1] + params_.timeStep*massInvForceVector[(particleId*2)+1];
        }
    }
    else if (params_.integrator == params_.TI_VELOCITY_VERLET)
    {
        // Update the q(i+1).
        int particleId = 0;
        for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            particleId = it->id;
            qVector_[particleId*2] = qVector_[particleId*2] + params_.timeStep*velocityVector_[particleId*2];
            qVector_[(particleId*2)+1] = qVector_[(particleId*2)+1] + params_.timeStep*velocityVector_[(particleId*2)+1];
        }
        updateConfigVectorqPrev(qVector_);
        Eigen::VectorXd totalForceVector = generateAllForces(qVector_, qPrevVector_);
        Eigen::VectorXd massInvForceVector = getMassInverseMatrix()*totalForceVector;
        particleId = 0;
        // Use the q(i+1) to generate Force and update velocitites
        for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            particleId = it->id;
            velocityVector_[particleId*2] = velocityVector_[particleId*2] + params_.timeStep*massInvForceVector[particleId*2];
            velocityVector_[(particleId*2)+1] = velocityVector_[(particleId*2)+1] + params_.timeStep*massInvForceVector[(particleId*2)+1];
        }
    }
    else if (params_.integrator == params_.TI_IMPLICIT_EULER)
    {
        int i;
        Eigen::VectorXd deltaX;
        Eigen::VectorXd xTildaVector = qVector_;
        Eigen::VectorXd Fval;
        Eigen::SparseMatrix<double> deltaF(particles_.size()*2, particles_.size()*2);
        cout << "\n Q vector in:"<< qVector_;
        cout << "\n XTilda in :"<<xTildaVector;
        updateConfigVectorqPrev(qVector_);
        Eigen::VectorXd totalForceVector = generateAllForces(xTildaVector, qPrevVector_);
        Eigen::VectorXd massInvForceVector = getMassInverseMatrix()*totalForceVector;
        Fval = xTildaVector - qVector_ - params_.timeStep*velocityVector_ - (params_.timeStep*params_.timeStep)*massInvForceVector;
        for (i=0; i<params_.NewtonMaxIters; i++)
        {
//            totalForceVector = generateAllForces(xTildaVector, qPrevVector_);
            Eigen::SparseMatrix<double> I(particles_.size()*2, particles_.size()*2);
            I.setIdentity();
            deltaF = params_.timeStep*params_.timeStep*getMassInverseMatrix()*(generateAllGradients(xTildaVector, qPrevVector_))+I;
            deltaF = deltaF*(-1);
            BiCGSTAB< SparseMatrix<double> > solver;
            solver.compute(deltaF);
            deltaX = solver.solve(Fval);
            cout << "\nDeltaX:"<< deltaX;
            xTildaVector = xTildaVector + deltaX;
            totalForceVector = generateAllForces(xTildaVector, qPrevVector_);
            massInvForceVector = getMassInverseMatrix()*totalForceVector;
            Fval = xTildaVector - qVector_ - params_.timeStep*velocityVector_ - (params_.timeStep*params_.timeStep)*massInvForceVector;
            if (Fval.norm() < params_.NewtonTolerance)
            {
                break;
            }
        }
        cout << "\n Q vector out :"<< qVector_;
        cout << "\n XTilda out :"<<xTildaVector;
        velocityVector_ = (xTildaVector - qVector_)/params_.timeStep;
        qVector_ = xTildaVector;
    }
    updateParticlePosFromQ(qVector_);
    updateParticleVelFromV(velocityVector_);
    snapSprings();
    checkSawCollisions();
}

void Simulation::addParticle(double x, double y)
{
    renderLock_.lock();
    {
        Vector2d newpos(x,y);
        double mass = params_.particleMass;
        if(!params_.F_FLOOR || y>floorHeight_)
        {
            particles_.push_back(Particle(newpos, mass, params_.particleFixed));
            //Build the configuration vectors.
            qVector_.resize(qVector_.rows()+2);
            qPrevVector_.resize(qVector_.rows()+2);
            velocityVector_.resize(qVector_.rows()+2);
            qVector_[qVector_.rows()-2] = newpos[0];
            qVector_[qVector_.rows()-1] = newpos[1];
            qPrevVector_[qPrevVector_.rows()-2] = newpos[0];
            qPrevVector_[qPrevVector_.rows()-1] = newpos[1];
            velocityVector_[velocityVector_.rows()-2] = 0;
            velocityVector_[velocityVector_.rows()-1] = 0;
            /*if distance between new particle and an existing particle is within maxSpringDistance,
            connect those particles with a spring*/
            for(int i = 0; i<particles_.size()-1; i++)
            {
                double x1x2 = (particles_[i].pos[0] - x);
                double y1y2 = (particles_[i].pos[1] - y);
                double distance = sqrt(((x1x2*x1x2)+(y1y2*y1y2)));
                if(params_.maxSpringDist >= distance)
                {
                    springs_.push_back(SpringComponent(particles_[i].id, particles_[particles_.size()-1].id, distance));
                }
            }
        }

    }
    renderLock_.unlock();
}

Eigen::SparseMatrix<double> Simulation::getMassInverseMatrix()
{
    Eigen::SparseMatrix<double> massInverseMatrix(particles_.size()*2, particles_.size()*2);
    massInverseMatrix.setZero();
    int particleId = 0;
    double mass;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        if (it->fixed)
        {
            mass = numeric_limits<double>::infinity();
        }
        else
        {
            mass = it->mass;
        }
        massInverseMatrix.coeffRef(particleId*2,particleId*2) = 1/mass;
        massInverseMatrix.coeffRef((particleId*2)+1,(particleId*2)+1) = 1/mass;
    }
    return massInverseMatrix;
}

void Simulation::updateConfigVectorqPrev(Eigen::VectorXd qConfig)
{
    int particleId;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        qPrevVector_[particleId*2] = qConfig[particleId*2];
        qPrevVector_[(particleId*2)+1] = qConfig[(particleId*2)+1];
    }
}

void Simulation::updateConfigVectorq()
{
    int particleId;
    qVector_.resize(0);
    qVector_.resize(particles_.size()*2);
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        qVector_[particleId*2] = it->pos[0];
        qVector_[(particleId*2) + 1] = it->pos[1];
    }
}

void Simulation::updateConfigVectorVel()
{
    int particleId;
    velocityVector_.resize(0);
    velocityVector_.resize(particles_.size()*2);
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        velocityVector_[particleId*2] = it->vel[0];
        velocityVector_[(particleId*2) + 1] = it->vel[1];
    }
}

void Simulation::updateParticlePosFromQ(Eigen::VectorXd qConfig)
{
    int particleId;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        it->pos[0] = qConfig[particleId*2];
        it->pos[1] = qConfig[(particleId*2) + 1];
    }
}

void Simulation::updateParticleVelFromV(Eigen::VectorXd vConfig)
{
    int particleId = 0;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        it->vel[0] = vConfig[particleId*2];
        it->vel[1] = vConfig[(particleId*2) + 1];
    }
}

Eigen::VectorXd Simulation::generateAllForces(Eigen::VectorXd qConfig, Eigen::VectorXd qPrevConfig)
{
    return generateGrForce(qConfig) + generateSpringForce(qConfig) + generateViscousDampingForce(qConfig, qPrevConfig);
}

Eigen::SparseMatrix<double> Simulation::generateAllGradients(Eigen::VectorXd xTildaVector, Eigen::VectorXd qPrev_)
{
    return generateGravityForceGradient(xTildaVector)+generateSpringForceGradient(xTildaVector)+generateViscousDampingGradient(xTildaVector, qPrev_);
}

Eigen::VectorXd Simulation::generateGrForce(Eigen::VectorXd qConfig)
{
    Eigen::VectorXd gravityForceVector(qConfig.rows());
    gravityForceVector.setZero();
    int particleId = 0;
    if ((params_.activeForces & 1)==0)
    {
        return gravityForceVector;
    }
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        if (it->fixed)
        {
            gravityForceVector[particleId*2] = 0.0;
            gravityForceVector[(particleId*2)+1] = 0.0;
        }
        else
        {
            gravityForceVector[particleId*2] = 0.0;
            gravityForceVector[(particleId*2)+1] = it->mass*params_.gravityG;
        }
    }
    return gravityForceVector;
}

Eigen::SparseMatrix<double> Simulation::generateGravityForceGradient(Eigen::VectorXd qConfig)
{
    Eigen::SparseMatrix<double> gravityForceGradient(particles_.size()*2, particles_.size()*2);
    gravityForceGradient.setZero();
    return gravityForceGradient;
}

Eigen::VectorXd Simulation::generateSpringForce(Eigen::VectorXd qConfig)
{
    double Kij;
    double L;
    double p1p2euclideanDistance;
    double Xi;
    double Xj;
    double Yi;
    double Yj;
    double tempForce;
    int pi;
    int pj;
    Eigen::VectorXd springForceVector(qConfig.rows());
    springForceVector.setZero();
    if ((params_.activeForces & 2)==0)
    {
        return springForceVector;
    }
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    {
        pi = it->p1Id;
        pj = it->p2Id;
        L =  it->restLength;
        Kij = params_.springStiffness/L;
        Xi = qConfig[pi*2];
        Yi = qConfig[(pi*2)+1];
        Xj = qConfig[pj*2];
        Yj = qConfig[(pj*2)+1];
        p1p2euclideanDistance = sqrt((Xi-Xj)*(Xi-Xj) + (Yi-Yj)*(Yi-Yj));
        tempForce = (Kij*(p1p2euclideanDistance-L))/p1p2euclideanDistance;
        springForceVector[pi*2] += tempForce*(Xj-Xi);
        springForceVector[(pi*2)+1] += tempForce*(Yj-Yi);
        springForceVector[pj*2] += tempForce*(Xi-Xj);
        springForceVector[(pj*2)+1] += tempForce*(Yi-Yj);
    }
    return springForceVector;
}

Eigen::SparseMatrix<double> Simulation::generateSpringForceGradient(Eigen::VectorXd qConfig)
{
    Eigen::SparseMatrix<double> springForceGradient(particles_.size()*2, particles_.size()*2);
    springForceGradient.setZero();
    vector< Triplet<double> > tripletList;
    tripletList.reserve(particles_.size()*2);
    double Kij;
    double L;
    double p1p2euclideanDistance;
    double Xi;
    double Xj;
    double Yi;
    double Yj;
    double dfxidxi;
    double dfxidxj;
    double dfxidyi;
    double dfxidyj;
    double dfxjdxi;
    double dfxjdxj;
    double dfxjdyi;
    double dfxjdyj;
    double dfyidxi;
    double dfyidxj;
    double dfyidyi;
    double dfyidyj;
    double dfyjdxi;
    double dfyjdxj;
    double dfyjdyi;
    double dfyjdyj;
    int Pi;
    int Pj;
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    {
        Pi = it->p1Id;
        Pj = it->p2Id;
        Xi = qConfig[Pi*2];
        Yi = qConfig[(Pi*2)+1];
        Xj = qConfig[Pj*2];
        Yj = qConfig[(Pj*2)+1];

        L = it->restLength;
        Kij = params_.springStiffness/L;
        p1p2euclideanDistance = euclideanDistanceFormula(Xi, Yi, Xj, Yj);

        dfxidxi = -Kij*(1-(L/sqrt(p1p2euclideanDistance))) - Kij*L*(pow(p1p2euclideanDistance, -1.5))*(Xj-Xi)*(Xj-Xi);
        dfxidyi = -Kij*L*(pow(p1p2euclideanDistance, -1.5))*(Xj-Xi)*(Yj-Yi);
        dfxidxj = -dfxidxi;
        dfxidyj = -dfxidyi;

        dfyidxi = dfxidyi;
        dfyidyi = -Kij*(1-(L/sqrt(p1p2euclideanDistance))) - Kij*L*(pow(p1p2euclideanDistance, -1.5))*(Yj-Yi)*(Yj-Yi);
        dfyidxj = -dfyidxi;
        dfyidyj = -dfyidyi;

        dfxjdxi = dfxidxj;
        dfxjdyi = dfyidxj;
        dfxjdxj = -dfxjdxi;
        dfxjdyj = -dfxjdyi;

        dfyjdxi = dfxidyj;
        dfyjdyi = dfyidyj;
        dfyjdxj = -dfyjdxi;
        dfyjdyj = -dfyjdyi;

        tripletList.push_back(Triplet<double>(2*Pi, 2*Pi, dfxidxi));
        tripletList.push_back(Triplet<double>(2*Pi, (2*Pi)+1, dfxidyi));
        tripletList.push_back(Triplet<double>(2*Pi+1, 2*Pi, dfyidxi));
        tripletList.push_back(Triplet<double>((2*Pi)+1, (2*Pi)+1, dfyidyi));

        tripletList.push_back(Triplet<double>(2*Pj, 2*Pj, dfxjdxj));
        tripletList.push_back(Triplet<double>(2*Pj, (2*Pj)+1, dfxjdyj));
        tripletList.push_back(Triplet<double>(2*Pj+1, 2*Pj, dfyjdxj));
        tripletList.push_back(Triplet<double>((2*Pj)+1, (2*Pj)+1, dfyjdyj));

        tripletList.push_back(Triplet<double>(2*Pi, 2*Pj, dfxidxj));
        tripletList.push_back(Triplet<double>(2*Pi, (2*Pj)+1, dfxidyj));
        tripletList.push_back(Triplet<double>((2*Pi)+1, 2*Pj, dfyidxj));
        tripletList.push_back(Triplet<double>((2*Pi)+1, (2*Pj)+1, dfyidyj));

        tripletList.push_back(Triplet<double>(2*Pj, 2*Pi, dfxjdxi));
        tripletList.push_back(Triplet<double>(2*Pj, (2*Pi)+1, dfxjdyi));
        tripletList.push_back(Triplet<double>((2*Pj)+1, 2*Pi, dfyjdxi));
        tripletList.push_back(Triplet<double>((2*Pj)+1, (2*Pi)+1, dfyjdyi));
    }
    springForceGradient.setFromTriplets(tripletList.begin(), tripletList.end());
    return springForceGradient;
}

Eigen::VectorXd Simulation::generateViscousDampingForce(Eigen::VectorXd qConfig, Eigen::VectorXd qPrevConfig)
{
    double p1Xiold;
    double p1Xi;
    double p2Xiold;
    double p2Xi;
    double p1Yiold;
    double p1Yi;
    double p2Yiold;
    double p2Yi;
    int pi;
    int pj;
    Eigen::VectorXd visDampingForceVector(qConfig.rows());
    visDampingForceVector.setZero();
    if ((params_.activeForces & 2)==0)
    {
        return visDampingForceVector;
    }
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    {
        pi = it->p1Id;
        pj = it->p2Id;
        p1Xi = qConfig[pi*2];
        p1Yi = qConfig[(pi*2)+1];
        p2Xi = qConfig[pj*2];
        p2Yi = qConfig[(pj*2)+1];

        p1Xiold = qPrevConfig[pi*2];
        p1Yiold = qPrevConfig[(pi*2)+1];
        p2Xiold = qPrevConfig[pj*2];
        p2Yiold = qPrevConfig[(pj*2)+1];

        visDampingForceVector[pi*2] += (((p2Xi - p2Xiold) - (p1Xi - p1Xiold))*params_.dampingStiffness)/params_.timeStep;
        visDampingForceVector[(pi*2)+1] += (((p2Yi - p2Yiold) - (p1Yi - p1Yiold))*params_.dampingStiffness)/params_.timeStep;
        visDampingForceVector[pj*2] += -visDampingForceVector[pi*2];
        visDampingForceVector[(pj*2)+1] += -visDampingForceVector[(pi*2)+1];
    }
}

Eigen::SparseMatrix<double> Simulation::generateViscousDampingGradient(VectorXd qConfig, Eigen::VectorXd qPrevConfig)
{
    Eigen::SparseMatrix<double> visDampingForceGradient(particles_.size()*2, particles_.size()*2);
    visDampingForceGradient.setZero();
    vector< Triplet<double> > tripletList;
    double KbyH = -params_.dampingStiffness/params_.timeStep;
    int Pi;
    int Pj;
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    {
        Pi = it->p1Id;
        Pj = it->p2Id;

        tripletList.push_back(Triplet<double>(2*Pi, 2*Pi, -KbyH));
        tripletList.push_back(Triplet<double>(2*Pi, (2*Pi)+1, 0));
        tripletList.push_back(Triplet<double>(2*Pi+1, 2*Pi, 0));
        tripletList.push_back(Triplet<double>((2*Pi)+1, (2*Pi)+1, -KbyH));

        tripletList.push_back(Triplet<double>(2*Pj, 2*Pj, -KbyH));
        tripletList.push_back(Triplet<double>(2*Pj, (2*Pj)+1, 0));
        tripletList.push_back(Triplet<double>(2*Pj+1, 2*Pj, 0));
        tripletList.push_back(Triplet<double>((2*Pj)+1, (2*Pj)+1, -KbyH));

        tripletList.push_back(Triplet<double>(2*Pi, 2*Pj, KbyH));
        tripletList.push_back(Triplet<double>(2*Pi, (2*Pj)+1, 0));
        tripletList.push_back(Triplet<double>((2*Pi)+1, 2*Pj, 0));
        tripletList.push_back(Triplet<double>((2*Pi)+1, (2*Pj)+1, KbyH));

        tripletList.push_back(Triplet<double>(2*Pj, 2*Pi, KbyH));
        tripletList.push_back(Triplet<double>(2*Pj, (2*Pi)+1, 0));
        tripletList.push_back(Triplet<double>((2*Pj)+1, 2*Pi, 0));
        tripletList.push_back(Triplet<double>((2*Pj)+1, (2*Pi)+1, KbyH));
    }
    visDampingForceGradient.setFromTriplets(tripletList.begin(), tripletList.end());
    return visDampingForceGradient;

}

void Simulation::snapSprings()
{
    int p1Id;
    int p2Id;
    double distance;
    double epsilon;
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end();)
    {
        p1Id = it->p1Id;
        p2Id = it->p2Id;
        distance = sqrt((particles_[p1Id].pos[0]-particles_[p2Id].pos[0])*(particles_[p1Id].pos[0]-particles_[p2Id].pos[0])
                        + (particles_[p1Id].pos[1]-particles_[p2Id].pos[1])*(particles_[p1Id].pos[1]-particles_[p2Id].pos[1]));
        epsilon = (distance - it->restLength)/it->restLength;
        if (epsilon > params_.maxSpringStrain)
        {
            it = springs_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void Simulation::checkSawCollisions()
{
    // TODO : Implement check saw collision
//    for ()
}

double Simulation::euclideanDistanceFormula(double x1, double y1, double x2, double y2)
{
    return (x1-x2)*(x1-x2)+(y1-y2)*(y1-y2);
}

double Simulation::distanceFromInfiniteLine(Vector2d q1, Vector2d q2, Vector2d p)
{
    return abs((q2[0]-q1[0])*(q1[1]-p[1]) - (q1[0]-p[0])*(q2[1]-q1[1]))/sqrt(euclideanDistanceFormula(q1[0], q1[1], q2[0], q2[1]));
}

double Simulation::distanceFromFiniteLine(Vector2d q1, Vector2d q2, Vector2d p)
{
    double distance;
    double distSquared = euclideanDistanceFormula(q1[0], q1[1], q2[0], q2[1]);
    if (distSquared == 0)
    {
        distance = sqrt(euclideanDistanceFormula(q1[0], q1[1], q2[0], q2[1]));
        return distance;
    }
    double projectionT = ((p[0] - q1[0]) * (q2[0] - q1[0]) + (p[1] - q1[1]) * (q2[1] - q1[1])) / distSquared;
    if (projectionT < 0)
    {
        distance = sqrt(euclideanDistanceFormula(p[0], p[1], q1[0], q1[1]));
    }
    else if (projectionT > 1)
    {
        distance = sqrt(euclideanDistanceFormula(p[0], p[1], q2[0], q2[1]));
    }
    else
    {
        double x = q1[0] + projectionT * (q2[0] - q1[0]);
        double y = q1[1] + projectionT * (q2[1] - q1[1]);
        distance = sqrt(euclideanDistanceFormula(p[0], p[1], x, y));
    }
    return distance;
}

void Simulation::addSaw(double x, double y)
{
    renderLock_.lock();
    {
        Vector2d newpos(x,y);
        double rad = params_.sawRadius;
        saws_.push_back(Saw(newpos, rad));
    }
    renderLock_.unlock();
}

// TODO : Remove particles outside the world.
void Simulation::removeOutsideParticles()
{
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end();)
    {
        if (abs(it->pos[0])>1 || abs(it->pos[1])>1)
        {
            it = particles_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void Simulation::clearScene()
{
    renderLock_.lock();
    {
        particles_.clear();
        saws_.clear();
        springs_.clear();
    }
    renderLock_.unlock();
}
