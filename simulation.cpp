
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
        generateAllForces();
        updateMassInvForceVector();
        updateConfigVectorqPrev();
        for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            particleId = it->id;
            qVector_[particleId*2] = qVector_[particleId*2] + params_.timeStep*velocityVector_[particleId*2];
            qVector_[(particleId*2)+1] = qVector_[(particleId*2)+1] + params_.timeStep*velocityVector_[(particleId*2)+1];
            velocityVector_[particleId*2] = velocityVector_[particleId*2] + params_.timeStep*massInvForceVector_[particleId*2];
            velocityVector_[(particleId*2)+1] = velocityVector_[(particleId*2)+1] + params_.timeStep*massInvForceVector_[(particleId*2)+1];
        }
    }
    else if (params_.integrator == params_.TI_VELOCITY_VERLET)
    {
        // Update the q(i+1).
        int particleId = 0;
        updateConfigVectorqPrev();
        for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            particleId = it->id;
            qVector_[particleId*2] = qVector_[particleId*2] + params_.timeStep*velocityVector_[particleId*2];
            qVector_[(particleId*2)+1] = qVector_[(particleId*2)+1] + params_.timeStep*velocityVector_[(particleId*2)+1];
        }
        generateAllForces();
        updateMassInvForceVector();
        particleId = 0;
        // Use the q(i+1) to generate Force and update velocitites
        for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            particleId = it->id;
            velocityVector_[particleId*2] = velocityVector_[particleId*2] + params_.timeStep*massInvForceVector_[particleId*2];
            velocityVector_[(particleId*2)+1] = velocityVector_[(particleId*2)+1] + params_.timeStep*massInvForceVector_[(particleId*2)+1];
        }
        updateParticlePosFromQ();
        updateParticleVelFromV();
    }
    updateParticlePosFromQ();
    updateParticleVelFromV();
    snapSprings();
    // TODO : Floor Force needs to be re-done
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
            qVector_[qVector_.rows()-2] = newpos[0];
            qVector_[qVector_.rows()-1] = newpos[1];
            qPrevVector_[qPrevVector_.rows()-2] = newpos[0];
            qPrevVector_[qPrevVector_.rows()-1] = newpos[1];
//            updateConfigVectorq();
//            updateConfigVectorqPrev();
            updateConfigVectorVel();
            updateMassMatrix();
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

void Simulation::updateMassMatrix()
{
    massMatrix_.resize(particles_.size()*2, particles_.size()*2);
    massMatrix_.setZero();
    massMatrixInverse_.resize(particles_.size()*2, particles_.size()*2);
    massMatrixInverse_.setZero();
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
        massMatrix_.coeffRef(particleId*2,particleId*2) = mass;
        massMatrix_.coeffRef((particleId*2)+1,(particleId*2)+1) = mass;
        massMatrixInverse_.coeffRef(particleId*2,particleId*2) = 1/mass;
        massMatrixInverse_.coeffRef((particleId*2)+1,(particleId*2)+1) = 1/mass;
    }
//    cout<<"Mass Inverse Matrix: "<<massMatrixInverse_;
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

void Simulation::updateConfigVectorqPrev()
{
    int particleId;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        qPrevVector_[particleId*2] = qVector_[particleId*2];
        qPrevVector_[(particleId*2)+1] = qVector_[(particleId*2)+1];
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

void Simulation::updateMassInvForceVector()
{
    massInvForceVector_.resize(0);
    massInvForceVector_.resize(particles_.size()*2);
    massInvForceVector_ = massMatrixInverse_*forceVector_;
//    cout<<"\nMass Inv Force Vector: "<<massInvForceVector_;
}

void Simulation::updateParticlePosFromQ()
{
    int particleId;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        it->pos[0] = qVector_[particleId*2];
        it->pos[1] = qVector_[(particleId*2) + 1];
    }
}

void Simulation::updateParticleVelFromV()
{
    int particleId = 0;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        it->vel[0] = velocityVector_[particleId*2];
        it->vel[1] = velocityVector_[(particleId*2) + 1];
    }
}

void Simulation::generateAllForces()
{
    generateGrForce();
    generateSpringForce();
    generateViscousDampingForce();
    combineForces();
}

void Simulation::generateGrForce()
{
    gravityForceVector_.resize(qVector_.rows());
    int particleId = 0;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        particleId = it->id;
        if ((params_.activeForces & 1)==0)
        {
            gravityForceVector_[particleId*2] = 0.0;
            gravityForceVector_[(particleId*2) + 1] = 0.0;
        }
        else
        {
            if (it->fixed)
            {
                gravityForceVector_[particleId*2] = 0.0;
                gravityForceVector_[(particleId*2)+1] = 0.0;
            }
            else
            {
                gravityForceVector_[particleId*2] = 0.0;
                gravityForceVector_[(particleId*2)+1] = it->mass*params_.gravityG;
            }
        }
    }
//    cout<<"\nGravity Force Vector: "<<gravityForceVector_;
}

void Simulation::generateSpringForce()
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
    springForceVector_.resize(0);
    springForceVector_.resize(qVector_.rows());
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    {
        pi = it->p1Id;
        pj = it->p2Id;
//        cout<<"\nSpring P1 id X and Y : "<<it->p1.id<<","<<it->p1.pos[0]<<","<<it->p1.pos[1];
//        cout<<"\nParticle P1 id X and Y: "<<particles_[0].id<<","<<particles_[0].pos[0]<<","<<particles_[0].pos[1];
        if ((params_.activeForces & 2)==0)
        {
            springForceVector_[pi*2] = 0.0;
            springForceVector_[(pi*2)+1] = 0.0;
            springForceVector_[pj*2] = 0.0;
            springForceVector_[(pj*2)+1] = 0.0;
        }
        else
        {
            L =  it->restLength;
            Kij = params_.springStiffness/L;
            Xi = qVector_[pi*2];
            Yi = qVector_[(pi*2)+1];
            Xj = qVector_[pj*2];
            Yj = qVector_[(pj*2)+1];
            p1p2euclideanDistance = sqrt((Xi-Xj)*(Xi-Xj) + (Yi-Yj)*(Yi-Yj));
            tempForce = (Kij*(p1p2euclideanDistance-L))/p1p2euclideanDistance;
            springForceVector_[pi*2] = tempForce*(Xj-Xi);
            springForceVector_[(pi*2)+1] = tempForce*(Yj-Yi);
            springForceVector_[pj*2] = tempForce*(Xi-Xj);
            springForceVector_[(pj*2)+1] = tempForce*(Yi-Yj);
        }
    }
//    cout<<"\nGenerated Spring Force Vector: "<<springForceVector_;
}

void Simulation::generateViscousDampingForce()
{
    double p1p2euclideanDistance;
    double p1Xiold;
    double p1Xi;
    double p2Xiold;
    double p2Xi;
    double p1Yiold;
    double p1Yi;
    double p2Yiold;
    double p2Yi;
    double tempForce;
    int pi;
    int pj;
    visDampingForceVector_.resize(0);
    visDampingForceVector_.resize(qVector_.rows());
    for (vector<SpringComponent>::iterator it = springs_.begin(); it != springs_.end(); ++it)
    {
        pi = it->p1Id;
        pj = it->p2Id;
        if ((params_.activeForces & 2)==0)
        {
            visDampingForceVector_[pi*2] = 0.0;
            visDampingForceVector_[(pi*2)+1] = 0.0;
            visDampingForceVector_[pj*2] = 0.0;
            visDampingForceVector_[(pj*2)+1] = 0.0;
        }
        else
        {
            p1Xi = qVector_[pi*2];
            p1Yi = qVector_[(pi*2)+1];
            p2Xi = qVector_[pj*2];
            p2Yi = qVector_[(pj*2)+1];

            p1Xiold = qPrevVector_[pi*2];
            p1Yiold = qPrevVector_[(pi*2)+1];
            p2Xiold = qPrevVector_[pj*2];
            p2Yiold = qPrevVector_[(pj*2)+1];

            visDampingForceVector_[pi*2] = (((p2Xi - p2Xiold) - (p1Xi - p1Xiold))*params_.dampingStiffness)/params_.timeStep;
            visDampingForceVector_[(pi*2)+1] = (((p2Yi - p2Yiold) - (p1Yi - p1Yiold))*params_.dampingStiffness)/params_.timeStep;
            visDampingForceVector_[pj*2] = -visDampingForceVector_[pi*2];
            visDampingForceVector_[(pj*2)+1] = -visDampingForceVector_[(pi*2)+1];
        }
    }
}

void Simulation::combineForces()
{
    int i=0;
    forceVector_.resize(qVector_.rows());
    for (i=0; i<qVector_.rows(); i++)
    {
        forceVector_[i] = 0;
        forceVector_[i] = gravityForceVector_[i] + springForceVector_[i] + visDampingForceVector_[i];
        springForceVector_[i]=0;
        gravityForceVector_[i]=0;
    }
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

