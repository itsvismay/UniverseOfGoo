
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
            glColor3f(0.5, 0.5, 0.5);
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
            glColor3f(0, 1,0);
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
    generateGrForce();
    generateSpringForce();
    combineForces();
    if (params_.integrator == params_.TI_EXPLICIT_EULER)
    {
        explicitEuler();
        updateConfigVectorq();
        updateConfigVectorVel();
    }
    if((params_.activeForces & 4)==4)
    {
        int particleNo = 0;
        for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            if(it->pos[1] < floorHeight_)
            {
                it->vel[1] = -it->vel[1];
                velocityVector_[(particleNo*2)+1] = it->vel[1];
            }
            particleNo++;
        }
    }
}

void Simulation::explicitEuler()
{
    int particleNo = 0;
    for (vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        it->pos[0] = qVector_[particleNo*2] + params_.timeStep*velocityVector_[particleNo*2];
        it->pos[1] = qVector_[(particleNo*2)+1] + params_.timeStep*velocityVector_[(particleNo*2)+1];
        it->vel[0] = velocityVector_[particleNo*2] + params_.timeStep*forceVector_[particleNo*2]/it->mass;
        it->vel[1] = velocityVector_[(particleNo*2)+1] + params_.timeStep*forceVector_[(particleNo*2)+1]/it->mass;
        particleNo++;
    }
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
            //Build the configuration vectors
            updateConfigVectorq();
            updateConfigVectorVel();

            /*if distance between new particle and an existing particle is within maxSpringDistance,
            connect those particles with a spring*/
            double maxSpringDistSqd = params_.maxSpringDist;
            for(int i = 0; i<particles_.size()-1; i++)
            {
                double x1x2 = (particles_[i].pos[0] - x);
                double y1y2 = (particles_[i].pos[1] - y);
                double distance = sqrt(((x1x2*x1x2)+(y1y2*y1y2)));
                if(maxSpringDistSqd >= distance)
                {
                    springs_.push_back(SpringComponent(particles_[i].id, particles_[particles_.size()-1].id, distance));
                }
            }
        }

    }
    renderLock_.unlock();
}

void Simulation::updateConfigVectorq()
{
    int particleNo = 0;
    qVector_.resize(0);
    qVector_.resize(particles_.size()*2);
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        qVector_[particleNo*2] = it->pos[0];
        qVector_[(particleNo*2) + 1] = it->pos[1];
        particleNo++;
    }
}

void Simulation::updateConfigVectorVel()
{
    int particleNo = 0;
    velocityVector_.resize(0);
    velocityVector_.resize(particles_.size()*2);
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        velocityVector_[particleNo*2] = it->vel[0];
        velocityVector_[(particleNo*2) + 1] = it->vel[1];
        particleNo++;
    }
}

void Simulation::updateParticlePosFromQ()
{
    int particleNo = 0;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        it->pos[0] = qVector_[particleNo*2];
        it->pos[1] = qVector_[(particleNo*2) + 1];
        particleNo++;
    }
}

void Simulation::updateParticleVelFromV()
{
    int particleNo = 0;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        it->vel[0] = velocityVector_[particleNo*2];
        it->vel[1] = velocityVector_[(particleNo*2) + 1];
        particleNo++;
    }
}

void Simulation::generateGrForce()
{
    gravityForceVector_.resize(velocityVector_.rows());
    int particleNo = 0;
    for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
    {
        if ((params_.activeForces & 1)==0)
        {
            gravityForceVector_[particleNo*2] = 0.0;
            gravityForceVector_[(particleNo*2) + 1] = 0.0;
        }
        else
        {
            gravityForceVector_[particleNo*2] = 0.0;
            gravityForceVector_[(particleNo*2)+1] = it->mass*params_.gravityG;
        }
        particleNo++;
    }
}

void Simulation::generateSpringForce()
{
    double Kij;
    double L;
    double p1dotp2;
    double Xi;
    double Xj;
    double Yi;
    double Yj;
    double tempForce;
    int pi;
    int pj;
    springForceVector_.resize(0);
    springForceVector_.resize(velocityVector_.rows());
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
            p1dotp2 = sqrt((particles_[pi].pos[0]-particles_[pj].pos[0])*(particles_[pi].pos[0]-particles_[pj].pos[0])
                           + (particles_[pi].pos[1]-particles_[pj].pos[1])*(particles_[pi].pos[1]-particles_[pj].pos[1]));
//            p1dotp2 =  particles_[pi].pos.dot(particles_[pj].pos);
            Xi = particles_[pi].pos[0];
            Yi = particles_[pi].pos[1];
            Xj = particles_[pj].pos[0];
            Yj = particles_[pj].pos[1];
            tempForce = (Kij*(p1dotp2-L))/p1dotp2;
            if (particles_[pi].fixed)
            {
                springForceVector_[pi*2] = 0;
                springForceVector_[(pi*2)+1] = 0;
            }
            else
            {
                springForceVector_[pi*2] = tempForce/(Xj-Xi);
                springForceVector_[(pi*2)+1] = tempForce/(Yj-Yi);
            }
            if (particles_[pj].fixed)
            {
                springForceVector_[pj*2] = 0;
                springForceVector_[(pj*2)+1] = 0;
            }
            else
            {
                springForceVector_[pj*2] = tempForce/(Xi-Xj);
                springForceVector_[(pj*2)+1] = tempForce/(Yi-Yj);
            }
        }
    }
//    cout<<"\nGenerated Spring Force Vector: "<<springForceVector_;
}

void Simulation::combineForces()
{
    int i=0;
    forceVector_.resize(gravityForceVector_.rows());
    for (i=0; i<gravityForceVector_.rows(); i++)
    {
        forceVector_[i] = 0;
        forceVector_[i] = gravityForceVector_[i] + springForceVector_[i];
        springForceVector_[i]=0;
        gravityForceVector_[i]=0;
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

