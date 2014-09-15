
#include "simulation.h"
#include <QGLWidget>
#include "simparameters.h"
#include <iostream>

const double PI = 3.1415926535898;

using namespace Eigen;
using namespace std;

Simulation::Simulation(const SimParameters &params) : params_(params), time_(0)
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
        if(params_.F_FLOOR)
        {
            //draw floor
            glColor3f(0.5, 0.5, 0.5);
            glBegin(GL_QUADS);
            {
                glVertex2f(-1,0);
                glVertex2f(1,0);
                glVertex2f(1,-1);
                glVertex2f(-1,-1);
            }
            glEnd();
        }
        else
        {
            //draw floor
            glColor3f(0, 0, 0);
            glBegin(GL_QUADS);
            {
                glVertex2f(-1,0);
                glVertex2f(1,0);
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
                glVertex2f(it->p1.pos[0], it->p1.pos[1]);
                glVertex2f(it->p2.pos[0], it->p2.pos[1]);
            }
            glEnd();
        }
    }
    renderLock_.unlock();
}

void Simulation::takeSimulationStep()
{
    time_ += params_.timeStep;
    if(params_.F_GRAVITY){
        for(vector<Particle>::iterator it = particles_.begin(); it != particles_.end(); ++it)
        {
            if(!(it->fixed))
            {
                if(it->pos[1] > 0)
                {
                    double initVelY=  it->vel[1];
                    //df =  di+ Vi*t + 0.5*a*t^2
                    it->pos[1] = it->pos[1] + initVelY*params_.timeStep + 0.5*(params_.timeStep*params_.timeStep*params_.gravityG);
                    //Vf =Vi + a*t
                    it->vel[1] = initVelY + params_.timeStep*params_.gravityG;
                }
                else
                {
                    //change direction of momentum of ball
                    double initVelY= -1*(it->vel[1]);
                    //df =  di+ Vi*t + 0.5*a*t^2
                    it->pos[1] = it->pos[1] + initVelY*params_.timeStep + 0.5*(params_.timeStep*params_.timeStep*params_.gravityG);
                    //Vf =Vi + a*t
                    it->vel[1] = initVelY + params_.timeStep*params_.gravityG;
                }

            }
        }
    }
}

void Simulation::addParticle(double x, double y)
{
    renderLock_.lock();
    {
        Vector2d newpos(x,y);
        double mass = params_.particleMass;
        if(!params_.F_FLOOR || y>=0)
        {
            particles_.push_back(Particle(newpos, mass, params_.particleFixed));

            //if distance between new particle and an existing particle is within maxSpringDistance,
            //connect those particles with a spring
            double maxSpringDistSqd = params_.maxSpringDist*params_.maxSpringDist;
            for(int i = 0; i<particles_.size()-1; i++)
            {
                double x1x2 = (particles_[i].pos[0] - x);
                double y1y2 = (particles_[i].pos[1] - y);
                if(maxSpringDistSqd >= ((x1x2*x1x2)+(y1y2*y1y2)))
                {
                    springs_.push_back(SpringComponent(particles_[i], particles_[particles_.size()-1]));
                }
            }
        }

    }
    renderLock_.unlock();
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

