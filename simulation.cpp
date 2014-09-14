
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

        for(int i=0; i< ConnectedParticles_.size(); i++)
        {
            for(vector <Particle>::iterator it = ConnectedParticles_[i].begin(); it!=ConnectedParticles_[i].end(); ++it)
            {
                glColor3f(0,0,1);
                glBegin(GL_LINES);
                {
                    glVertex2f(particles_[i].pos[0], particles_[i].pos[1]);
                    glVertex2f(it->pos[0], it->pos[1]);
                }
            }
        }
    }
    renderLock_.unlock();
}

void Simulation::takeSimulationStep()
{
    time_ += params_.timeStep;
    cout <<"HERE";
}

void Simulation::addParticle(double x, double y)
{
    renderLock_.lock();
    {
        Vector2d newpos(x,y);
        double mass = params_.particleMass;
        particles_.push_back(Particle(newpos, mass, params_.particleFixed));

        vector<Particle> a;
        ConnectedParticles_.push_back(a);
        if(particles_.size() != ConnectedParticles_.size()){
            cout << "ERROR 1"<<"\n";
        }

        //if distance between new particle and an existing particle is within maxSpringDistance,
        //connect those particles with a spring
        double maxSpringDistSqd = params_.maxSpringDist*params_.maxSpringDist;
        for(int i = 0; i<particles_.size()-1; i++)
        {
            double x1x2 = (particles_[i].pos[0] - x);
            double y1y2 = (particles_[i].pos[1] - y);
            if(maxSpringDistSqd >= ((x1x2*x1x2)+(y1y2*y1y2)))
            {
                ConnectedParticles_[i].push_back(particles_[particles_.size()-1]);
                ConnectedParticles_[particles_.size()-1].push_back(particles_[i]);
            }

        }


    }
    renderLock_.unlock();
}

void Simulation::clearScene()
{
    renderLock_.lock();
    {
        particles_.clear();
        saws_.clear();
        ConnectedParticles_.clear();
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
