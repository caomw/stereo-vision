/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <cmath>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/math/Rand.h>

#include "eyesCalibration.h"

#define DEG2RAD     (M_PI/180.0)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/**************************************************************************/
struct Parameters
{
    int    numParticles;
    int    maxIter;
    double maxT;
    double omega;
    double phi_p;
    double phi_g;
    double cost;
    Matrix lim;

    /**************************************************************************/
    Parameters() : numParticles(40),
                   maxIter(std::numeric_limits<int>::max()),
                   maxT(std::numeric_limits<double>::infinity()),
                   omega(0.8),
                   phi_p(0.1),
                   phi_g(0.1),
                   cost(0.0)
    {
        lim.resize(6,2);
        lim(0,0)=-0.02;         lim(0,1)=0.02;
        lim(1,0)=-0.02;         lim(1,1)=0.02;
        lim(2,0)=-0.02;         lim(2,1)=0.02;
        lim(3,0)=-10.0*DEG2RAD; lim(3,1)=10.0*DEG2RAD;
        lim(4,0)=-10.0*DEG2RAD; lim(4,1)=10.0*DEG2RAD;
        lim(5,0)=-10.0*DEG2RAD; lim(5,1)=10.0*DEG2RAD;
    }
};


/**************************************************************************/
struct Particle
{
    Vector pos;
    Vector vel;
    double cost;    
    
    Particle() : pos(6,0.0), vel(6,0.0),
                 cost(std::numeric_limits<double>::infinity()) { }
};


/**************************************************************************/
class Optimizer
{
    const std::deque<CalibrationData> &data;
    Parameters parameters;

    std::deque<Particle> x,p;
    Particle g;

    Vector rand_min,rand_max;
    int iter;
    double t,t0;

    /**************************************************************************/
    void randomize()
    {
        for (size_t i=0; i<x.size(); i++)
        {
            Particle &particle=x[i];
            for (size_t i=0; i<particle.pos.length(); i++)
                particle.pos[i]=Rand::scalar(parameters.lim(i,0),parameters.lim(i,1));
            
            particle.vel[0]=Rand::scalar(-1e-5,1e-5);
            particle.vel[1]=Rand::scalar(-1e-5,1e-5);
            particle.vel[2]=Rand::scalar(-1e-5,1e-5);
            particle.vel[3]=Rand::scalar(-0.1,0.1)*DEG2RAD;
            particle.vel[4]=Rand::scalar(-0.1,0.1)*DEG2RAD;
            particle.vel[5]=Rand::scalar(-0.1,0.1)*DEG2RAD;
        }
    }

    /**************************************************************************/
    double evaluate(Particle &particle)
    {
        Matrix Hl,Hr;
        getExtrinsics(particle.pos,Hl,Hr);

        particle.cost=0.0;
        if (data.size()>0)
        {
            for (size_t i=0; i<data.size(); i++)
            {
                Matrix Hl_=data[i].eye_kin_left*Hl;
                Matrix Hr_=data[i].eye_kin_right*Hr;
                Matrix D=SE3inv(Hr_)*Hl_;

                particle.cost+=norm(data[i].fundamental.getCol(3).subVector(0,2)-D.getCol(3).subVector(0,2));
                particle.cost+=norm(dcm2rpy(data[i].fundamental)-dcm2rpy(D));
            }
            particle.cost/=data.size();
        }
        
        return particle.cost;
    }

    /**************************************************************************/
    void print(const bool randomize_print=false)
    {
        ostringstream str;
        str<<"iter #"<<iter<<" t="<<setprecision(3)<<fixed<<t<<" [s]: ";
        str.unsetf(ios::floatfield);
        str<<"cost="<<g.cost<<" ("<<parameters.cost<<"); ";
        if (randomize_print)
            str<<"particles scattered away";
        yInfo()<<str.str();
    }

public:
    /**************************************************************************/
    Optimizer(const std::deque<CalibrationData> &data_) : data(data_)
    {
        rand_min.resize(6,0.0);
        rand_max.resize(6,1.0);
    }

    /**************************************************************************/
    Parameters &getParameters()
    {
        return parameters;
    }

    /**************************************************************************/
    bool getExtrinsics(const Vector &x, Matrix &Hl, Matrix &Hr)
    {
        if (x.length()>=6)
        {
            Hr=rpy2dcm(x.subVector(3,5)); 
            Hr.setSubcol(x.subVector(0,2),0,3);

            Vector y=x;
            y[0]=-y[0];
            y[5]=-y[5];
            Hl=rpy2dcm(y.subVector(3,5));
            Hl.setSubcol(y.subVector(0,2),0,3);
            return true;
        }
        else
            return false;
    }

    /**************************************************************************/
    void init()
    {
        x.assign(parameters.numParticles,Particle());
        randomize();
        p=x;
        
        for (size_t i=0; i<x.size(); i++)
            if (evaluate(p[i])<g.cost)
                g=p[i];
        
        iter=0;
        t0=Time::now();
        t=0.0;
    }

    /**************************************************************************/
    bool step()
    {
        iter++;
        for (size_t i=0; i<x.size(); i++)
        {
            Vector r1=Rand::vector(rand_min,rand_max);
            Vector r2=Rand::vector(rand_min,rand_max);
            
            x[i].vel=parameters.omega*x[i].vel+
                     parameters.phi_p*r1*(p[i].pos-x[i].pos)+
                     parameters.phi_g*r2*(g.pos-x[i].pos);
            
            x[i].pos+=x[i].vel;
            for (size_t j=0; j<x[i].pos.length(); j++)
                x[i].pos[j]=std::min(std::max(x[i].pos[j],parameters.lim(j,0)),parameters.lim(j,1));
            
            double f=evaluate(x[i]);
            if (f<p[i].cost)
            {
                p[i]=x[i];
                p[i].cost=f;
                if (f<g.cost)
                    g=p[i];
            }
        }
        
        bool randomize_print=false;
        if ((iter%100)==0)
        {
            double mean=0.0;
            if (x.size()>0)
            {
                for (size_t i=0; i<x.size(); i++)
                    mean+=norm(g.pos-x[i].pos);            
                mean/=x.size();
            }

            if (mean<5e-9)
            {
                randomize();
                randomize_print=true;
            }
        }
        
        t=Time::now()-t0;
        bool term=(iter<parameters.maxIter) &&
                  (g.cost>parameters.cost) &&
                  (t<parameters.maxT);
        
        if ((iter%10)==0)
            print(randomize_print);
        
        return term;
    }

    /**************************************************************************/
    const Particle &finalize()
    {
        print();
        return g;
    }
};


/**************************************************************************/
CalibrationData &EyesCalibration::addData()
{
    CalibrationData d;
    data.push_back(d);
    return data.back();
}


/**************************************************************************/
double EyesCalibration::calibrate(Matrix &extrinsics_left,
                                  Matrix &extrinsics_right)
{
    Rand::init();

    Optimizer swarm(data);
    swarm.getParameters().maxT=10.0;    
    swarm.init();    

    int cnt=0;
    double t0=Time::now();
    while (swarm.step())
    {
        if (++cnt>=10)
        {
            Time::yield();
            cnt=0;
        }
    }
    double t=Time::now()-t0;

    const Particle &g=swarm.finalize();    
    yInfo()<<"solution: ("<<g.pos.toString(5,5)<<") "
           <<"found in "<<t<<" [s]";

    swarm.getExtrinsics(g.pos,extrinsics_left,extrinsics_right);    
    return g.cost;
}


