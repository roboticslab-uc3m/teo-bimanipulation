// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>
#include <ICartesianTrajectory.hpp>

#define PI 3.141592654

class BalanceThread : public yarp::os::RateThread
{
public:
    BalanceThread(yarp::dev::IEncoders *iEncoders,                  
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  //yarp::dev::IPositionControl *iPosition,
                  int period)
        : yarp::os::RateThread(period),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),          
          iPositionDirect(iPositionDirect),
          //iPosition(iPosition),
          axes(0)
    {}

    void setCartesianPosition(std::vector<double> position) {
       positionMutex.wait();
       this->position = position;
       positionMutex.post();
    }


    void getCartesianPosition(std::vector<double> *position) {
        positionMutex.wait();
        *position = this->position;
        positionMutex.post();
    }



protected:    
    virtual bool threadInit();
    virtual void run();

private:
    yarp::os::Semaphore positionMutex;
    yarp::dev::IEncoders *iEncoders;    
    roboticslab::ICartesianSolver *iCartesianSolver;
    yarp::dev::IPositionDirect *iPositionDirect;
    //yarp::dev::IPositionControl *iPosition;
    std::vector<double> position;
    int axes;
};

