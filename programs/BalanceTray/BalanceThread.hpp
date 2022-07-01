// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>

#define PI 3.141592654

class BalanceThread : public yarp::os::PeriodicThread
{
public:
    BalanceThread(yarp::dev::IEncoders *iEncoders,
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  int period)
        : yarp::os::PeriodicThread(period * 0.001),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),
          iPositionDirect(iPositionDirect),
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
    bool threadInit() override;
    void run() override;

private:
    yarp::os::Semaphore positionMutex;
    yarp::dev::IEncoders *iEncoders;
    roboticslab::ICartesianSolver *iCartesianSolver;
    yarp::dev::IPositionDirect *iPositionDirect;
    std::vector<double> position;
    int axes;
};
