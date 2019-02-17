// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/RateThread.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>
#include <ICartesianTrajectory.hpp>

class BalanceThread : public yarp::os::RateThread
{
public:
    BalanceThread(yarp::dev::IEncoders *rightArmIEncoders,
                  yarp::dev::IEncoders *leftArmIEncoders,
                  roboticslab::ICartesianSolver *rightArmICartesianSolver,
                  roboticslab::ICartesianSolver *leftArmICartesianSolver,
                  roboticslab::ICartesianTrajectory *iCartTrajectory,
                  int period)
        : yarp::os::RateThread(period),
          axes(0),
          startTime(0)
    {}

protected:
    virtual bool threadInit();
    virtual void run();

private:
    yarp::dev::IEncoders *rightArmIEncoders, *leftArmIEncoders;
    roboticslab::ICartesianSolver *rightArmICartesianSolver, *leftArmICartesianSolver;
    roboticslab::ICartesianTrajectory * iCartTrajectory;
    int axes;
    double startTime;

};

