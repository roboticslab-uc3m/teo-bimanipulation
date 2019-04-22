// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/RateThread.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>
#include <ICartesianTrajectory.hpp>

class TrajectoryThread : public yarp::os::RateThread
{
public:
    TrajectoryThread(yarp::dev::IEncoders *iEncoders,
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  //roboticslab::ICartesianTrajectory *iCartTrajectory,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  int period)
        : yarp::os::RateThread(period),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),
          //iCartTrajectory(iCartTrajectory),
          iPositionDirect(iPositionDirect),
          axes(0),
          startTime(0)
    {}

    void setICartesianTrajectory(roboticslab::ICartesianTrajectory *iCartTrajectory) {
        this->iCartTrajectory = iCartTrajectory;
    }

    void resetTime();

protected:
    virtual bool threadInit();
    virtual void run();

private:
    yarp::dev::IEncoders *iEncoders;
    roboticslab::ICartesianSolver *iCartesianSolver;
    roboticslab::ICartesianTrajectory * iCartTrajectory;
    yarp::dev::IPositionDirect *iPositionDirect;
    int axes;
    double startTime;

};
