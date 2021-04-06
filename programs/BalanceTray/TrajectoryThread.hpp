// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/RateThread.h>

#include <kdl/trajectory.hpp>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>

class TrajectoryThread : public yarp::os::RateThread
{
public:
    TrajectoryThread(yarp::dev::IEncoders *iEncoders,
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  int period)
        : yarp::os::RateThread(period),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),
          trajectory(nullptr),
          iPositionDirect(iPositionDirect),
          axes(0),
          startTime(0)
    {}

    void setICartesianTrajectory(KDL::Trajectory *trajectory) {
        this->trajectory = trajectory;
    }

    void resetTime();

protected:
    virtual bool threadInit();
    virtual void run();

private:
    yarp::dev::IEncoders *iEncoders;
    roboticslab::ICartesianSolver *iCartesianSolver;
    KDL::Trajectory *trajectory;
    yarp::dev::IPositionDirect *iPositionDirect;
    int axes;
    double startTime;

};
