// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/PeriodicThread.h>

#include <kdl/trajectory.hpp>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>
#include "ICartesianSolver.h"
#include <ConfigurationSelector.hpp>

class TrajectoryThread : public yarp::os::PeriodicThread
{
public:
    TrajectoryThread(yarp::dev::IEncoders *iEncoders,
                  roboticslab::ICartesianSolver *iCartesianSolver,
                  yarp::dev::IPositionDirect *iPositionDirect,
                  int period)
        : yarp::os::PeriodicThread(period * 0.001),
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
    bool threadInit() override;
    void run() override;

private:
    yarp::dev::IEncoders *iEncoders;
    roboticslab::ICartesianSolver *iCartesianSolver;
    KDL::Trajectory *trajectory;
    yarp::dev::IPositionDirect *iPositionDirect;
    int axes;
    double startTime;
};
