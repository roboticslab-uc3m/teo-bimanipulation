// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_THREAD_HPP__
#define __TRAJECTORY_THREAD_HPP__

#include <kdl/trajectory.hpp>

#include <yarp/conf/version.h>

#include <yarp/os/PeriodicThread.h>

#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionDirect.h>

#include <ICartesianSolver.h>

class TrajectoryThread : public yarp::os::PeriodicThread
{
public:
    TrajectoryThread(yarp::dev::IEncoders * iEncoders,
                     roboticslab::ICartesianSolver * iCartesianSolver,
                     yarp::dev::IPositionDirect * iPositionDirect,
                     int period)
        : yarp::os::PeriodicThread(period * 0.001),
          iEncoders(iEncoders),
          iCartesianSolver(iCartesianSolver),
          iPositionDirect(iPositionDirect)
    {}

    void setICartesianTrajectory(KDL::Trajectory * trajectory)
    {
        this->trajectory = trajectory;
    }

    void resetTime();

protected:
    bool threadInit() override;
    void run() override;

private:
    yarp::dev::IEncoders * iEncoders {nullptr};
    roboticslab::ICartesianSolver * iCartesianSolver {nullptr};
    KDL::Trajectory * trajectory {nullptr};
    yarp::dev::IPositionDirect * iPositionDirect {nullptr};
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    std::size_t axes {0};
#else
    int axes {0};
#endif
    double startTime {0.0};
};

#endif  // __TRAJECTORY_THREAD_HPP__
