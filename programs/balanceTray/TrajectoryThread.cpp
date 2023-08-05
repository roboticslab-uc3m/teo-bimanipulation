// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryThread.hpp"

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>

using namespace roboticslab::KinRepresentation;
using namespace roboticslab::KdlVectorConverter;

bool TrajectoryThread::threadInit()
{
    startTime = yarp::os::Time::now();
    return iEncoders->getAxes(&axes);
}

void TrajectoryThread::resetTime()
{
  startTime = yarp::os::Time::now();
}

void TrajectoryThread::run()
{
    double movementTime = yarp::os::Time::now() - startTime;

    std::vector<double> position = frameToVector(trajectory->Pos(movementTime));
    std::vector<double> positionInAA;

    decodePose(position, positionInAA, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );

    yDebug() << "PTO:" << positionInAA << "with time:" << movementTime;


    std::vector<double> currentQ(axes);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "Failed getEncoders() of right-arm";
        return;
    }

    // inverse kinematic
    std::vector<double> desireQ(axes);

    if (!iCartesianSolver->invKin(position, currentQ, desireQ))
    {
        yError() << "invKin() failed";
        return;
    }

    iPositionDirect->setPositions(desireQ.data());
}
