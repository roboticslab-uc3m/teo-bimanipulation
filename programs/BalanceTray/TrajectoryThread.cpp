// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryThread.hpp"
#include <yarp/os/Time.h>
#include <KdlVectorConverter.hpp>
#include "KinematicRepresentation.hpp"
#include <ColorDebug.h>

using namespace roboticslab::KinRepresentation;

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

    std::vector<double> position, positionInAA;
    iCartTrajectory->getPosition(movementTime, position);

    decodePose(position, positionInAA, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );

    CD_DEBUG_NO_HEADER("PTO: [");
    for(int i=0; i<positionInAA.size(); i++){
        CD_DEBUG_NO_HEADER("%f ",positionInAA[i]);
    }
    CD_DEBUG_NO_HEADER("] (%f)\n", movementTime);


    std::vector<double> currentQ(axes);
    if ( ! iEncoders->getEncoders( currentQ.data() ) ){
        CD_ERROR("Failed getEncoders of right-arm\n");
        return;
    }
    // inverse kinematic
    std::vector<double> desireQ(axes);
    if ( ! iCartesianSolver->invKin(position, currentQ, desireQ) )    {
        CD_ERROR("invKin failed.\n");
        return;
    }

    iPositionDirect->setPositions(desireQ.data());
}

