// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceThread.hpp"
#include <yarp/os/Time.h>
#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>
#include <ColorDebug.h>

bool BalanceThread::threadInit()
{
    startTime = yarp::os::Time::now();
    return rightArmIEncoders->getAxes(&axes);
}

void BalanceThread::run()
{
    double movementTime = yarp::os::Time::now() - startTime;

    std::vector<double> position;
    iCartTrajectory->getPosition(movementTime, position);

    // -- Right-Arm
    std::vector<double> rightArmCurrentQ(axes);
    if ( ! rightArmIEncoders->getEncoders( rightArmCurrentQ.data() ) ){
        CD_ERROR("Failed getEncoders of right-arm\n");
        return;
    }
    // inverse kinematic
    std::vector<double> rightArmDesireQ(axes);
    if ( ! rightArmICartesianSolver->invKin(position, rightArmCurrentQ, rightArmDesireQ) )    {
        CD_ERROR("invKin failed.\n");
        return;
    }

    /*
    // -- Left-Arm
    std::vector<double> leftArmCurrentQ(6);
    if ( ! leftArmIEncoders->getEncoders( leftArmCurrentQ.data() ) ){
        CD_ERROR("Failed getEncoders of left-arm\n");
        return;
    }
    // inverse kinematic
    std::vector<double> leftArmDesireQ(6);
    if ( ! leftArmICartesianSolver->invKin(position, leftArmCurrentQ, leftArmDesireQ) )    {
        CD_ERROR("invKin failed.\n");
        return;
    }
    */

    rightArmIPositionDirect->setPositions(rightArmDesireQ.data());
}
