// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceThread.hpp"
#include <yarp/os/Time.h>
#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>
#include <ColorDebug.h>

using namespace roboticslab;

bool BalanceThread::threadInit()
{
    return iEncoders->getAxes(&axes);
}


void BalanceThread::run()
{
    std::vector<double> position;
    getCartesianPosition(&position);

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
