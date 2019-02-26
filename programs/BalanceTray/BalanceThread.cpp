// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceThread.hpp"
#include <yarp/os/Time.h>
#include <KdlVectorConverter.hpp>
#include "KinematicRepresentation.hpp"
#include <ColorDebug.h>

using namespace roboticslab;

bool BalanceThread::threadInit()
{
    return iEncoders->getAxes(&axes);
}


void BalanceThread::run()
{
    std::vector<double> position, positionInAA;

    getCartesianPosition(&position);
    KinRepresentation::decodePose(position, positionInAA, KinRepresentation::CARTESIAN, KinRepresentation::AXIS_ANGLE, KinRepresentation::DEGREES );

    CD_DEBUG_NO_HEADER("POSS: [");
    for(int i=0; i<positionInAA.size(); i++){
        CD_DEBUG_NO_HEADER("%f ",positionInAA[i]);
    }
    CD_DEBUG_NO_HEADER("]\n ");

    if(positionInAA[6]>5){
        CD_WARNING("Turnning STOP (> 5º)!!\n");
        return;
    }

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
