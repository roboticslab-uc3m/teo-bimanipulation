// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "BalanceThread.hpp"

#include <yarp/os/LogStream.h>

using namespace roboticslab;

bool BalanceThread::threadInit()
{
#if YARP_VERSION_COMPARE(>=, 4, 0, 0)
    return iEncoders->getAxes(axes);
#else
    return iEncoders->getAxes(&axes);
#endif
}

void BalanceThread::run()
{
    std::vector<double> position;
    getCartesianPosition(position);

    std::vector<double> currentQ(axes);

    if (!iEncoders->getEncoders(currentQ.data()))
    {
        yError() << "Failed getEncoders() of right-arm";
        return;
    }

    // inverse kinematic
    std::vector<double> desireQ(axes);

    if (!iCartesianSolver->inverseKinematics(position, currentQ, desireQ))
    {
        yError() << "inverseKinematics() failed";
        return;
    }

    iPositionDirect->setPositions(desireQ.data());
}
