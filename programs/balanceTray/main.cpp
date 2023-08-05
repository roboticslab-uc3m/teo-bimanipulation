// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_bimanipulation_programs
 * @defgroup balanceTray balanceTray
 * @brief Creates an instance of teo::BalanceTray.
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include "BalanceTray.hpp"

int main(int argc, char * argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("kinematics");
    rf.configure(argc, argv);

    teo::BalanceTray mod;

    yInfo() << "balanceTray checking for YARP network...";

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "balanceTray found no YARP network (try running \"yarpserver &\")";
        return 1;
    }

    return mod.runModule(rf);
}
