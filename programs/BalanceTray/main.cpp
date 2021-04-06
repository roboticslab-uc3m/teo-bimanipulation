// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo_bimanipulation_programs
 * \defgroup balanceTray balanceTray
 *
 * @brief Creates an instance of teo::BalanceTray.
 *
 * @section balanceTray Legal
 *
 * Copyright: 2017 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/people/r-de-santos">Raul de Santos Rico</a>
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 3.0 or later
 *
 * <hr>
 *
 * This file can be edited at balanceTray
 *
 */

#include <yarp/os/LogStream.h>

#include "BalanceTray.hpp"

int main(int argc, char **argv) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("kinematics"); // context to find kinematic config files
    rf.configure(argc, argv);

    teo::BalanceTray mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    yInfo("Run \"%s --help\" for options", argv[0]);
    yInfo("%s checking for yarp network...", argv[0]);
    fflush(stdout);

    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError("%s found no yarp network (try running \"yarpserver &\"), bye!", argv[0]);
        return 1;
    };

    return mod.runModule(rf);
}
