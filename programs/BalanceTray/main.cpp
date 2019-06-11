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

    CD_INFO_NO_HEADER("Run \"%s --help\" for options.\n",argv[0]);
    CD_INFO_NO_HEADER("%s checking for yarp network... ",argv[0]);
    fflush(stdout);

    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return 1;
    } else CD_SUCCESS_NO_HEADER("[ok]\n");


    return mod.runModule(rf);

}

