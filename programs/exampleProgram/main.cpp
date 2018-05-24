// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 *
 * @ingroup teo-bimanipulation_programs
 * \defgroup exampleProgram
 *
 * @brief Creates an instance of roboticslab::ExampleProgram
 *
 * @section exampleProgram_legal Legal
 *
 * Copyright: 2018 (C) Universidad Carlos III de Madrid
 *
 * Author: Raul de Santos Rico
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the LGPLv2.1
 *
 * <hr>
 *
 * This file can be edited at exampleProgram
 *
 */

#include <yarp/os/all.h>

#include "ExampleProgram.hpp"

int main(int argc, char **argv) {

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("exampleProgram");
    rf.setDefaultConfigFile("exampleProgram.ini");
    rf.configure(argc, argv);

    roboticslab::ExampleProgram mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"%s --help\" for options.\n",argv[0]);
    printf("%s checking for yarp network... ",argv[0]);
    fflush(stdout);
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        fprintf(stderr,"[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n",argv[0]);
        return 1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

