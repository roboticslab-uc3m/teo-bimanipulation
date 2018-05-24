# Copyright: Universidad Carlos III de Madrid (C) 2018
# Authors: Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(teo-bimanipulation)

### Go through single applications

#add_subdirectory(prog)
