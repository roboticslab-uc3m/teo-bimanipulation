# Copyright: Universidad Carlos III de Madrid (C) 2018
# Authors: Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1

if(ENABLE_StaticLibrary)

# Configure library build target.
add_library(StaticLibrary STATIC StaticLibrary.cpp
                                  StaticLibrary.hpp)

target_include_directories(StaticLibrary PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

endif()
