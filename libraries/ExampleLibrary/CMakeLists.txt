# Copyright: Universidad Carlos III de Madrid (C) 2018
# Authors: Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1

if(ENABLE_ExampleLibrary)

# Configure library build target.
add_library(ExampleLibrary STATIC ExampleLibrary.cpp
                                  ExampleLibrary.hpp)

target_include_directories(ExampleLibrary PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

endif()
