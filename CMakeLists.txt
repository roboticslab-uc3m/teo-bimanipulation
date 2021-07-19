# Copyright: Universidad Carlos III de Madrid (C) 2018
# Authors: Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1

cmake_minimum_required(VERSION 3.5)

# Start a project.
project(TEO_BIMANIPULATION)

# Set module path.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/modules
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules
                              ${CMAKE_SOURCE_DIR}/cmake/ycm-modules)

### options: cpp libraries
option(ENABLE_StaticLibrary "Choose if you want to compile StaticLibrary" TRUE)

### options: cpp programs
option(ENABLE_balanceTray "Choose if you want to compile balanceTray demostration" TRUE)

# Bootstrap YCM.
# https://github.com/robotology/ycm/issues/118
set(YCM_TAG v0.9.0)
include(YCMBootstrap)

# Retrieve current TEO_BIMANIPULATION version.
include(TEO_BIMANIPULATIONVersion)

# Define standard paths, check system configuration.
include(TEO_BIMANIPULATIONSystem)

# Setup build options.
include(TEO_BIMANIPULATIONOptions)

# Define global properties to store relevant configuration.
set_property(GLOBAL PROPERTY TEO_BIMANIPULATION_INCLUDE_DIRS)
set_property(GLOBAL PROPERTY TEO_BIMANIPULATION_TARGETS)

# Define and enter subdirectories.
add_subdirectory(cmake)
add_subdirectory(share)
add_subdirectory(libraries)
add_subdirectory(programs)

# Output configuration files.
include(TEO_BIMANIPULATIONDescribe)

# Create target for documentation generation.
include(TEO_BIMANIPULATIONDoc)

# Invoke CPack from here.
include(TEO_BIMANIPULATIONPackage)

# Generate a report of enabled/disabled features.
feature_summary(FATAL_ON_MISSING_REQUIRED_PACKAGES WHAT ALL)
