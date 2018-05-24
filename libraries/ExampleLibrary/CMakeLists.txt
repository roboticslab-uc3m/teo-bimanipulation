# Copyright: Universidad Carlos III de Madrid (C) 2018
# Authors: Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1

if(ENABLE_ExampleLibrary)

set(KEYWORD "ExampleLibrary")

# Find YARP.
find_package(YARP REQUIRED)

# Add current directory to global include paths, retrieve current values.
set_property(GLOBAL APPEND PROPERTY TEO_BIMANIPULATION_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}")
get_property(_common_dirs GLOBAL PROPERTY TEO_BIMANIPULATION_INCLUDE_DIRS)

# Define include paths.
include_directories(${YARP_INCLUDE_DIRS}
                    ${_common_dirs})

# Configure library build target.
add_library(${KEYWORD} ExampleLibrary.cpp
                       ExampleLibrary.hpp)

# Link external libraries.
target_link_libraries(${KEYWORD} LINK_PUBLIC ${YARP_LIBRARIES})

# Create installation rule.
install(TARGETS ${KEYWORD}
        EXPORT TEO_BIMANIPULATION
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT example_library
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT example_library
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT example_library)

# Export current target.
set_property(GLOBAL APPEND PROPERTY TEO_BIMANIPULATION_TARGETS ${KEYWORD})

endif()
