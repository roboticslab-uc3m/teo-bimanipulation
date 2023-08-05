cmake_minimum_required(VERSION 3.16)

project(TEO_BIMANIPULATION LANGUAGES CXX)

# Let the user specify a configuration (only single-config generators).
if(NOT CMAKE_CONFIGURATION_TYPES)
    # Possible values.
    set(_configurations Debug Release MinSizeRel RelWithDebInfo)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${_configurations})

    foreach(_conf ${_configurations})
        set(_conf_string "${_conf_string} ${_conf}")
    endforeach()

    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY HELPSTRING
                 "Choose the type of build, options are:${_conf_string}")

    if(NOT CMAKE_BUILD_TYPE)
        # Encourage the user to specify build type.
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
    endif()
endif()

# Hard dependencies.
find_package(YCM 0.11 REQUIRED)
find_package(YARP 3.7 REQUIRED COMPONENTS os dev sig)

# Soft dependencies.
find_package(orocos_kdl 1.4 QUIET)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS QUIET)

# Configure installation paths for YARP resources.
yarp_configure_external_installation(teo-bimanipulation)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Create targets if specific requirements are satisfied.
include(CMakeDependentOption)

# Add main contents.
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(share)

# Configure and create uninstall target.
include(AddUninstallTarget)
