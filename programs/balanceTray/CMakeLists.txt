if(NOT DEFINED ENABLE_balanceTray OR ENABLE_balanceTray)
    if(NOT orocos_kdl_FOUND)
        message(WARNING "orocos_kdl package not found, disabling balanceTray")
    elseif(NOT TARGET ROBOTICSLAB::KdlVectorConverterLib)
        message(WARNING "KdlVectorConverterLib not found, disabling balanceTray")
    elseif(NOT TARGET ROBOTICSLAB::KinematicRepresentationLib)
        message(WARNING "KinematicRepresentationLib not found, disabling balanceTray")
    endif()
endif()

cmake_dependent_option(ENABLE_balanceTray "Enable/disable balanceTray program" ON
                       "orocos_kdl_FOUND;TARGET ROBOTICSLAB::KdlVectorConverterLib;TARGET ROBOTICSLAB::KinematicRepresentationLib" OFF)

if(ENABLE_balanceTray)

    add_executable(balanceTray main.cpp
                               BalanceThread.hpp
                               BalanceThread.cpp
                               BalanceTray.hpp
                               BalanceTray.cpp
                               DialogueManager.hpp
                               DialogueManager.cpp
                               TrajectoryThread.hpp
                               TrajectoryThread.cpp)

    target_link_libraries(balanceTray ${orocos_kdl_LIBRARIES}
                                      YARP::YARP_os
                                      YARP::YARP_init
                                      YARP::YARP_dev
                                      YARP::YARP_sig
                                      ROBOTICSLAB::KdlVectorConverterLib
                                      ROBOTICSLAB::KinematicRepresentationLib
                                      StaticLibrary)

    target_include_directories(balanceTray PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    install(TARGETS balanceTray)

else()

    set(ENABLE_balanceTray OFF CACHE BOOL "Enable/disable balanceTray program" FORCE)

endif()
