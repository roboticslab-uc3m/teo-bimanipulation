# Store the package in the user registry.
export(PACKAGE TEO_BIMANIPULATION)

# Retrieve global properties.
get_property(_common_includes GLOBAL PROPERTY TEO_BIMANIPULATION_INCLUDE_DIRS)
get_property(_exported_targets GLOBAL PROPERTY TEO_BIMANIPULATION_TARGETS)

# CMake installation path.
if(WIN32)
    set(_cmake_destination CMake)
else()
    set(_cmake_destination ${CMAKE_INSTALL_LIBDIR}/cmake/TEO_BIMANIPULATION)
endif()

# Create and install config files.
include(CMakePackageConfigHelpers)

# <name>ConfigVersion.cmake file (same for build and install trees).
write_basic_package_version_file(${CMAKE_BINARY_DIR}/TEO_BIMANIPULATIONConfigVersion.cmake
                                 VERSION ${TEO_BIMANIPULATION_VERSION_SHORT}
                                 COMPATIBILITY AnyNewerVersion)

install(FILES ${CMAKE_BINARY_DIR}/TEO_BIMANIPULATIONConfigVersion.cmake
        DESTINATION ${_cmake_destination})

# Set exported variables (build tree).
set(TEO_BIMANIPULATION_INCLUDE_DIR "${_common_includes}")
set(TEO_BIMANIPULATION_MODULE_DIR ${CMAKE_SOURCE_DIR}/cmake)

# <pkg>Config.cmake (build tree).
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/templates/TEO_BIMANIPULATIONConfig.cmake.in
                              ${CMAKE_BINARY_DIR}/TEO_BIMANIPULATIONConfig.cmake
                              INSTALL_DESTINATION ${CMAKE_BINARY_DIR}
                              INSTALL_PREFIX ${CMAKE_BINARY_DIR}
                              PATH_VARS TEO_BIMANIPULATION_INCLUDE_DIR
                                        TEO_BIMANIPULATION_MODULE_DIR
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

# Set exported variables (install tree).
set(TEO_BIMANIPULATION_INCLUDE_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(TEO_BIMANIPULATION_MODULE_DIR ${CMAKE_INSTALL_DATADIR}/TEO_BIMANIPULATION/cmake)

# <pkg>Config.cmake (install tree).
configure_package_config_file(${CMAKE_SOURCE_DIR}/cmake/templates/TEO_BIMANIPULATIONConfig.cmake.in
                              ${CMAKE_BINARY_DIR}/TEO_BIMANIPULATIONConfig.cmake.install
                              INSTALL_DESTINATION ${_cmake_destination}
                              PATH_VARS TEO_BIMANIPULATION_INCLUDE_DIR
                                        TEO_BIMANIPULATION_MODULE_DIR
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO)

# Install <pkg>Config.cmake.
install(FILES ${CMAKE_BINARY_DIR}/TEO_BIMANIPULATIONConfig.cmake.install
        RENAME TEO_BIMANIPULATIONConfig.cmake
        DESTINATION ${_cmake_destination})

# Export library targets if enabled.
# https://github.com/roboticslab-uc3m/project-generator/issues/19
if(_exported_targets)
    # <pkg>Targets.cmake (build tree).
    # In CMake 3.0 or later: export(EXPORT TEO_BIMANIPULATION...)
    export(TARGETS ${_exported_targets}
           NAMESPACE TEO_BIMANIPULATION::
           FILE TEO_BIMANIPULATIONTargets.cmake)

    # <pkg>Targets.cmake (install tree).
    install(EXPORT TEO_BIMANIPULATION
            DESTINATION ${_cmake_destination}
            NAMESPACE TEO_BIMANIPULATION::
            FILE TEO_BIMANIPULATIONTargets.cmake)
endif()

# Configure and create uninstall target (YCM).
include(AddUninstallTarget)
