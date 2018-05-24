# Set CPack variables.

# Miscellanea.
set(CPACK_PACKAGE_VENDOR "Universidad Carlos III de Madrid")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Demostration of Teo manipulating objects with two arms Demostration of Teo manipulating objects with two arms")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_PACKAGE_CONTACT "Roboticslab team at UC3M")

# Package version.
set(CPACK_PACKAGE_VERSION_MAJOR "${TEO_BIMANIPULATION_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${TEO_BIMANIPULATION_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${TEO_BIMANIPULATION_VERSION_PATCH}")

set(CPACK_PACKAGE_VERSION
    "${TEO_BIMANIPULATION_VERSION_MAJOR}.${TEO_BIMANIPULATION_VERSION_MINOR}.${TEO_BIMANIPULATION_VERSION_PATCH}")

# Configure components.
include(CPackComponent)

cpack_add_component(example_library
                    DISPLAY_NAME "Example library"
                    DESCRIPTION "Binary files (.so/.dll/.lib) of example library."
                    REQUIRED
                    GROUP libraries
                    INSTALL_TYPES full)

cpack_add_component(example_program
                    DISPLAY_NAME "Example program"
                    DESCRIPTION "Executable of example program."
                    GROUP applications
                    DEPENDS example_library
                    INSTALL_TYPES full)

cpack_add_component_group(libraries
                          DISPLAY_NAME "Libraries"
                          DESCRIPTION "Libraries and other linkable stuff."
                          EXPANDED
                          BOLD_TITLE)

cpack_add_component_group(programs
                          DISPLAY_NAME "Programs"
                          DESCRIPTION "Collection of executables."
                          EXPANDED)

cpack_add_install_type(full
                       DISPLAY_NAME "Full install")

# Create shortcuts for executables.
if(ENABLE_exampleProgram)
    list(APPEND CPACK_PACKAGE_EXECUTABLES exampleProgram "Example program")
    list(APPEND CPACK_CREATE_DESKTOP_LINKS exampleProgram)
endif()

# NSIS-specific configuration variables.
if(WIN32 AND CMAKE_GENERATOR MATCHES "^Visual Studio")
    # Visual C++ toolchain.
    if(MSVC10)
        set(CPACK_SYSTEM_NAME v10)
    elseif(MSVC11)
        set(CPACK_SYSTEM_NAME v11)
    elseif(MSVC12)
        set(CPACK_SYSTEM_NAME v12)
    elseif(MSVC14)
        set(CPACK_SYSTEM_NAME v14)
    endif()

    # Architecture-dependent configuration.
    if(CMAKE_GENERATOR_PLATFORM MATCHES "x64" OR CMAKE_GENERATOR MATCHES "Win64")
        set(CPACK_SYSTEM_NAME "${CPACK_SYSTEM_NAME}_x86_amd64")
        set(CPACK_NSIS_INSTALL_ROOT "\\$PROGRAMFILES64\\\\ROBOTICSLAB")
    elseif(NOT CMAKE_GENERATOR MATCHES "ARM")
        set(CPACK_SYSTEM_NAME "${CPACK_SYSTEM_NAME}_x86")
        set(CPACK_NSIS_INSTALL_ROOT "\\$PROGRAMFILES32\\\\ROBOTICSLAB")
    endif()

    # Package name.
    set(CPACK_PACKAGE_FILE_NAME "teo-bimanipulation_${CPACK_PACKAGE_VERSION}_${CPACK_SYSTEM_NAME}")

    # Miscellanea.
    set(CPACK_NSIS_HELP_LINK "http://github.com/roboticslab-uc3m/teo-bimanipulation/")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    set(CPACK_NSIS_MODIFY_PATH ON)
    set(CPACK_NSIS_URL_INFO_ABOUT "http://roboticslab.uc3m.es/roboticslab/")

    # Additional Start Menu shortcuts.
    list(APPEND CPACK_NSIS_MENU_LINKS "http://roboticslab.uc3m.es/" "RoboticsLab UC3M Main Page")
endif()

if(UNIX)
    find_program(LSB_RELEASE_EXEC lsb_release)

    # Detect Linux platform.
    if(LSB_RELEASE_EXEC)
        execute_process(COMMAND ${LSB_RELEASE_EXEC} -cs
                        OUTPUT_VARIABLE LINUX_PLATFORM
                        OUTPUT_STRIP_TRAILING_WHITESPACE)

        set(CPACK_SYSTEM_NAME ${LINUX_PLATFORM})
    endif()

    # Detect architecture.
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(CPACK_SYSTEM_NAME "${CPACK_SYSTEM_NAME}_amd64")
    else()
        set(CPACK_SYSTEM_NAME "${CPACK_SYSTEM_NAME}_i386")
    endif()

    # Package name.
    set(CPACK_PACKAGE_FILE_NAME "teo-bimanipulation-${CPACK_PACKAGE_VERSION}.${CPACK_SYSTEM_NAME}")
endif()

# Launch CPack.
include(CPack)
