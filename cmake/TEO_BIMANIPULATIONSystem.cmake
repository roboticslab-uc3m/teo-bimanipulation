# Enable calls to set_package_properties() and add_feature_info().
include(FeatureSummary)

# Standard installation directories.
include(GNUInstallDirs)

# Control where libraries and executables are placed during the build.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})

# Add "d" postfix to debug libraries on Windows.
if(MSVC)
  message(STATUS "Running on windows")    
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Install system runtime libraries.
include(InstallRequiredSystemLibraries)
