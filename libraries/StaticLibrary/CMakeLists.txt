add_library(StaticLibrary STATIC StaticLibrary.cpp
                                 StaticLibrary.hpp)

target_include_directories(StaticLibrary PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
