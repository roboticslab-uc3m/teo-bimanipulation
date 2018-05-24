#!/bin/sh

echo "Detecting CMake..."
which cmake || {
    echo "'cmake' command not found, exiting..."
    exit 1
}

echo "Detecting Doxygen..."
which doxygen || {
    echo "'doxygen' command not found, exiting..."
    exit 1
}

echo "Creating and entering build directory..."
mkdir -p build || {
    echo "Failed to create 'build' directory, exiting..."
    exit 1
}

cd build

echo "Configuring CMake..."
cmake .. -DENABLE_ExampleLibrary:BOOL=ON \
         -DENABLE_exampleProgram:BOOL=ON \
         -DTEO_BIMANIPULATION_DOXYGEN_HTML:BOOL=ON \
         -DCPACK_BINARY_DEB:BOOL=ON \
|| {
    echo "Errors during CMake configuration, exiting..."
    exit 1
}

echo "Building project..."
make -j$(nproc) || {
    echo "Errors during build phase, exiting..."
    exit 1
}

echo "Generating documentation..."
make dox || {
    echo "Documentation could not be generated, exiting..."
    exit 1
}

echo "Packaging files..."
make package || {
    echo "Package step failed, exiting..."
    exit 1
}

echo "Project built and packaged."

