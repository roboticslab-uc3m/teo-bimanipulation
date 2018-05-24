echo off

if [%1]==[] (
    echo Parameter missing: CMake generator, aborting...
    exit /B 1
) else (
    echo CMake generator supplied via CLI: %1
)

echo Detecting CMake...
where cmake || (
    echo 'cmake' command not found, exiting...
    exit /B 1
)

echo Detecting Doxygen...
where doxygen || (
    echo 'doxygen' command not found, exiting...
    exit /B 1
)

if exist build (
    echo Removing existing build directory...
    rd /s /q build
)

echo Creating and entering build directory...
md build || (
    echo Failed to create 'build' directory, exiting...
    exit /B 1
)

cd build

echo Configuring CMake...
cmake .. -G %1 ^
         -DENABLE_ExampleLibrary:BOOL=ON ^
         -DENABLE_exampleProgram:BOOL=ON ^
         -DTEO_BIMANIPULATION_DOXYGEN_HTML:BOOL=ON ^
         -DCPACK_BINARY_NSIS:BOOL=ON ^
 || (
    echo Errors during CMake configuration, exiting...
    exit /B 1
)

echo Building project...
cmake --build . --config Release || (
    echo Errors during build phase, exiting...
    exit /B 1
)

echo Generating documentation...
cmake --build . --config Release --target dox || (
    echo Documentation could not be generated, exiting...
    exit /B 1
)

echo Packaging files...
cmake --build . --config Release --target package || (
    echo Package step failed, exiting...
    exit /B 1
)

echo Project built and packaged.

pause >nul

