@REM SPDX-License-Identifier: GPL-3.0+
@REM SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

@echo off
echo Building lwIOLink tests...

if not exist "build" mkdir build

REM Run CMake from the tests directory (current directory)
cmake -S . -B build -G "Unix Makefiles" -DCMAKE_CXX_COMPILER="C:/msys64/mingw64/bin/g++.exe" -DCMAKE_C_COMPILER="C:/msys64/mingw64/bin/gcc.exe"
if %errorlevel% neq 0 (
    echo CMake configuration failed
    exit /b 1
)

REM Run make in the build directory from current directory
cmake --build build
if %errorlevel% neq 0 (
    echo Build failed
    exit /b 1
)

echo Build completed successfully!
echo Running tests...

REM Run tests directly from current directory
build\lwIOLink_tests.exe
if %errorlevel% neq 0 (
    echo Tests failed
    exit /b 1
)

echo All tests passed!

REM You can also run tests using CTest:
REM cd build && ctest --output-on-failure && cd ..
