@REM SPDX-License-Identifier: GPL-3.0+
@REM SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

@echo off
echo Running lwIOLink unit tests...

REM Check if executable exists
if not exist "build\lwIOLink_tests.exe" (
    echo Error: lwIOLink_tests.exe not found. Please build first using build_and_test.bat
    exit /b 1
)

REM Run the tests from current directory
build\lwIOLink_tests.exe
if %errorlevel% neq 0 (
    echo Tests failed
    exit /b 1
)

echo All tests passed!
pause
