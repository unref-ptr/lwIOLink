// SPDX-License-Identifier: GPL-3.0+
// SPDX-FileCopyrightText: 2025 unref-ptr <unref-ptr@protonmail.com>

#include "CppUTest/CommandLineTestRunner.h"

int main(int ac, char** av)
{
    return CommandLineTestRunner::RunAllTests(ac, av);
}
