# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-FileCopyrightText: Copyright (c) 2026 Jeff Welder
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(smp_bt_client)

target_sources(app PRIVATE src/main.c)

# The image management client headers reach into <bootutil/image.h>, and the sample builds
# an MCUboot image header of its own, so the bootutil include path is needed here.
zephyr_library_link_libraries(MCUBOOT_BOOTUTIL)
