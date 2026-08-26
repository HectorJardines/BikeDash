# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(csc_central)

target_sources(app PRIVATE 
src/main.c
src/drivers/csc_ble_cli.c
src/drivers/barometric_alt.c
src/drivers/display.c
src/app/csc_computer.c
src/app/statistics.c
src/drivers/display.c
src/ui/screens.c
src/ui/ui_font_5_5_fnt.c
src/ui/ui.c
src/app/stopwatch.c
)
target_include_directories(app PRIVATE 
include
)
