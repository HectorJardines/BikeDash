# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(csc_peripheral)

target_sources(app PRIVATE 
src/main.c
src/drivers/accel.c
src/drivers/csc_ble.c
src/drivers/csc_evt.c
src/app/csc_sensor.c
)
target_include_directories(app PRIVATE 
include
)
