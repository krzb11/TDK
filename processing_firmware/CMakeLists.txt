cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(processing_firmware)

target_sources(app PRIVATE 
    src/main.c
    src/system_tasks.c
    src/processing.c
    src/ADS1298/ADS1298.c
)

target_include_directories(app PRIVATE src/ADS1298)
target_include_directories(app PRIVATE src)