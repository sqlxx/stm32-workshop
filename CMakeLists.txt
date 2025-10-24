cmake_minimum_required(VERSION 3.22)

# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32-workshop)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Building ${CMAKE_PROJECT_NAME}.bin and ${CMAKE_PROJECT_NAME}.hex"
)

find_program(OPENOCD openocd)
find_program(ST_FLASH st-flash)

if(OPENOCD)
  add_custom_target(flash
      COMMAND ${OPENOCD} -f interface/stlink.cfg -f target/stm32f1x.cfg -c "program ${CMAKE_PROJECT_NAME}.bin verify reset exit 0x8000000"
      COMMENT "Flashing ${CMAKE_PROJECT_NAME}.bin to target with OpenOCD"
  )
elseif(ST_FLASH)
  add_custom_target(flash
      COMMAND st-flash write ${CMAKE_PROJECT_NAME}.bin 0x8000000
      DEPENDS ${CMAKE_PROJECT_NAME}
      COMMENT "Flashing ${CMAKE_PROJECT_NAME}.bin to target with ST_LINK"
  )
else()
  message(WARNING "No flashing tool (OpenOCD or ST_FLASH) found in PATH")
endif()

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
set(LV_BUILD_CONF_PATH "LVGL/lv_conf.h" CACHE PATH "LVGL Config file path")
set(CONFIG_LV_BUILD_EXAMPLES off CACHE BOOL "Build LVGL examples")
set(CONFIG_LV_BUILD_DEMOS off CACHE BOOL "Build LVGL demos")
add_subdirectory(LVGL/lvgl)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
    lvgl
)

file(GLOB_RECURSE USER_SOURCES "User/Src/*.c")

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_SOURCES}
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
  "User/Inc"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
