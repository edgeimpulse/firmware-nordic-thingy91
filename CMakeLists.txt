cmake_minimum_required(VERSION 3.20.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(firmware-nordic-thingy91)

zephyr_compile_definitions(PROJECT_NAME=${PROJECT_NAME})

# Needed for colorful output
zephyr_compile_options(-fdiagnostics-color=always)

# Do not quantize filterbank
# Enable manual loop unrolling in DSP functions
add_definitions(-DEIDSP_QUANTIZE_FILTERBANK=0
                -DARM_MATH_LOOPUNROLL
                )

# Add all required source files
add_subdirectory(ei-model/edge-impulse-sdk/cmake/zephyr)
add_subdirectory(firmware-sdk)
add_subdirectory(src/inference)
add_subdirectory(src/sensors)
add_subdirectory(src)

# Include directories (everything in the SDK is already included here)
target_include_directories(app PRIVATE .)
target_include_directories(app PRIVATE ei-model)

# Use GLOB to include model files, because model file names differes whether model is EON compiled or not
RECURSIVE_FIND_FILE(MODEL_FILES ei-model/tflite-model "*.cpp")
target_sources(app PRIVATE ${MODEL_FILES})
