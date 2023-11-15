target_include_directories(app PRIVATE .)
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_accelerometer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_environment.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_light.cpp
)