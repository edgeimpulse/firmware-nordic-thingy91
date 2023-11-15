target_include_directories(app PRIVATE .)
target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_at_handlers.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_device_nrf91.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_memory.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ei_sampler.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_sources_ifdef(CONFIG_REMOTE_INGESTION app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ei_ws_client.cpp)
