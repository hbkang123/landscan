cmake_minimum_required(VERSION 3.16)

project(landscan LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PROJ REQUIRED)

add_library(sf45 STATIC
    third_party/sf45/lwNx.cpp
    third_party/sf45/linux/lwSerialPortLinux.cpp
    third_party/sf45/linux/platformLinux.cpp
)

target_include_directories(sf45 PUBLIC
    third_party/sf45
    third_party/sf45/linux
)

add_executable(landscan
    src/main.cpp
    src/landscan_application.cpp
    src/payload_manager.cpp
    src/positioning_manager.cpp
    src/pps_manager.cpp
    src/navigation_manager.cpp
    src/navigation_data_recorder.cpp
    src/timesync_manager.cpp
    src/lidar_manager.cpp
    src/lidar_time_estimator.cpp
    src/postprocess_data_loader.cpp
    src/postprocess_manager.cpp
    src/postprocess_interpolator.cpp
    src/postprocess_coordinate_transformer.cpp
    src/postprocess_crs_transformer.cpp
    src/postprocess_point_cloud_generator.cpp
    src/postprocess_point_cloud_projector.cpp
    src/postprocess_point_cloud_writer.cpp
    src/runtime_config.cpp

    third_party/psdk/platform/application/application.cpp

    third_party/psdk/platform/common/osal/osal.c
    third_party/psdk/platform/common/osal/osal_fs.c
    third_party/psdk/platform/common/osal/osal_socket.c

    third_party/psdk/platform/hal/hal_uart.c
    third_party/psdk/platform/hal/hal_i2c.c
    third_party/psdk/platform/hal/hal_network.c
    third_party/psdk/platform/hal/hal_usb_bulk.c
)

target_include_directories(landscan PRIVATE
    include
    third_party/psdk/include
    third_party/psdk/platform/application
    third_party/psdk/platform/hal
    third_party/psdk/platform/common
    third_party/psdk/platform/common/osal
)

target_link_libraries(landscan PRIVATE
    sf45
    ${CMAKE_SOURCE_DIR}/third_party/psdk/lib/libpayloadsdk.a
    pthread
    gpiod
    PROJ::proj
)