cmake_minimum_required(VERSION 2.8)
project(cam_test)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

message(GENICAM_ROOT_V2_3: $ENV{GENICAM_ROOT_V2_3})
message(DAHENG_ROOT: $ENV{DAHENG_ROOT})

find_package(OpenCV 3 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    $ENV{GENICAM_ROOT_V2_3}/library/CPP/include
    $ENV{DAHENG_ROOT}/sdk/include
    ${OpenCV_INCLUDE_DIRS}
    )

link_directories(
    $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64
    $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/GenApi/Generic
    $ENV{DAHENG_ROOT}/sdk/lib
)

add_executable(capture_test capture_test.cpp)

target_link_libraries(
    capture_test
        dhgentl
        gxiapi
        GCBase_gcc40_v2_3
        GenApi_gcc40_v2_3
        log4cpp_gcc40_v2_3
        Log_gcc40_v2_3
        MathParser_gcc40_v2_3
    )

add_executable(capture_as_opencv capture_as_opencv.cpp)

target_link_libraries(
    capture_as_opencv
        dhgentl
        gxiapi
        GCBase_gcc40_v2_3
        GenApi_gcc40_v2_3
        log4cpp_gcc40_v2_3
        Log_gcc40_v2_3
        MathParser_gcc40_v2_3
        dximageproc
        Threads::Threads
        ${OpenCV_LIBS}
    )