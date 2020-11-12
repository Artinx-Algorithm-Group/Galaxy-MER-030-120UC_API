project(uvc_driver)

#uvc_driver



include_directories(
    $ENV{GENICAM_ROOT_V2_3}/library/CPP/include
    $ENV{DAHENG_ROOT}/sdk/include
    ${OpenCV_INCLUDE_DIRS}
    ../include
    )

link_directories(
    $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64
    $ENV{GENICAM_ROOT_V2_3}/bin/Linux64_x64/GenApi/Generic
    $ENV{DAHENG_ROOT}/sdk/lib
)


add_library(uvc_driver
  SHARED
  uvc_driver.cpp
)


target_link_libraries(uvc_driver
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
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

target_include_directories(uvc_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)

add_library(driver::uvc_driver ALIAS uvc_driver)
