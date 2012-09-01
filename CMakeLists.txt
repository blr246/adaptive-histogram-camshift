cmake_minimum_required(VERSION 2.8)
project(adaptive_histogram_camshift)

# Make sure that OpenCV_DIR is set in environment or find_package() will fail.
find_package(OpenCV REQUIRED)

file(GLOB HEADERS "*.h" "*.hpp")

set(SRCS
    adaptive_histogram_camshift.cpp
    ${HEADERS}
    )

add_executable(adaptive_histogram_camshift ${SRCS})
target_link_libraries(adaptive_histogram_camshift ${OpenCV_LIBS})

# Set max warning level and disable c99 initialization warning for MSVC.
set_target_properties(adaptive_histogram_camshift PROPERTIES COMPILE_FLAGS "/W4 /wd4351")

message("OpenCV_LIB_DIR: ${OpenCV_LIB_DIR}")
list(GET OpenCV_LIB_DIR 0 CV_LIB_DIR_0)
file(COPY "${CV_LIB_DIR_0}/../bin/opencv_core242.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_core242d.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_highgui242.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_highgui242d.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_imgproc242.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_imgproc242d.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_video242.dll"
          "${CV_LIB_DIR_0}/../bin/opencv_video242d.dll"
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

