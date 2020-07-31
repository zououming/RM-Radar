cmake_minimum_required(VERSION 3.7.1)
project(RM_radar)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_BUILD_TYPE "Release")
## 添加c++ 11标准支持
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

add_executable(RM_radar main.cpp
        CameraClass/DxImageProc.h
        CameraClass/GxIAPI.h
        CameraClass/CameraClass.cpp
        CameraClass/CameraClass.h

        ArmorDetector/ArmorDetector.h
        ArmorDetector/ArmorDetector.cpp
        ArmorDetector/opencv_extended.h
        ArmorDetector/General.h

        Settings/Settings.hpp

        Energy/Energy.hpp

        Pose/AngleSolver.hpp
        Pose/AngleSolver.cpp

        Serial/CRC_Check.h  
        Serial/CRC_Check.cpp
        Serial/serialport.cpp
        Serial/serialport.h
        ImageConsProd/ImageConsProd.h
        ImageConsProd/ImageConsProd.cpp

        YOLO/yoloApi.h
        YOLO/yoloApi.cpp
        )

target_link_libraries (RM_radar
        gxiapi
        pthread
        /home/zououming/darknet/libdarknet.so
        )

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)
set(CUDA_NVCC_FLAGS -G;-g;-std=c++11)
INCLUDE_DIRECTORIES(/home/zououming/darknet/include)
include_directories(${CUDA_INCLUDE_DIRS})

target_link_libraries(RM_radar
        ${OpenCV_LIBS}
        ${CUDA_LIBS}
        )

