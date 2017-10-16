cmake_minimum_required(VERSION 3.8)
project(c__)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp Matrix2x2.h Matrix3x3.h Pose3D.h RotationMatrix.cpp RotationMatrix.h Vector2.h Vector3.h
        Range.h MVTools.cpp MVTools.h RobotDimensions.h)
add_executable(c__ ${SOURCE_FILES})