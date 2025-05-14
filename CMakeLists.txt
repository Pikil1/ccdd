cmake_minimum_required(VERSION 3.10)
project(self_collision_checker)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(PINOCCHIO_INCLUDE_DIR "/opt/openrobots/include")
set(PINOCCHIO_LIBRARY_DIR "/opt/openrobots/lib")


include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/third_party           
    ${PINOCCHIO_INCLUDE_DIR}
)


link_directories(${PINOCCHIO_LIBRARY_DIR})


find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


set(SOURCES
    src/capsule.cpp
    src/detector.cpp
    src/joint_load.cpp
    src/collisionchecker.cpp
)


add_executable(collisionchecker ${SOURCES})


target_link_libraries(collisionchecker
    ${PINOCCHIO_LIBRARY_DIR}/libpinocchio_default.so
    ${PINOCCHIO_LIBRARY_DIR}/libpinocchio_parsers.so
    ${PINOCCHIO_LIBRARY_DIR}/libpinocchio_collision.so   
)