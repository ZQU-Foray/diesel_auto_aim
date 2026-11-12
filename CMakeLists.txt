cmake_minimum_required(VERSION 3.8)
project(diesel_auto_aim)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(${PROJECT_NAME} SHARED
    src/filter_friendly.cpp
    src/aimer.cpp
    src/target.cpp
)


target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_filter_friendly test/test_filter_friendly.cpp)
    target_link_libraries(test_filter_friendly ${PROJECT_NAME})

    ament_add_gtest(test_aimer test/test_aimer.cpp) 
    target_link_libraries(test_aimer ${PROJECT_NAME}) 

    ament_add_gtest(test_target test/test_target.cpp)
    target_link_libraries(test_target ${PROJECT_NAME})

endif()

ament_export_dependencies(Eigen3)

ament_package()
