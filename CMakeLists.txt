cmake_minimum_required(VERSION 3.16)
project(diesel_vision_backend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# 云台指令下发走平台链路层；本仓不引入 OpenCV / 推理运行时（硬实时后端）。
find_package(diesel_platform_core REQUIRED)

add_library(${PROJECT_NAME} SHARED
    src/filter_friendly.cpp
    src/aimer.cpp
    src/target.cpp
    src/tracker.cpp
    src/sp_association.cpp
    src/tracked_aim.cpp
    src/world_aim_solver.cpp
    src/aim_pipeline.cpp
    src/native_control_loop.cpp
    src/control_frame_adapter.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
    diesel_platform::link
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_freshness_policy test/test_freshness_policy.cpp)
    target_link_libraries(test_freshness_policy ${PROJECT_NAME})

    ament_add_gtest(test_aim_pipeline test/test_aim_pipeline.cpp)
    target_link_libraries(test_aim_pipeline ${PROJECT_NAME})
    ament_add_gtest(test_native_control_loop test/test_native_control_loop.cpp)
    target_link_libraries(test_native_control_loop ${PROJECT_NAME})
    ament_add_gtest(test_filter_friendly test/test_filter_friendly.cpp)
    target_link_libraries(test_filter_friendly ${PROJECT_NAME})

    ament_add_gtest(test_aimer test/test_aimer.cpp)
    target_link_libraries(test_aimer ${PROJECT_NAME})

    ament_add_gtest(test_target test/test_target.cpp)
    target_link_libraries(test_target ${PROJECT_NAME})

    ament_add_gtest(test_tracker test/test_tracker.cpp)
    target_link_libraries(test_tracker ${PROJECT_NAME})

    ament_add_gtest(test_sp_association test/test_sp_association.cpp)
    target_link_libraries(test_sp_association ${PROJECT_NAME})

    ament_add_gtest(test_tracked_aim test/test_tracked_aim.cpp)
    target_link_libraries(test_tracked_aim ${PROJECT_NAME})

    ament_add_gtest(test_world_aim_solver test/test_world_aim_solver.cpp)
    target_link_libraries(test_world_aim_solver ${PROJECT_NAME})

    ament_add_gtest(test_control_frame_adapter test/test_control_frame_adapter.cpp)
    target_link_libraries(test_control_frame_adapter ${PROJECT_NAME})
endif()

ament_export_dependencies(Eigen3 diesel_platform_core)

ament_package()
