cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(robomaster_can_controller)

option(BUILD_PYTHON_BINDINGS "Build with pybind11 for python bindings" OFF)
option(BUILD_RUN_TESTS "Build with gtest for testing" OFF)

find_package(Threads REQUIRED)

set(SRC_LIST src/can_socket.cpp
             src/handler.cpp
             src/utils.cpp
             src/queue_msg.cpp
             src/robomaster.cpp
             src/data.cpp
             src/message.cpp)


add_library(${PROJECT_NAME} STATIC ${SRC_LIST})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

# Example 
add_executable(${PROJECT_NAME}_example examples/cpp_example.cpp)
target_link_libraries(${PROJECT_NAME}_example PRIVATE robomaster_can_controller ${CMAKE_THREAD_LIBS_INIT})

if(BUILD_PYTHON_BINDINGS)
    find_package(pybind11 REQUIRED)

    pybind11_add_module(robomaster_can_controller_py src/pybind_robomaster.cpp)
    target_link_libraries(robomaster_can_controller_py PUBLIC ${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
    set_property(TARGET robomaster_can_controller_py PROPERTY CXX_STANDARD 14)
endif()

if(BUILD_RUN_TESTS)
    find_package(GTest REQUIRED)

    enable_testing()

    add_executable(run_tests 
        tests/main_test.cpp 
        tests/data_test.cpp
        tests/message_test.cpp
        tests/utils_test.cpp
        tests/queue_test.cpp)

    target_link_libraries(run_tests PRIVATE GTest::GTest robomaster_can_controller)

    add_test(run_tests data_test)
    add_test(run_tests message_test)
    add_test(run_tests util_test)
    add_test(run_tests queue_test)

endif()
