cmake_minimum_required(VERSION 3.20)
project(vanttec_controllers)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


file(GLOB_RECURSE vanttec_controllers_SRC src/*.cpp)

add_library(vanttec_controllers "")

# Add header files that can be included in other projects.
target_include_directories(vanttec_controllers
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include/vanttec_controllers> # This will install headers prefixed with "vanttec_controllers"
)
# Add cpp sources here.
target_sources(vanttec_controllers
  PRIVATE
  src/controllers/control_laws/PID/first_order/pid.cpp
)

include(GoogleTest)
file(GLOB_RECURSE vanttec_controllers_TESTS tests/*.cpp)
enable_testing()
add_executable(vanttec_controllers_test ${vanttec_controllers_TESTS})
target_include_directories(vanttec_controllers_test PRIVATE src/)
target_link_libraries(vanttec_controllers_test gtest_main vanttec_controllers)

gtest_discover_tests(vanttec_controllers_test)

enable_testing()
