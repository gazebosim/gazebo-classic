include_directories (
  ${PROJECT_BINARY_DIR}/gazebo
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
)

link_directories(
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  ${ODE_LIBRARY_DIRS}
)

if(HAVE_URDFDOM_HEADERS)
  link_directories(
    ${urdfdom_LIBRARY_DIRS}
    ${urdfdom_headers_LIBRARY_DIRS})
endif(HAVE_URDFDOM_HEADERS)

if(HAVE_CONSOLE_BRIDGE)
  link_directories(
    ${console_bridge_LIBRARY_DIRS})
endif(HAVE_CONSOLE_BRIDGE)

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

set(TEST_TYPE "PERFORMANCE")
add_subdirectory(performance)
set(TEST_TYPE "INTEGRATION")
add_subdirectory(regression)
add_subdirectory(cmake)
add_subdirectory(pkgconfig)

set(headers ServerFixture.hh)
gz_install_includes("test" ${headers} ${PROJECT_BINARY_DIR}/test_config.h)