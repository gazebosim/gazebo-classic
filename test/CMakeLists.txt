include_directories (
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/test/gtest/include
  ${PROJECT_SOURCE_DIR}/test/gtest
  ${PROJECT_SOURCE_DIR}/test
)

link_directories(
  ${CCD_LIBRARY_DIRS}
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  ${ODE_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

configure_file (test_config.h.in ${PROJECT_BINARY_DIR}/test_config.h)

# Build gtest
if (WIN32)
  # Windows will fail to link the non-static libraries between gtest_main and
  # gtest unless .lib is created. It is easier to keep things static unless
  # other reasons appear.
  add_library(gtest STATIC gtest/src/gtest-all.cc)
  add_library(gtest_main STATIC gtest/src/gtest_main.cc)
else()
  # No static on UNIX to avoid problems on ARM platforms linking
  # with no fPIC
  add_library(gtest gtest/src/gtest-all.cc)
  add_library(gtest_main gtest/src/gtest_main.cc)
  target_link_libraries(gtest pthread)
endif()

target_link_libraries(gtest_main gtest)
execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(plugins)

set(TEST_TYPE "PERFORMANCE")
add_subdirectory(performance)
set(TEST_TYPE "INTEGRATION")
add_subdirectory(integration)
set(TEST_TYPE "EXAMPLE")
add_subdirectory(examples)
set(TEST_TYPE "REGRESSION")
add_subdirectory(regression)
add_subdirectory(cmake)
add_subdirectory(pkgconfig)
