# Build type link flags
set (CMAKE_LINK_FLAGS_RELEASE " " CACHE INTERNAL "Link flags for release" FORCE)
set (CMAKE_LINK_FLAGS_RELWITHDEBINFO " " CACHE INTERNAL "Link flags for release with debug support" FORCE)
set (CMAKE_LINK_FLAGS_DEBUG " " CACHE INTERNAL "Link flags for debug" FORCE)
set (CMAKE_LINK_FLAGS_PROFILE " -pg" CACHE INTERNAL "Link flags for profile" FORCE)
set (CMAKE_LINK_FLAGS_COVERAGE " --coverage" CACHE INTERNAL "Link flags for static code coverage" FORCE)

set (CMAKE_C_FLAGS_RELEASE "")
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # -s doesn't work with clang, see alternative in link below:
  # http://stackoverflow.com/questions/6085491/gcc-vs-clang-symbol-strippingu
  set (CMAKE_C_FLAGS_RELEASE "-s")
endif()
set (CMAKE_C_FLAGS_RELEASE " ${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release" FORCE)
set (CMAKE_CXX_FLAGS_RELEASE ${CMAKE_C_FLAGS_RELEASE})

set (CMAKE_C_FLAGS_RELWITHDEBINFO " -g -O2 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for release with debug support" FORCE)
set (CMAKE_CXX_FLAGS_RELWITHDEBINFO ${CMAKE_C_FLAGS_RELWITHDEBINFO})

set (CMAKE_C_FLAGS_DEBUG " -ggdb3 ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for debug" FORCE)
set (CMAKE_CXX_FLAGS_DEBUG ${CMAKE_C_FLAGS_DEBUG})

set (CMAKE_C_FLAGS_PROFILE " -fno-omit-frame-pointer -g -pg ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for profile" FORCE)
set (CMAKE_CXX_FLAGS_PROFILE ${CMAKE_C_FLAGS_PROFILE})

set (CMAKE_C_FLAGS_COVERAGE " -g -O0 -Wformat=2 --coverage -fno-inline ${CMAKE_C_FLAGS_ALL}" CACHE INTERNAL "C Flags for static code coverage" FORCE)
set (CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_C_FLAGS_COVERAGE}")
if (NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  # -fno-default-inline -fno-implicit-inline-templates are unimplemented, cause errors in clang
  # -fno-elide-constructors can cause seg-faults in clang 3.4 and earlier
  # http://llvm.org/bugs/show_bug.cgi?id=12208
  set (CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_COVERAGE} -fno-default-inline -fno-implicit-inline-templates -fno-elide-constructors")
endif()

#####################################
# Set all the global build flags
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
# Visual Studio enables c++11 support by default
if (NOT MSVC)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}} -std=c++11")
endif()
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE_UPPERCASE}}")

# Compiler-specific C++11 activation.
if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "GNU")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
  if (NOT (GCC_VERSION VERSION_GREATER 4.7))
    message(FATAL_ERROR "${PROJECT_NAME} requires g++ 4.8 or greater.")
  endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  execute_process(
    COMMAND ${CMAKE_CXX_COMPILER} -dumpversion OUTPUT_VARIABLE CLANG_VERSION)
  if (NOT (CLANG_VERSION VERSION_GREATER 3.2))
    message(FATAL_ERROR "${PROJECT_NAME} requires clang 3.3 or greater.")
  endif ()

  if ("${CMAKE_SYSTEM_NAME}" MATCHES "Darwin")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    if (NOT MSVC12)
      message(FATAL_ERROR "${PROJECT_NAME} requires VS 2013 os greater.")
    endif()
else ()
    message(FATAL_ERROR "Your C++ compiler does not support C++11.")
endif ()
