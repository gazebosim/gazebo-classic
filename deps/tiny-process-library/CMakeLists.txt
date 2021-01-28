include(FetchContent)
FetchContent_Declare(
  tinyprocesslibrary
  GIT_REPOSITORY https://gitlab.com/eidheim/tiny-process-library.git
  GIT_TAG v2.0.2)

FetchContent_GetProperties(tinyprocesslibrary)

if(NOT tinyprocesslibrary_POPULATED)
  FetchContent_Populate(tinyprocesslibrary)

  # We don't want to install this library in the system, we instead
  # compile it as an OBJECT library and embed in either the shared or
  # static libraries that need it.
  # From https://gitlab.kitware.com/cmake/cmake/-/issues/18935 it seems
  # that OBJECT libraries that are not installed become INTERFACE when
  # part of an EXPORT set.
  # This behaviour allows setting transitively tiny-process-library infos
  # to the consuming targets while not breaking the EXPORT process. In fact,
  # the conversion to INTERFACE allows to add tiny-process-library to the
  # targets of the EXPORT that contains targets linking against it.
  # See also https://cmake.org/pipermail/cmake/2018-September/068250.html.

  if(WIN32)
    add_library(tiny-process-library OBJECT
      ${tinyprocesslibrary_SOURCE_DIR}/process.cpp
      ${tinyprocesslibrary_SOURCE_DIR}/process_win.cpp)
    #If compiled using MSYS2, use sh to run commands
    if(MSYS)
      target_compile_definitions(tiny-process-library
        PUBLIC MSYS_PROCESS_USE_SH)
    endif()
  else()
    add_library(tiny-process-library OBJECT
      ${tinyprocesslibrary_SOURCE_DIR}/process.cpp
      ${tinyprocesslibrary_SOURCE_DIR}/process_unix.cpp)
  endif()
  add_library(tiny-process-library::tiny-process-library ALIAS tiny-process-library)

  if(MSVC)
    target_compile_definitions(tiny-process-library
      PRIVATE /D_CRT_SECURE_NO_WARNINGS)
  endif()

  find_package(Threads REQUIRED)

  target_link_libraries(tiny-process-library PRIVATE
    ${CMAKE_THREAD_LIBS_INIT})
  target_include_directories(tiny-process-library PUBLIC
    $<BUILD_INTERFACE:${tinyprocesslibrary_SOURCE_DIR}>)

endif()
