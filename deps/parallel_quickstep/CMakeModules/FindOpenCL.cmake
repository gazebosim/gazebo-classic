# - Try to find OpenCL
# This module tries to find an OpenCL implementation on your system. It supports
# AMD / ATI, Apple and NVIDIA implementations, but shoudl work, too.
#
# Once done this will define
#  OPENCL_FOUND        - system has OpenCL
#  OPENCL_INCLUDE_DIRS  - the OpenCL include directory
#  OPENCL_LIBRARIES    - link these to use OpenCL
#
# WIN32 should work, but is untested

FIND_PACKAGE( PackageHandleStandardArgs )

SET (OPENCL_VERSION_STRING "0.1.0")
SET (OPENCL_VERSION_MAJOR 0)
SET (OPENCL_VERSION_MINOR 1)
SET (OPENCL_VERSION_PATCH 0)

if(NOT CUDA_TOOLKIT_ROOT_DIR)
  # Search in the CUDA_BIN_PATH first.
  find_path(CUDA_TOOLKIT_ROOT_DIR
    NAMES nvcc nvcc.exe
    PATHS ENV CUDA_BIN_PATH
    DOC "Toolkit location."
    NO_DEFAULT_PATH
    )
  # Now search default paths
  find_path(CUDA_TOOLKIT_ROOT_DIR
    NAMES nvcc nvcc.exe
    PATHS /usr/local/bin
          /usr/local/cuda/bin
    DOC "Toolkit location."
    )
  if (CUDA_TOOLKIT_ROOT_DIR)
    string(REGEX REPLACE "[/\\\\]?bin[64]*[/\\\\]?$" "" CUDA_TOOLKIT_ROOT_DIR ${CUDA_TOOLKIT_ROOT_DIR})
    # We need to force this back into the cache.
    set(CUDA_TOOLKIT_ROOT_DIR ${CUDA_TOOLKIT_ROOT_DIR} CACHE PATH "Toolkit location." FORCE)
  endif(CUDA_TOOLKIT_ROOT_DIR)
  if (NOT EXISTS ${CUDA_TOOLKIT_ROOT_DIR})
    if(CUDA_FIND_REQUIRED)
      message(FATAL_ERROR "Specify CUDA_TOOLKIT_ROOT_DIR")
    elseif(NOT CUDA_FIND_QUIETLY)
      message("CUDA_TOOLKIT_ROOT_DIR not found or specified")
    endif()
  endif (NOT EXISTS ${CUDA_TOOLKIT_ROOT_DIR})
endif (NOT CUDA_TOOLKIT_ROOT_DIR)

IF (APPLE)

  FIND_LIBRARY(OPENCL_LIBRARIES OpenCL DOC "OpenCL lib for OSX")
  FIND_PATH(OPENCL_INCLUDE_DIRS OpenCL/cl.h DOC "Include for OpenCL on OSX")
  FIND_PATH(_OPENCL_CPP_INCLUDE_DIRS OpenCL/cl.hpp DOC "Include for OpenCL CPP bindings on OSX")

ELSE (APPLE)

  IF (WIN32)

    FIND_PATH(OPENCL_INCLUDE_DIRS CL/cl.h)
    FIND_PATH(_OPENCL_CPP_INCLUDE_DIRS CL/cl.hpp)

    # The AMD SDK currently installs both x86 and x86_64 libraries
    # This is only a hack to find out architecture
    IF( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64" )
      SET(OPENCL_LIB_DIR "$ENV{ATISTREAMSDKROOT}/lib/x86_64")
    ELSE (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64")
      SET(OPENCL_LIB_DIR "$ENV{ATISTREAMSDKROOT}/lib/x86")
    ENDIF( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "AMD64" )
    FIND_LIBRARY(OPENCL_LIBRARIES OpenCL.lib ${OPENCL_LIB_DIR})

    GET_FILENAME_COMPONENT(_OPENCL_INC_CAND ${OPENCL_LIB_DIR}/../../include ABSOLUTE)

    # On Win32 search relative to the library
    FIND_PATH(OPENCL_INCLUDE_DIRS CL/cl.h PATHS "${_OPENCL_INC_CAND}")
    FIND_PATH(_OPENCL_CPP_INCLUDE_DIRS CL/cl.hpp PATHS "${_OPENCL_INC_CAND}")

  ELSE (WIN32)

    # Unix style platforms
    FIND_LIBRARY(OPENCL_LIBRARIES OpenCL
      ENV LD_LIBRARY_PATH
      )

    GET_FILENAME_COMPONENT(OPENCL_LIB_DIR ${OPENCL_LIBRARIES} PATH)
    GET_FILENAME_COMPONENT(_OPENCL_INC_CAND ${OPENCL_LIB_DIR}/../include ABSOLUTE)

    # The AMD SDK currently does not place its headers
    # in /usr/include, therefore also search relative
    # to the library
    #FIND_PATH(OPENCL_INCLUDE_DIRS CL/cl.h PATHS ${_OPENCL_INC_CAND})
    #FIND_PATH(_OPENCL_CPP_INCLUDE_DIRS CL/cl.hpp PATHS ${_OPENCL_INC_CAND})
    FIND_PATH(OPENCL_INCLUDE_DIRS
      CL/cl.h
      PATHS "${CUDA_TOOLKIT_ROOT_DIR}/include"
      ENV CUDA_INC_PATH
      NO_DEFAULT_PATH
      )
    FIND_PATH(_OPENCL_CPP_INCLUDE_DIRS CL/cl.hpp PATHS ${_OPENCL_INC_CAND})

  ENDIF (WIN32)

ENDIF (APPLE)

FIND_PACKAGE_HANDLE_STANDARD_ARGS( OpenCL DEFAULT_MSG OPENCL_LIBRARIES OPENCL_INCLUDE_DIRS )

IF( _OPENCL_CPP_INCLUDE_DIRS )
  SET( OPENCL_HAS_CPP_BINDINGS TRUE )
  LIST( APPEND OPENCL_INCLUDE_DIRS ${_OPENCL_CPP_INCLUDE_DIRS} )
  # This is often the same, so clean up
  LIST( REMOVE_DUPLICATES OPENCL_INCLUDE_DIRS )
ENDIF( _OPENCL_CPP_INCLUDE_DIRS )

MARK_AS_ADVANCED(
  OPENCL_INCLUDE_DIRS
  )

