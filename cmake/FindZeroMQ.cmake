##=============================================================================
#
# CMake - Cross Platform Makefile Generator
# Copyright 2000-2011 Kitware, Inc., Insight Software Consortium
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# * Neither the names of Kitware, Inc., the Insight Software Consortium,
# nor the names of their contributors may be used to endorse or promote
# products derived from this software without specific prior written
# permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#  Modified by Jose Luis Rivero <jrivero@osrfoundation.org>
#
##=============================================================================
# - Try to find ZeroMQ headers and libraries
#
# Usage of this module as follows:
#
#     find_package(ZeroMQ)
#
# Variables used by this module, they can change the default behaviour and need
# to be set before calling find_package:
#
#  ZeroMQ_ROOT_DIR  Set this variable to the root installation of
#                            ZeroMQ if the module has problems finding
#                            the proper installation path.
#
# Variables defined by this module:
#
#  ZEROMQ_FOUND              System has ZeroMQ libs/headers
#  ZeroMQ_LIBRARIES          The ZeroMQ libraries
#  ZeroMQ_INCLUDE_DIR        The location of ZeroMQ headers

if (UNIX)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(ZeroMQ libzmq>=3.2.0)
  if (NOT ZeroMQ_FOUND)
    message (STATUS "Looking for zmq pkgconfig file - not found")
  else()
    message (STATUS "Looking for zmq pkgconfig file - found")
  endif()
endif()

if(MSVC)
  #add in all the names it can have on windows
  if(CMAKE_GENERATOR_TOOLSET MATCHES "v140" OR MSVC14)
    set(_zmq_TOOLSET "-v140")
  elseif(CMAKE_GENERATOR_TOOLSET MATCHES "v120" OR MSVC12)
    set(_zmq_TOOLSET "-v120")
  elseif(CMAKE_GENERATOR_TOOLSET MATCHES "v110_xp")
    set(_zmq_TOOLSET "-v110_xp")
  elseif(CMAKE_GENERATOR_TOOLSET MATCHES "v110" OR MSVC11)
    set(_zmq_TOOLSET "-v110")
  elseif(CMAKE_GENERATOR_TOOLSET MATCHES "v100" OR MSVC10)
    set(_zmq_TOOLSET "-v100")
  elseif(CMAKE_GENERATOR_TOOLSET MATCHES "v90" OR MSVC90)
    set(_zmq_TOOLSET "-v90")
  endif()

  set(_zmq_versions "4_0_4" "4_0_3" "4_0_2" "4_0_1" "4_0_0"
                    "3_2_5" "3_2_4" "3_2_3" "3_2_2"  "3_2_1" "3_2_0" "3_1_0")
  set(_zmq_release_names)
  set(_zmq_debug_names)
  foreach( ver ${_zmq_versions})
    list(APPEND _zmq_release_names "libzmq${_zmq_TOOLSET}-mt-${ver}")
  endforeach()
  foreach( ver ${_zmq_versions})
    list(APPEND _zmq_debug_names "libzmq${_zmq_TOOLSET}-mt-gd-${ver}")
  endforeach()

  #now try to find the release and debug version
  find_library(ZeroMQ_LIBRARY_RELEASE
    NAMES ${_zmq_release_names} zmq libzmq
    HINTS ${ZeroMQ_ROOT_DIR}/bin
          ${ZeroMQ_ROOT_DIR}/lib
    )

  find_library(ZeroMQ_LIBRARY_DEBUG
    NAMES ${_zmq_debug_names} zmq libzmq
    HINTS ${ZeroMQ_ROOT_DIR}/bin
          ${ZeroMQ_ROOT_DIR}/lib
    )

  if(ZeroMQ_LIBRARY_RELEASE AND ZeroMQ_LIBRARY_DEBUG)
    set(ZeroMQ_LIBRARY
        debug ${ZeroMQ_LIBRARY_DEBUG}
        optimized ${ZeroMQ_LIBRARY_RELEASE}
        )
  elseif(ZeroMQ_LIBRARY_RELEASE)
    set(ZeroMQ_LIBRARY ${ZeroMQ_LIBRARY_RELEASE})
  elseif(ZeroMQ_LIBRARY_DEBUG)
    set(ZeroMQ_LIBRARY ${ZeroMQ_LIBRARY_DEBUG})
  endif()

  find_path(ZeroMQ_INCLUDE_DIR
    NAMES zmq.h
    HINTS ${ZeroMQ_ROOT_DIR}/include
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(ZeroMQ DEFAULT_MSG
    ZeroMQ_LIBRARY
    ZeroMQ_INCLUDE_DIR
  )

  set(ZeroMQ_INCLUDE_DIRS ${ZeroMQ_INCLUDE_DIR})
  set(ZeroMQ_LIBRARIES ${ZeroMQ_LIBRARY})

  mark_as_advanced(
   ZeroMQ_ROOT_DIR
   ZeroMQ_LIBRARY
   ZeroMQ_LIBRARY_DEBUG
   ZeroMQ_LIBRARY_RELEASE
   ZeroMQ_INCLUDE_DIR
  )

  # ZEROMQ_FOUND is uppercase in Windows, make it compatible with pkgconfig
  if (ZEROMQ_FOUND)
    set(ZeroMQ_FOUND TRUE)
  endif()
endif()