/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#ifndef GAZEBO_PLUGIN_REGISTERMACROS_HH_
#define GAZEBO_PLUGIN_REGISTERMACROS_HH_

#include <type_traits>
#include "gazebo/plugin/PluginInfo.hh"


/// TODO Delete this and import gazebo/util/system.hh
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define GZ_PLUGIN_VISIBLE __attribute__ ((dllexport))
  #else
    #define GZ_PLUGIN_VISIBLE __declspec(dllexport)
  #endif
  #define GZ_PLUGIN_HIDDEN
#else
  #if __GNUC__ >= 4
    #define GZ_PLUGIN_VISIBLE __attribute__ ((visibility ("default")))
    #define GZ_PLUGIN_HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define GZ_PLUGIN_VISIBLE
    #define GZ_PLUGIN_HIDDEN
  #endif
#endif


/// \brief Registers a shared library's one and only plugin
/// 
/// This has a limitation of one plugin per shared library
/// It adds a symbol "void* GZGetOnlyPluginV1()"
#define GZ_REGISTER_SINGLE_PLUGIN(class_name, base_class) \
  struct GZ_REGISTER_SINGLE_PLUGIN_TEST; \
  static_assert(std::is_same<GZ_REGISTER_SINGLE_PLUGIN_TEST, \
      ::GZ_REGISTER_SINGLE_PLUGIN_TEST>::value, \
      "GZ_REGISTER_SINGLE_PLUGIN must be in global namespace"); \
  static_assert(std::is_base_of<base_class, class_name>::value, \
      #class_name " must inherit from " #base_class); \
  static_assert(!std::is_same<class_name, base_class>::value, \
      "Class and Base class must be different"); \
  extern "C" GZ_PLUGIN_VISIBLE const \
  gazebo::plugin::PluginInfo GZSinglePluginInfoV1() \
  { \
    gazebo::plugin::PluginInfo plugin; \
    plugin.name = #class_name; \
    plugin.interface = #base_class; \
    plugin.factory = []() { return static_cast<void*>(new class_name()); }; \
    return plugin; \
  }; \
  extern "C" GZ_PLUGIN_VISIBLE const \
  std::size_t GZSinglePluginInfoSizeV1 = sizeof(gazebo::plugin::PluginInfo);

/// TODO macro supporting multiple plugins per shared library

#endif
