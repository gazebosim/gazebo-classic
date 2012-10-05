/*
 * Copyright 2011 Nate Koenig
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
#ifndef _GAZEBO_COMMON_TYPES_HH_
#define _GAZEBO_COMMON_TYPES_HH_

#include <vector>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

/////////////////////////////////////////////////////////////////////////////
// Defines
/////////////////////////////////////////////////////////////////////////////
#ifndef NULL
#define NULL 0
#endif

/////////////////////////////////////////////////////////////////////////////
// Macros
/////////////////////////////////////////////////////////////////////////////

#if defined(__GNUC__)
#define GAZEBO_DEPRECATED __attribute__((deprecated))
#define GAZEBO_FORCEINLINE __attribute__((always_inline))
#elif defined(MSVC)
#define GAZEBO_DEPRECATED
#define GAZEBO_FORCEINLINE __forceinline
#else
#define GAZEBO_DEPRECATED
#define GAZEBO_FORCEINLINE
#endif


/// \file
/// \ingroup gazebo_common
/// \{

/// \brief Forward declarations for the common classes
namespace gazebo
{
  class WorldPlugin;
  class ModelPlugin;
  class SensorPlugin;
  class GUIPlugin;
  class SystemPlugin;

  typedef boost::shared_ptr<WorldPlugin> WorldPluginPtr;
  typedef boost::shared_ptr<ModelPlugin> ModelPluginPtr;
  typedef boost::shared_ptr<SensorPlugin> SensorPluginPtr;
  typedef boost::shared_ptr<GUIPlugin> GUIPluginPtr;
  typedef boost::shared_ptr<SystemPlugin> SystemPluginPtr;

  namespace common
  {
    class Param;
    class Time;
    class Image;
    class Mesh;
    class MouseEvent;
    class PoseAnimation;
    class NumericAnimation;
    class Animation;
    class Color;

    template <typename T>
    class ParamT;

    typedef std::vector<common::Param*> Param_V;
    typedef std::map<std::string, std::string> StrStr_M;
    typedef boost::shared_ptr<Animation> AnimationPtr;
    typedef boost::shared_ptr<PoseAnimation> PoseAnimationPtr;
    typedef boost::shared_ptr<NumericAnimation> NumericAnimationPtr;
  }

  namespace event
  {
    class Connection;
    typedef boost::shared_ptr<Connection> ConnectionPtr;
    typedef std::vector<ConnectionPtr> Connection_V;
  }
}
/// \}

#endif
