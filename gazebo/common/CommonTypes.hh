/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/util/system.hh"

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
#define GAZEBO_DEPRECATED(version) __attribute__((deprecated))
#define GAZEBO_FORCEINLINE __attribute__((always_inline))
#elif defined(MSVC)
#define GAZEBO_DEPRECATED(version) ()
#define GAZEBO_FORCEINLINE __forceinline
#else
#define GAZEBO_DEPRECATED(version) ()
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
  class VisualPlugin;

  /// \def WorldPluginPtr
  /// \brief boost::shared_ptr to WorldPlugin
  typedef boost::shared_ptr<WorldPlugin> WorldPluginPtr;

  /// \def ModelPluginPtr
  /// \brief boost::shared_ptr to ModelPlugin
  typedef boost::shared_ptr<ModelPlugin> ModelPluginPtr;

  /// \def SensorPluginPtr
  /// \brief boost::shared_ptr to SensorPlugin
  typedef boost::shared_ptr<SensorPlugin> SensorPluginPtr;

  /// \def GUIPluginPtr
  /// \brief boost::shared_ptr to GUIPlugin
  typedef boost::shared_ptr<GUIPlugin> GUIPluginPtr;

  /// \def SystemPluginPtr
  /// \brief boost::shared_ptr to SystemPlugin
  typedef boost::shared_ptr<SystemPlugin> SystemPluginPtr;

  /// \def VisualPluginPtr
  /// \brief boost::shared_ptr to VisualPlugin
  typedef boost::shared_ptr<VisualPlugin> VisualPluginPtr;

  namespace common
  {
    class Animation;
    class Color;
    class DiagnosticTimer;
    class Image;
    class Mesh;
    class SubMesh;
    class MouseEvent;
    class NumericAnimation;
    class Param;
    class PoseAnimation;
    class SkeletonAnimation;
    class SphericalCoordinates;
    class Time;

    template <typename T>
    class ParamT;

    /// \brief Speed of light.
    static const double SpeedOfLight = 299792458;

    /// \def Param_V
    /// \brief std::vector of Param*
    typedef std::vector<common::Param*> Param_V;

    /// \def StrStr_M
    /// \brief std::map of a std::string to a std::string
    typedef std::map<std::string, std::string> StrStr_M;

    /// \def AnimationPtr
    /// \brief boost::shared_ptr to an Animation class
    typedef boost::shared_ptr<Animation> AnimationPtr;

    /// \def PoseAnimationPtr
    /// \brief boost::shared_ptr to a PoseAnimation class
    typedef boost::shared_ptr<PoseAnimation> PoseAnimationPtr;

    /// \def NumericAnimationPtr
    /// \brief boost::shared_ptr to a NumericAnimation class
    typedef boost::shared_ptr<NumericAnimation> NumericAnimationPtr;

    /// \def DiagnosticTimerPtr
    /// \brief boost::shared_ptr to a DiagnosticTimer class
    typedef boost::shared_ptr<DiagnosticTimer> DiagnosticTimerPtr;

    /// \def SphericalCoordinatesPtr
    /// \brief Boost shared pointer to a SphericalCoordinates object
    typedef boost::shared_ptr<SphericalCoordinates> SphericalCoordinatesPtr;
  }

  namespace event
  {
    class Connection;

    /// \def ConnectionPtr
    /// \brief boost::shared_ptr to a Connection class
    typedef boost::shared_ptr<Connection> ConnectionPtr;

    /// \def Connection_V
    /// \brief std::vector of ConnectionPtr
    typedef std::vector<ConnectionPtr> Connection_V;
  }
}
/// \}

#endif
