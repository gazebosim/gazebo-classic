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

/////////////////////////////////////////////////////////////////////////////
// Macros
/////////////////////////////////////////////////////////////////////////////

/// \file
/// \ingroup ignition_common
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
  }
}
/// \}

#endif
