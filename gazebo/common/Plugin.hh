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
#ifndef _GAZEBO_PLUGIN_HH_
#define _GAZEBO_PLUGIN_HH_

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <dlfcn.h>

#include <list>
#include <string>

#include <sdf/sdf.hh>

#include <ignition/common.hh>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  /// \addtogroup gazebo_common Common
  /// \{

  /// \class WorldPlugin Plugin.hh common/common.hh
  /// \brief A plugin with access to physics::World.  See
  ///        <a href="http://gazebosim.org/wiki/tutorials/plugins">
  ///        reference</a>.
  class WorldPlugin : public ignition::common::PluginT<WorldPlugin>
  {
    /// \brief Constructor
    public: WorldPlugin()
             {}

    /// \brief Destructor
    public: virtual ~WorldPlugin() {}

    /// \brief Load function
    ///
    /// Called when a Plugin is first created, and after the World has been
    /// loaded. This function should not be blocking.
    /// \param[in] _world Pointer the World
    /// \param[in] _sdf Pointer the the SDF element of the plugin.
    public: virtual void Load(physics::WorldPtr _world,
                              sdf::ElementPtr _sdf) = 0;

    public: virtual void Init() {}
    public: virtual void Reset() {}
  };

  /// \brief A plugin with access to physics::Model.  See
  ///        <a href="http://gazebosim.org/wiki/tutorials/plugins">
  ///        reference</a>.
  class ModelPlugin : public ignition::common::PluginT<ModelPlugin>
  {
    /// \brief Constructor
    public: ModelPlugin()
             {}

    /// \brief Destructor
    public: virtual ~ModelPlugin() {}

    /// \brief Load function
    ///
    /// Called when a Plugin is first created, and after the World has been
    /// loaded. This function should not be blocking.
    /// \param[in] _model Pointer to the Model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: virtual void Load(physics::ModelPtr _model,
                              sdf::ElementPtr _sdf) = 0;

    /// \brief Override this method for custom plugin initialization behavior.
    public: virtual void Init() {}

    /// \brief Override this method for custom plugin reset behavior.
    public: virtual void Reset() {}
  };

  /// \class SensorPlugin Plugin.hh common/common.hh
  /// \brief A plugin with access to physics::Sensor.  See
  ///        <a href="http://gazebosim.org/wiki/tutorials/plugins">
  ///        reference</a>.
  class SensorPlugin : public ignition::common::PluginT<SensorPlugin>
  {
    /// \brief Constructor
    public: SensorPlugin()
             {}

    /// \brief Destructor
    public: virtual ~SensorPlugin() {}

    /// \brief Load function
    ///
    /// Called when a Plugin is first created, and after the World has been
    /// loaded. This function should not be blocking.
    /// \param[in] _sensor Pointer the Sensor.
    /// \param[in] _sdf Pointer the the SDF element of the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr _sdf) = 0;

    /// \brief Override this method for custom plugin initialization behavior.
    public: virtual void Init() {}

    /// \brief Override this method for custom plugin reset behavior.
    public: virtual void Reset() {}
  };

  /// \brief A plugin loaded within the gzserver on startup.  See
  ///        <a href="http://gazebosim.org/wiki/tutorials/plugins">
  ///        reference</a>.
  /// @todo how to make doxygen reference to the file gazebo.cc#g_plugins?
  class SystemPlugin : public ignition::common::PluginT<SystemPlugin>
  {
    /// \brief Constructor
    public: SystemPlugin()
             {}

    /// \brief Destructor
    public: virtual ~SystemPlugin() {}

    /// \brief Load function
    ///
    /// Called before Gazebo is loaded. Must not block.
    /// \param _argc Number of command line arguments.
    /// \param _argv Array of command line arguments.
    public: virtual void Load(int _argc = 0, char **_argv = NULL) = 0;

    /// \brief Initialize the plugin
    ///
    /// Called after Gazebo has been loaded. Must not block.
    public: virtual void Init() {}

    /// \brief Override this method for custom plugin reset behavior.
    public: virtual void Reset() {}
  };

  /// \brief A plugin loaded within the gzserver on startup.  See
  ///        <a href="http://gazebosim.org/wiki/tutorials/plugins">
  ///        reference</a>.
  class VisualPlugin : public ignition::common::PluginT<VisualPlugin>
  {
    public: VisualPlugin()
             {}

    /// \brief Load function
    ///
    /// Called when a Plugin is first created, and after the World has been
    /// loaded. This function should not be blocking.
    /// \param[in] _visual Pointer the Visual Object.
    /// \param[in] _sdf Pointer the the SDF element of the plugin.
    public: virtual void Load(rendering::VisualPtr _visual,
                              sdf::ElementPtr _sdf) = 0;

    /// \brief Initialize the plugin
    ///
    /// Called after Gazebo has been loaded. Must not block.
    public: virtual void Init() {}

    /// \brief Override this method for custom plugin reset behavior.
    public: virtual void Reset() {}
  };

  /// \}

/// \brief Plugin registration function for model plugin. Part of the shared
/// object interface. This function is called when loading the shared library
/// to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_MODEL_PLUGIN(classname) \
  extern "C" gazebo::ModelPlugin *RegisterPlugin(); \
  gazebo::ModelPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for world plugin. Part of the shared
/// object interface. This function is called when loading the shared library
/// to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_WORLD_PLUGIN(classname) \
  extern "C" gazebo::WorldPlugin *RegisterPlugin(); \
  gazebo::WorldPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for sensors. Part of the shared object
/// interface. This function is called when loading the shared library to add
/// the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_SENSOR_PLUGIN(classname) \
  extern "C" gazebo::SensorPlugin *RegisterPlugin(); \
  gazebo::SensorPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for system plugin. Part of the
/// shared object interface. This function is called when loading the shared
/// library to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_SYSTEM_PLUGIN(classname) \
  extern "C" gazebo::SystemPlugin *RegisterPlugin(); \
  gazebo::SystemPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for visual plugin. Part of the
/// shared object interface. This function is called when loading the shared
/// library to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_VISUAL_PLUGIN(classname) \
  extern "C" gazebo::VisualPlugin *RegisterPlugin(); \
  gazebo::VisualPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }
}

#endif
