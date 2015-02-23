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
#ifndef _GZ_PLUGIN_HH_
#define _GZ_PLUGIN_HH_

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <gazebo/gazebo_config.h>
#ifdef HAVE_DL
#include <dlfcn.h>
#elif HAVE_LTDL
#include <ltdl.h>
#endif

#include <list>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  class Event;

  /// \addtogroup gazebo_common Common
  /// \{

  /// \enum PluginType
  /// \brief Used to specify the type of plugin.
  enum PluginType
  {
    /// \brief A World plugin
    WORLD_PLUGIN,
    /// \brief A Model plugin
    MODEL_PLUGIN,
    /// \brief A Sensor plugin
    SENSOR_PLUGIN,
    /// \brief A System plugin
    SYSTEM_PLUGIN,
    /// \brief A Visual plugin
    VISUAL_PLUGIN
  };


  /// \class PluginT Plugin.hh common/common.hh
  /// \brief A class which all plugins must inherit from
  template<class T>
  class PluginT
  {
    /// \brief plugin pointer type definition
    public: typedef boost::shared_ptr<T> TPtr;

    /// \brief Constructor
    public: PluginT()
            {
              this->dlHandle = NULL;
            }

    /// \brief Destructor
    public: virtual ~PluginT()
            {
            }

    /// \brief Get the name of the handler
    public: std::string GetFilename() const
            {
              return this->filename;
            }

    /// \brief Get the short name of the handler
    public: std::string GetHandle() const
            {
              return this->handle;
            }

    /// \brief a class method that creates a plugin from a file name.
    /// It locates the shared library and loads it dynamically.
    /// \param[in] _filename the path to the shared library.
    /// \param[in] _handle short name of the handler
    /// \return Shared Pointer to this class type
    public: static TPtr Create(const std::string &_filename,
                const std::string &_handle)
            {
              TPtr result;
              // PluginPtr result;
              struct stat st;
              bool found = false;
              std::string fullname, filename(_filename);
              std::list<std::string>::iterator iter;
              std::list<std::string> pluginPaths =
                common::SystemPaths::Instance()->GetPluginPaths();

#ifdef __APPLE__
              // This is a hack to work around issue #800,
              // error loading plugin libraries with different extensions
              {
                size_t soSuffix = filename.rfind(".so");
                const std::string macSuffix(".dylib");
                if (soSuffix != std::string::npos)
                  filename.replace(soSuffix, macSuffix.length(), macSuffix);
              }
#endif  // ifdef __APPLE__

              for (iter = pluginPaths.begin();
                   iter!= pluginPaths.end(); ++iter)
              {
                fullname = (*iter)+std::string("/")+filename;
                if (stat(fullname.c_str(), &st) == 0)
                {
                  found = true;
                  break;
                }
              }

              if (!found)
                fullname = filename;

#ifdef HAVE_DL
              fptr_union_t registerFunc;
              std::string registerName = "RegisterPlugin";

              void *dlHandle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
              if (!dlHandle)
              {
                gzerr << "Failed to load plugin " << fullname << ": "
                  << dlerror() << "\n";
                return result;
              }

              registerFunc.ptr = dlsym(dlHandle, registerName.c_str());

              if (!registerFunc.ptr)
              {
                gzerr << "Failed to resolve " << registerName
                      << ": " << dlerror();
                return result;
              }

              // Register the new controller.
              result.reset(registerFunc.func());
              result->dlHandle = dlHandle;

#elif HAVE_LTDL
              gzerr << "LTDL is deprecated as of Gazebo 2.0\n";
              fptr_union_t registerFunc;
              std::string registerName = "RegisterPlugin";

              static bool init_done = false;

              if (!init_done)
              {
                int errors = lt_dlinit();
                if (errors)
                {
                  gzerr << "Error(s) initializing dynamic loader ("
                    << errors << ", " << lt_dlerror() << ")";
                  return NULL;
                }
                else
                  init_done = true;
              }

              lt_dlhandle handle = lt_dlopenext(fullname.c_str());

              if (!handle)
              {
                gzerr << "Failed to load " << fullname
                      << ": " << lt_dlerror();
                return NULL;
              }

              T *(*registerFunc)() =
                (T *(*)())lt_dlsym(handle, registerName.c_str());
              resigsterFunc.ptr = lt_dlsym(handle, registerName.c_str());
              if (!registerFunc.ptr)
              {
                gzerr << "Failed to resolve " << registerName << ": "
                      << lt_dlerror();
                return NULL;
              }

              // Register the new controller.
              result.result(registerFunc.func());
              result->dlHandle = NULL;

#else  // HAVE_LTDL

              gzthrow("Cannot load plugins as libtool is not installed.");

#endif  // HAVE_LTDL

              result->handle = _handle;
              result->filename = filename;

              return result;
            }

    /// \brief Returns the type of the plugin
    /// \return type of the plugin
    public: PluginType GetType() const
            {
              return this->type;
            }

    /// \brief Type of plugin
    protected: PluginType type;

    /// \brief Path to the shared library file
    protected: std::string filename;

    /// \brief Short name
    protected: std::string handle;

    /// \brief Pointer to shared library registration function definition
    private: typedef union
             {
               T *(*func)();
               void *ptr;
             } fptr_union_t;

    /// \brief Handle used for closing the dynamic library.
    private: void *dlHandle;
  };

  /// \class WorldPlugin Plugin.hh common/common.hh
  /// \brief A plugin with access to physics::World.  See
  ///        <a href="http://gazebosim.org/wiki/tutorials/plugins">
  ///        reference</a>.
  class WorldPlugin : public PluginT<WorldPlugin>
  {
    /// \brief Constructor
    public: WorldPlugin()
             {this->type = WORLD_PLUGIN;}

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
  class ModelPlugin : public PluginT<ModelPlugin>
  {
    /// \brief Constructor
    public: ModelPlugin()
             {this->type = MODEL_PLUGIN;}

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
  class SensorPlugin : public PluginT<SensorPlugin>
  {
    /// \brief Constructor
    public: SensorPlugin()
             {this->type = SENSOR_PLUGIN;}

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
  class SystemPlugin : public PluginT<SystemPlugin>
  {
    /// \brief Constructor
    public: SystemPlugin()
             {this->type = SYSTEM_PLUGIN;}

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
  class VisualPlugin : public PluginT<VisualPlugin>
  {
    public: VisualPlugin()
             {this->type = VISUAL_PLUGIN;}

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
