/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef _WIN32
  #include <unistd.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>

#include <gazebo/gazebo_config.h>
#include <dlfcn.h>

#include <list>
#include <string>

#include <sdf/sdf.hh>
#include <boost/filesystem.hpp>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

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
    VISUAL_PLUGIN,
    /// \brief A GUI plugin
    GUI_PLUGIN
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
              this->dlHandle = nullptr;
            }

    /// \brief Destructor
    public: virtual ~PluginT()
            {
              // dlclose has been disabled due to segfaults in the test suite
              // This workaround is detailed in #1026 and #1066. After the test
              // or gazebo execution the plugin is not loaded in memory anymore
              // \todo Figure out the right thing to do.

              // dlclose(this->dlHandle);
            }

    /// \brief Get the name of the handler
    public: std::string GetFilename() const
            {
              return this->filename;
            }

    /// \brief Get the short name of the handler
    public: std::string GetHandle() const
            {
              return this->handleName;
            }

    /// \brief a class method that creates a plugin from a file name.
    /// It locates the shared library and loads it dynamically.
    /// \param[in] _filename the path to the shared library.
    /// \param[in] _name short name of the plugin
    /// \return Shared Pointer to this class type
    public: static TPtr Create(const std::string &_filename,
                const std::string &_name)
            {
              TPtr result;
              // PluginPtr result;
              std::string filename(_filename);
              std::list<std::string> pluginPaths =
                common::SystemPaths::Instance()->GetPluginPaths();
              std::ostringstream errorStream;

              // helper function to find and dlopen a plugin file
              // returns void * dlHandle
              auto findAndDlopenPluginFile = [](
                  const std::string &_pluginFilename,
                  const std::list<std::string> &_pluginPaths,
                  std::ostringstream &_errorStream) -> void *
              {
                struct stat st;
                bool found = false;
                std::string fullname;
                std::list<std::string>::const_iterator iter;

                for (iter = _pluginPaths.begin();
                     iter!= _pluginPaths.end(); ++iter)
                {
                  fullname = (*iter)+std::string("/")+_pluginFilename;
                  fullname = boost::filesystem::path(fullname)
                      .make_preferred().string();
                  if (stat(fullname.c_str(), &st) == 0)
                  {
                    found = true;
                    break;
                  }
                }

                if (!found)
                  fullname = _pluginFilename;

                void *dlHandle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
                if (!dlHandle)
                {
                  _errorStream << "Failed to load plugin " << fullname << ": "
                    << dlerror() << "\n";
                }
                return dlHandle;
              };

              // This logic is to support different extensions on each OS
              // see issue #800
              //
              // Linux: lib*.so
              // macOS: lib*.so, lib*.dylib
              // Windows: *.dll
              //
              // Assuming that most plugin names are specified as lib*.so,
              // replace prefix and suffix depending on the OS.
              // On macOS, first try the lib*.so name, then try lib*.dylib
#ifdef _WIN32
              {
                // replace .so with .dll
                size_t soSuffix = filename.rfind(".so");
                if (soSuffix != std::string::npos)
                {
                  const std::string winSuffix(".dll");
                  filename.replace(soSuffix, winSuffix.length(), winSuffix);
                }
                size_t libPrefix = filename.find("lib");
                if (libPrefix == 0)
                {
                  // remove the lib prefix
                  filename.erase(0, 3);
                }
              }
#endif  // ifdef _WIN32

              // Try to find and dlopen plugin with the following pattern:
              // Linux: lib*.so
              // macOS: lib*.so
              // Windows: *.dll
              void *dlHandle = findAndDlopenPluginFile(filename, pluginPaths,
                                                       errorStream);
#ifdef __APPLE__
              if (!dlHandle)
              {
                // lib*.so file could not be found or opened, try lib*.dylib
                size_t soSuffix = filename.rfind(".so");
                if (soSuffix != std::string::npos)
                {
                  const std::string macSuffix(".dylib");
                  filename.replace(soSuffix, macSuffix.length(), macSuffix);
                }
              }
              // macOS: lib*.dylib
              dlHandle = findAndDlopenPluginFile(filename, pluginPaths, errorStream);
#endif  // ifdef __APPLE__

              if (!dlHandle)
              {
                gzerr << errorStream.str();
                return result;
              }

              fptr_union_t registerFunc;
              const char *registerName = "RegisterPlugin";

              registerFunc.ptr = dlsym(dlHandle, registerName);

              if (!registerFunc.ptr)
              {
                gzerr << "Failed to resolve " << registerName
                      << ": " << dlerror();
                return result;
              }

              // Register the new controller.
              result.reset(registerFunc.func());
              result->dlHandle = dlHandle;

              result->handleName = _name;
              result->filename = filename;

              return result;
            }

    /// \brief Returns the type of the plugin
    /// \return type of the plugin
    public: PluginType GetType() const
            {
              return this->type;
            }

    /// \brief Load parameter value from _sdf and store it to the given
    ///        reference, using the supplied default value if the element in
    ///        _sdf is not found. A message is written using gzmsg reporting
    ///       whether the default value was used or not.
    /// \param[in] _sdf The SDF element of the plugin.
    /// \param[in] _name Name of a tag inside the SDF.
    /// \param[out] _target The reference to store the param value to.
    /// \param[in] _defaultValue The default value.
    protected: template <typename V> void LoadParam(const sdf::ElementPtr &_sdf,
                const std::string &_name, V &_target,
                V _defaultValue = V()) const
            {
              auto result = _sdf->Get<V>(_name, _defaultValue);

              if (!result.second)
              {
                gzmsg << this->handleName.c_str() << " Plugin missing <"
                      << _name.c_str() << ">, defaults to "
                      << result.first << std::endl;
              }
              else
              {
                gzmsg << this->handleName.c_str() << " Plugin <"
                      << _name.c_str() << "> set to "
                      << result.first << std::endl;
              }
              _target = result.first;
            }

    /// \brief Load parameter value from _sdf and store it to the given
    ///        reference, using the supplied default value if the element in
    ///        _sdf is not found. A message is written using gzmsg reporting
    ///        whether the default value was used or not. String specialization
    ///        to allow accepting const char* values for std::string parameters.
    /// \param[in] _sdf The SDF element of the plugin.
    /// \param[in] _name Name of a tag inside the SDF.
    /// \param[out] _target The reference to store the param value to.
    /// \param[in] _defaultValue The default value.
    protected: void LoadParam(sdf::ElementPtr &_sdf,
                const std::string &_name, std::string &_target,
                const char* _defaultValue) const
            {
              this->LoadParam<std::string>(_sdf, _name, _target, _defaultValue);
            }

    /// \brief Type of plugin
    protected: PluginType type;

    /// \brief Path to the shared library file
    protected: std::string filename;

    /// \brief Short name
    protected: std::string handleName;

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
  /// <a href="http://gazebosim.org/tutorials/?tut=plugins_world
  /// &cat=write_plugin">reference</a>.
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
  /// <a href="http://gazebosim.org/tutorials?tut=plugins_model
  /// &cat=write_plugin">reference</a>.
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
  /// <a href="http://gazebosim.org/tutorials?tut=plugins_hello_world
  /// &cat=write_plugin">reference</a>.
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
  /// <a href="http://gazebosim.org/tutorials?tut=system_plugin
  /// &cat=write_plugin">reference</a>
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
    public: virtual void Load(int _argc = 0, char **_argv = nullptr) = 0;

    /// \brief Initialize the plugin
    ///
    /// Called after Gazebo has been loaded. Must not block.
    public: virtual void Init() {}

    /// \brief Override this method for custom plugin reset behavior.
    public: virtual void Reset() {}
  };

  /// \brief A plugin with access to rendering::Visual.  See
  /// <a href="http://gazebosim.org/tutorials?tut=plugins_hello_world
  /// &cat=write_plugin">reference</a>.
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
  extern "C" GZ_PLUGIN_VISIBLE gazebo::ModelPlugin *RegisterPlugin(); \
  gazebo::ModelPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for world plugin. Part of the shared
/// object interface. This function is called when loading the shared library
/// to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_WORLD_PLUGIN(classname) \
  extern "C" GZ_PLUGIN_VISIBLE gazebo::WorldPlugin *RegisterPlugin(); \
  gazebo::WorldPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for sensors. Part of the shared object
/// interface. This function is called when loading the shared library to add
/// the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_SENSOR_PLUGIN(classname) \
  extern "C" GZ_PLUGIN_VISIBLE gazebo::SensorPlugin *RegisterPlugin(); \
  gazebo::SensorPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for system plugin. Part of the
/// shared object interface. This function is called when loading the shared
/// library to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_SYSTEM_PLUGIN(classname) \
  extern "C" GZ_PLUGIN_VISIBLE gazebo::SystemPlugin *RegisterPlugin(); \
  gazebo::SystemPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

/// \brief Plugin registration function for visual plugin. Part of the
/// shared object interface. This function is called when loading the shared
/// library to add the plugin to the registered list.
/// \return the name of the registered plugin
#define GZ_REGISTER_VISUAL_PLUGIN(classname) \
  extern "C" GZ_PLUGIN_VISIBLE gazebo::VisualPlugin *RegisterPlugin(); \
  gazebo::VisualPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }
}

#endif
