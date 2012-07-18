/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef GZ_PLUGIN_HH
#define GZ_PLUGIN_HH

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <gazebo_config.h>
#ifdef HAVE_DL
#include <dlfcn.h>
#elif HAVE_LTDL
#include <ltdl.h>
#endif

#include <list>
#include <string>

#include "common/CommonTypes.hh"
#include "common/SystemPaths.hh"
#include "common/Console.hh"
#include "common/Exception.hh"

#include "physics/PhysicsTypes.hh"
#include "sensors/SensorTypes.hh"
#include "sdf/sdf.hh"

namespace gazebo
{
  class Event;

  /// \addtogroup gazebo_common Common
  /// \{
  /// \brief A class which all plugins must inherit from
  template<class T>
  class PluginT
  {
    public: typedef boost::shared_ptr<T> TPtr;

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

    public: static TPtr Create(const std::string &_filename,
                const std::string &_handle)
            {
              TPtr result;
              // PluginPtr result;
              struct stat st;
              bool found = false;
              std::string fullname;
              std::list<std::string>::iterator iter;
              std::list<std::string> pluginPaths =
                common::SystemPaths::Instance()->GetPluginPaths();

              for (iter = pluginPaths.begin();
                   iter!= pluginPaths.end(); ++iter)
              {
                fullname = (*iter)+std::string("/")+_filename;
                if (stat(fullname.c_str(), &st) == 0)
                {
                  found = true;
                  break;
                }
              }

              if (!found)
                fullname = _filename;

              fptr_union_t registerFunc;
              std::string registerName = "RegisterPlugin";

#ifdef HAVE_DL
              void* handle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
              if (!handle)
              {
                gzerr << "Failed to load plugin " << fullname << ": "
                  << dlerror() << "\n";
                return result;
              }

              registerFunc.ptr = dlsym(handle, registerName.c_str());

              if (!registerFunc.ptr)
              {
                gzerr << "Failed to resolve " << registerName
                      << ": " << dlerror();
                return result;
              }

              // Register the new controller.
              result.reset(registerFunc.func());

#elif HAVE_LTDL

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

#else  // HAVE_LTDL

              gzthrow("Cannot load plugins as libtool is not installed.");

#endif  // HAVE_LTDL

              result->handle = _handle;
              result->filename = _filename;

              return result;
            }

    protected: std::string filename;
    protected: std::string handle;

    private: typedef union
             {
               T *(*func)();
               void *ptr;
             } fptr_union_t;
  };

  class WorldPlugin : public PluginT<WorldPlugin>
  {
    /// \brief Load function
    public: virtual void Load(physics::WorldPtr _world,
                sdf::ElementPtr _sdf) = 0;
    public: virtual void Init() {}
    public: virtual void Reset() {}
  };

  class ModelPlugin : public PluginT<ModelPlugin>
  {
    /// \brief Load function
    public: virtual void Load(physics::ModelPtr _model,
                sdf::ElementPtr _sdf) = 0;
    public: virtual void Init() {}
    public: virtual void Reset() {}
  };

  class SensorPlugin : public PluginT<SensorPlugin>
  {
    /// \brief Load function
    public: virtual void Load(sensors::SensorPtr _sensor,
                sdf::ElementPtr _sdf) = 0;
    public: virtual void Init() {}
    public: virtual void Reset() {}
  };

  class SystemPlugin : public PluginT<SystemPlugin>
  {
    /// \brief Load function
    public: virtual void Load() = 0;
    public: virtual void Init() {}
    public: virtual void Reset() {}
  };

  /// \}

#define GZ_REGISTER_MODEL_PLUGIN(classname) \
  extern "C" gazebo::ModelPlugin *RegisterPlugin(); \
  gazebo::ModelPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

#define GZ_REGISTER_WORLD_PLUGIN(classname) \
  extern "C" gazebo::WorldPlugin *RegisterPlugin(); \
  gazebo::WorldPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

#define GZ_REGISTER_SENSOR_PLUGIN(classname) \
  extern "C" gazebo::SensorPlugin *RegisterPlugin(); \
  gazebo::SensorPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }

#define GZ_REGISTER_SYSTEM_PLUGIN(classname) \
  extern "C" gazebo::SystemPlugin *RegisterPlugin(); \
  gazebo::SystemPlugin *RegisterPlugin() \
  {\
    return new classname();\
  }
}
#endif
