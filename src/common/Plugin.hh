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
#ifndef PLUGIN_HH
#define PLUGIN_HH

#include <string>
#include <boost/signals.hpp>
#include "common/CommonTypes.hh"
#include "sdf/sdf.h"

namespace gazebo
{
  class Event;

  /// \addtogroup gazebo_common Common 
  /// \{

  /// \brief A class which all plugins must inherit from
  class Plugin : public boost::signals::trackable
  {
    /// \brief Constructor
    public: Plugin();

    /// \brief Destructor
    public: virtual ~Plugin();

    /// \brief Load function
    public: virtual void Load( sdf::ElementPtr &_sdf ) = 0;

    /// \brief Get the name of the handler
    public: std::string GetFilename() const;

    /// \brief Get the short name of the handler
    public: std::string GetHandle() const;

    public: static PluginPtr Create(const std::string &_filename, 
                                    const std::string &_handle);

    protected: std::string filename;
    protected: std::string handle;
  };
  /// \}

#define GZ_REGISTER_PLUGIN(classname) \
extern "C" gazebo::Plugin *RegisterPlugin(); \
gazebo::Plugin *RegisterPlugin() \
{\
  return new classname();\
}
}
#endif
