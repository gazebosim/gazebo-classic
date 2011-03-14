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
#ifndef HANDLER_HH
#define HANDLER_HH

#include <string>
#include <boost/signals.hpp>

namespace gazebo
{
	namespace common
{
  class Event;

  class Plugin : public boost::signals::trackable
  {
    public: Plugin();
    public: virtual ~Plugin();
    public: virtual void Load() = 0;

    /// \brief Get the name of the handler
    public: std::string GetFilename() const;

    /// \brief Get the short name of the handler
    public: std::string GetHandle() const;

    public: static Plugin *Create(const std::string &filename, const std::string &handle);

    protected: std::string filename;
    protected: std::string handle;
  };
}

#define GZ_REGISTER_PLUGIN(name, classname) \
extern "C" gazebo::Plugin *RegisterPlugin(); \
gazebo::Plugin *RegisterPlugin() \
{\
  return new classname();\
}
}
#endif
