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
#ifndef _GAZEBO_GUIIFACE_HH_
#define _GAZEBO_GUIIFACE_HH_

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem.hpp>
#include "gazebo/rendering/RenderingIface.hh"

extern boost::property_tree::ptree g_propTree;
namespace gazebo
{
  namespace gui
  {
    /// \brief Load the graphical interface.
    /// \return True on success.
    bool load();

    void init();

    bool run(int _argc, char **_argv);
    void stop();

    void set_world(const std::string& _name);
    std::string get_world();

    void set_active_camera(rendering::UserCameraPtr _cam);
    rendering::UserCameraPtr get_active_camera();
    void clear_active_camera();

    unsigned int get_entity_id(const std::string &_name);
    bool has_entity_name(const std::string &_name);

    /// \brief Load an INI configuration file.
    /// \param[in] _file Full path to the INI file.
    /// \return True on success.
    bool loadINI(const boost::filesystem::path &_file);

    /// \brief Get a property from the GUI INI file.
    /// \param[in] _key String based key[ SECTION.VALUE ]
    /// \param[in] _default Default value to use if property is not found.
    /// \return Property value for the key.
    template<typename T>
    T getINIProperty(const std::string &_key, const T &_default)
    {
      try
      {
        return g_propTree.get<T>(_key);
      }
      catch(...)
      {
      }

      return _default;
    }

    /// \brief Set a value in the INI property tree. Note, this doesn't save
    /// properties to disk.
    /// \param[in] _key Key for the value, such as "geometry.x"
    /// \param[in] _value Value for the key
    /// \sa gui::saveINI
    template<typename T>
    bool setINIProperty(const std::string &_key, const T &_value)
    {
      g_propTree.put(_key, _value);
      return true;
    }

    /// \brief Save the configuration parameters to file.
    /// \param[in] _file Filename in which to write the values.
    /// \return True on success.
    bool saveINI(const boost::filesystem::path &_file);
  }
}
#endif
