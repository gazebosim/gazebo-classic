/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/util/system.hh"

extern GZ_GUI_VISIBLE boost::property_tree::ptree g_propTree;
namespace gazebo
{
  namespace gui
  {
    class MainWindow;

    /// \brief Load the graphical interface.
    /// \return True on success.
    GZ_GUI_VISIBLE
    bool load();

    GZ_GUI_VISIBLE
    void init();

    GZ_GUI_VISIBLE
    bool run(int _argc, char **_argv);
    GZ_GUI_VISIBLE
    void stop();

    GZ_GUI_VISIBLE
    void set_world(const std::string& _name);
    GZ_GUI_VISIBLE
    std::string get_world();

    GZ_GUI_VISIBLE
    void set_active_camera(rendering::UserCameraPtr _cam);
    GZ_GUI_VISIBLE
    rendering::UserCameraPtr get_active_camera();
    GZ_GUI_VISIBLE
    void clear_active_camera();

    /// \brief Return a pointer to the main graphical window.
    GZ_GUI_VISIBLE
    MainWindow *get_main_window();

    GZ_GUI_VISIBLE
    unsigned int get_entity_id(const std::string &_name);
    GZ_GUI_VISIBLE
    bool has_entity_name(const std::string &_name);

    /// \brief Locate and load the INI configuration file.
    /// If the GAZEBO_GUI_INI_FILE environment variable is set and contains
    /// valid content, load and return true. If GAZEBO_GUI_INI_FILE is
    /// not set, load from ~/.gazebo/gui.ini (a gui.ini file will be created
    /// if it doesn't exist) and return true.
    /// If GAZEBO_GUI_INI_FILE is set but the path does not exist, or if it
    /// exists and contains invalid content, do not load, and return false.
    /// \param[in] _file Path to a gui.ini file. This will override
    /// the environment variables.
    /// \return True if an INI file was loaded, false otherwise.
    GZ_GUI_VISIBLE
    bool loadINI(boost::filesystem::path _file = "");

    /// \brief Get a property from the GUI INI file.
    /// \param[in] _key String based key[ SECTION.VALUE ]
    /// \param[in] _default Default value to use if property is not found.
    /// \return Property value for the key.
    template<typename T>
    GZ_GUI_VISIBLE
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
    GZ_GUI_VISIBLE
    bool setINIProperty(const std::string &_key, const T &_value)
    {
      g_propTree.put(_key, _value);
      return true;
    }

    /// \brief Save the configuration parameters to file.
    /// \param[in] _file Filename in which to write the values.
    /// \return True on success.
    GZ_GUI_VISIBLE
    bool saveINI(const boost::filesystem::path &_file);
  }
}
#endif
