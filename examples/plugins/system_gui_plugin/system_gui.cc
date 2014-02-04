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
#include <boost/filesystem.hpp>

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
    }

    private: void Init()
    {
      // Get a pointer to the active user camera
      this->userCam = gui::get_active_camera();

      // Enable saving frames
      this->userCam->EnableSaveFrame(true);

      // Get a path suitable for temporary files
      boost::system::error_code ec;
      boost::filesystem::path tmp = boost::filesystem::temp_directory_path(ec);
      if (ec != 0)
      {
        gzerr << "Failed creating temp directory: " << ec.message() << "\n";
        return;
      }

      // Specify the path to save frames into
      this->userCam->SetSaveFramePathname((tmp / "gazebo_frames").string());
    }

    private: rendering::UserCameraPtr userCam;
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
