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

// Include Rand.hh first due to compilation error on osx (boost #5010)
// https://svn.boost.org/trac/boost/ticket/5010
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~SystemGUI()
    {
      this->connections.clear();
      if (this->userCam)
        this->userCam->EnableSaveFrame(false);
      this->userCam.reset();
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      this->connections.push_back(
          event::Events::ConnectPreRender(
            boost::bind(&SystemGUI::Update, this)));
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
      if (!this->userCam)
      {
        // Get a pointer to the active user camera
        this->userCam = gui::get_active_camera();

        // Enable saving frames
        this->userCam->EnableSaveFrame(true);

        // Specify the path to save frames into
        this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
      }

      // Get scene pointer
      rendering::ScenePtr scene = rendering::get_scene();

      // Wait until the scene is initialized.
      if (!scene || !scene->GetInitialized())
        return;

      // Look for a specific visual by name.
      if (scene->GetVisual("ground_plane"))
        std::cout << "Has ground plane visual\n";
    }

    /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
