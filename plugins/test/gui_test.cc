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
#include <boost/bind.hpp>

#include "gui/Gui.hh"
#include "rendering/Scene.hh"
#include "rendering/Camera.hh"
#include "rendering/DepthCamera.hh"
#include "rendering/RenderEngine.hh"
#include "rendering/UserCamera.hh"
#include "rendering/GUIOverlay.hh"
#include "gazebo.h"

namespace gazebo
{
  class GUITest : public GUIPlugin
  {
    public: void Load()
    {
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->connections.push_back( 
          event::Events::ConnectPreRender( 
            boost::bind(&GUITest::PreRender, this) ) );


    }

    private: void PreRender()
             {
               static bool connected = false;

               if (!connected)
               {
                 rendering::UserCameraPtr userCam = gui::get_active_camera();
                 rendering::ScenePtr scene = rendering::RenderEngine::Instance()->GetScene("default");
                 if (!scene)
                   gzerr << "Unable to find scene[default]\n";

                 this->camera = scene->CreateDepthCamera("my_camera");
                 //this->camera = scene->CreateCamera("my_camera");

                 this->camera->Load();
                 this->camera->Init();
                 this->camera->SetClipDist(0.1, 20);
                 this->camera->SetCaptureData(true);

                 //this->camera->CreateRenderTexture("help_me");
                 this->camera->CreateDepthTexture("help_me");

                 this->camera->SetWorldPosition( math::Vector3(0,0,.5) );

                 if (!camera)
                   gzerr << "Unable to find camera[camera]\n";

                 userCam->GetGUIOverlay()->AttachCameraToImage( this->camera, 
                     "Root/CameraView");

                 connected = true;
               }

             }

    private: void Init()
    {
      rendering::UserCameraPtr userCam = gui::get_active_camera();
      if (userCam && userCam->GetGUIOverlay())
      {
        userCam->GetGUIOverlay()->LoadLayout( "gui_test.layout" );
      }

    }

    private: transport::NodePtr node;
    private: std::vector<event::ConnectionPtr> connections;
    private: rendering::DepthCameraPtr camera;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_GUI_PLUGIN(GUITest)
}
 
