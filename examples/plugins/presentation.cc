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
#include <boost/bind.hpp>

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/GUIOverlay.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  class State
  {
    public: State(const std::string &_slide,
                  const math::Vector3 _offset,
                  double _pitch, double _yaw, double _duration)
            : slide(_slide), offset(_offset),
              pitch(_pitch), yaw(_yaw), duration(_duration) {}
    public: std::string slide;
    public: math::Vector3 offset;
    public: double pitch;
    public: double yaw;
    public: double duration;
  };

  class Presentation : public SystemPlugin
  {
    public: void Load()
    {
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      this->connections.push_back(
          gui::Events::ConnectKeyPress(
            boost::bind(&Presentation::KeyPress, this, _1)));

      this->states.push_back(new State("default::slide1",
            math::Vector3(0, 0, 3.6), M_PI*0.5, 0, 0.6));
      this->states.push_back(new State("default::slide1",
            math::Vector3(0, 0, 3.6), M_PI*0.5, 0, 0.6));
      this->states.push_back(new State("default::slide2",
            math::Vector3(0, 0, 0.03), M_PI*0.5, 0, 0.6));
      this->states.push_back(new State("default::slide3",
            math::Vector3(0, 0, 0.03), M_PI*0.5, -M_PI*0.5, 0.5));
      this->states.push_back(new State("default::slide4",
            math::Vector3(0, 0, 0.03), M_PI*0.5, -M_PI*0.5, 0.5));
      this->states.push_back(new State("default::slide5",
            math::Vector3(0, 0, 0.03), M_PI*0.5, -M_PI*0.5, 0.5));
      this->states.push_back(new State("default::slide6",
            math::Vector3(0, 0, 0.035), M_PI*0.5, -M_PI*0.5, 0.5));
      this->states.push_back(new State("default::slide7",
            math::Vector3(0, 0, 0.035), M_PI*0.5, -M_PI*0.5, 0.5));

      this->states.push_back(new State("default::slide8",
            math::Vector3(0, 0, 3.5), M_PI*0.5, 0, 0));

      this->states.push_back(new State("default::slide9",
            math::Vector3(0, 0, .037), M_PI*0.5, 0, 0.5));
      this->states.push_back(new State("default::slide10",
            math::Vector3(0, 0, .037), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide8",
            math::Vector3(0, 0, 3.5), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide11",
            math::Vector3(0, 0, 0.037), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide8",
            math::Vector3(0, 0, 3.5), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide12",
            math::Vector3(0, 0, 0.037), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide8",
            math::Vector3(0, 0, 3.5), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide13",
            math::Vector3(0, 0, 0.037), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide8",
            math::Vector3(0, 0, 3.5), M_PI*0.5, 0, 0.5));

      this->states.push_back(new State("default::slide14",
            math::Vector3(0, 0, 3.5), M_PI*0.5, M_PI*0.5, 0.5));

      this->states.push_back(new State("default::slide15",
            math::Vector3(0, 0, 3.5), M_PI*0.5, M_PI, 0.5));



      this->iter = this->states.begin();
    }
    private: void Init()
             {
             }

    private: void KeyPress(const std::string &_key)
             {
               rendering::UserCameraPtr userCam = gui::get_active_camera();

               if (_key == " ")
                 this->iter++;
               else if (_key == "b")
                 this->iter--;

               if (this->iter != this->states.end())
                 this->Update();
               else
                 this->iter = this->states.end()-1;
             }

    private: void Update()
             {
               rendering::UserCameraPtr userCam = gui::get_active_camera();
               rendering::Scene *scene = userCam->GetScene();
               rendering::VisualPtr visual =
                 scene->GetVisual((*this->iter)->slide);
               math::Pose pose = visual->GetWorldPose();
               math::Vector3 p = pose.pos +  (*this->iter)->offset;

               if ((*this->iter)->duration >0)
               {
                 userCam->MoveToPosition(p, (*this->iter)->pitch,
                                         (*this->iter)->yaw,
                                         (*this->iter)->duration);
               }
               else
               {
                 pose.pos = p;
                 pose.rot.SetFromEuler(
                     math::Vector3(0, (*this->iter)->pitch,
                                   (*this->iter)->yaw));
                 userCam->SetWorldPose(pose);
               }
             }

    private: transport::NodePtr node;
    private: std::vector<event::ConnectionPtr> connections;
    private: std::vector<State*> states;
    private: std::vector<State*>::iterator iter;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(Presentation)
}
