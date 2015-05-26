/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <algorithm>
#include <string>

#include "gazebo/rendering/Scene.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/MovableText.hh"
#include "gazebo/rendering/OculusCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "plugins/DroneTimerPlugin.hh"

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(DroneTimerPlugin)

/////////////////////////////////////////////////
DroneTimerPlugin::DroneTimerPlugin()
{
  this->text = NULL;
}

/////////////////////////////////////////////////
void DroneTimerPlugin::Load(rendering::VisualPtr _vis, sdf::ElementPtr /*_sdf*/)
{
  this->vis = _vis;

  // Connect to the render signal
  this->updateConnection = event::Events::ConnectPreRender(
        boost::bind(&DroneTimerPlugin::Update, this));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->timerSub = this->node->Subscribe("~/timer",
      &DroneTimerPlugin::OnTime, this);
}

/////////////////////////////////////////////////
DroneTimerPlugin::~DroneTimerPlugin()
{
  this->vis->DetachObjects();
  delete this->text;
  this->text = NULL;
}

/////////////////////////////////////////////////
void DroneTimerPlugin::Reset()
{
}

/////////////////////////////////////////////////
void DroneTimerPlugin::Update()
{
  if (!this->text)
  {
    this->text = new rendering::MovableText();
    this->text->Load("drone_timer", "Time remaining:", "Console", 4000,
        common::Color(255, 255, 255));
    this->text->SetShowOnTop(true);
    this->text->SetTextAlignment(rendering::MovableText::H_CENTER,
        rendering::MovableText::V_ABOVE);
    this->vis->GetSceneNode()->attachObject(this->text);
    this->vis->SetVisible(true);
    this->text->Update();
  }
  std::ostringstream stream;
  if (this->sec >= 0)
  {
    stream << "Time "
      << std::setw(2) << std::setfill('0') << this->min << ":"
      << std::setw(2) << std::setfill('0') << this->sec;
  }
  else
    stream << "Game over!";

  this->text->SetText(stream.str());
}

/////////////////////////////////////////////////
void DroneTimerPlugin::OnTime(ConstTimePtr &_msg)
{
  this->sec = _msg->sec();
  this->min = sec / 60;
  this->sec -= min * 60;
}
