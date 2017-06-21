/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <gazebo/common/PID.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include "KeysToPosesPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(KeysToPosesPlugin)

/////////////////////////////////////////////////
KeysToPosesPlugin::KeysToPosesPlugin()
{
}

/////////////////////////////////////////////////
KeysToPosesPlugin::~KeysToPosesPlugin()
{
}

/////////////////////////////////////////////////
void KeysToPosesPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Load params from SDF
  if (!_sdf->HasElement("map") || !_sdf->HasElement("toggle_key"))
  {
    gzwarn << "Missing <map> element, not loading plugin."
        << std::endl;
    return;
  }

  this->toggleKey = _sdf->Get<int>("toggle_key");
  this->enabled = false;

  auto mapElem = _sdf->GetElement("map");
  auto linkName = mapElem->Get<std::string>("link");
  auto link = _model->GetLink(linkName);
  if (!link)
  {
    gzwarn << "Can't find link [" << linkName << "], not loading plugin."
           << std::endl;
    return;
  }

  if (!mapElem->HasAttribute("x_pos") ||
      !mapElem->HasAttribute("x_neg") ||
      !mapElem->HasAttribute("y_pos") ||
      !mapElem->HasAttribute("y_neg") ||
      !mapElem->HasAttribute("z_pos") ||
      !mapElem->HasAttribute("z_neg") ||
      !mapElem->HasAttribute("scale"))
  {
    gzwarn << "Missing some attribute, not loading plugin." << std::endl;
    return;
  }

  this->info.keys.push_back(mapElem->Get<int>("x_pos"));
  this->info.keys.push_back(mapElem->Get<int>("x_neg"));
  this->info.keys.push_back(mapElem->Get<int>("y_pos"));
  this->info.keys.push_back(mapElem->Get<int>("y_neg"));
  this->info.keys.push_back(mapElem->Get<int>("z_pos"));
  this->info.keys.push_back(mapElem->Get<int>("z_neg"));
  this->info.link = link;
  this->info.scale = mapElem->Get<double>("scale");

  // Initialize transport
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->keyboardSub = this->node->Subscribe("~/keyboard/keypress",
      &KeysToPosesPlugin::OnKeyPress, this, true);
}

/////////////////////////////////////////////////
void KeysToPosesPlugin::OnKeyPress(ConstAnyPtr &_msg)
{
  if (_msg->int_value() == this->toggleKey)
  {
    this->enabled = !this->enabled;

    if (this->enabled)
    {
      auto world = physics::get_world();
      auto model = world->GetModel("dummy");

      sdf::ElementPtr elem(new sdf::Element());
      sdf::initFile("joint.sdf", elem);

      sdf::readString(
       "<sdf version='" SDF_VERSION "'>\
        <joint name='plugin_joint' type='fixed'>\
         <parent>dummy</parent>\
         <child>" + this->info.link->GetName() + "</child>\
        </joint></sdf>", elem);

     this->fixedJoint = model->CreateJoint(elem);
     this->fixedJoint->Init();
     gzmsg << "Enabled" << std::endl;
   }
   else
   {
     this->fixedJoint.reset();
     gzmsg << "Disabled" << std::endl;
   }
  }

  if (!this->enabled || !this->fixedJoint)
    return;

      auto world = physics::get_world();
      auto model = world->GetModel("dummy");

  for (unsigned int i = 0; i < this->info.keys.size(); ++i)
  {
    if (_msg->int_value() != this->info.keys[i])
      continue;

    auto current = model->GetWorldPose().Ign();

    // +x
    if (i == 0)
    {
      current.Pos().X(current.Pos().X() + this->info.scale);
      gzmsg << "+X" << std::endl;
    }
    // -x
    else if (i == 1)
    {
      current.Pos().X(current.Pos().X() - this->info.scale);
      gzmsg << "-X" << std::endl;
    }
    // +y
    else if (i == 2)
    {
      current.Pos().Y(current.Pos().Y() + this->info.scale);
      gzmsg << "+Y" << std::endl;
    }
    // -y
    else if (i == 3)
    {
      current.Pos().Y(current.Pos().Y() - this->info.scale);
      gzmsg << "-Y" << std::endl;
    }
    // +z
    else if (i == 4)
    {
      current.Pos().Z(current.Pos().Z() + this->info.scale);
      gzmsg << "+Z" << std::endl;
    }
    // -z
    else if (i == 5)
    {
      current.Pos().Z(current.Pos().Z() - this->info.scale);
      gzmsg << "-Z" << std::endl;
    }

    model->SetWorldPose(current);
  }
}

