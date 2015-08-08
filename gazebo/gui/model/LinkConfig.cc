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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/LinkConfig.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LinkConfig::LinkConfig()
{
  this->setObjectName("LinkConfig");

  // Message
  msgs::Link linkMsg;

  // ConfigWidget
  this->configWidget = new ConfigWidget;
  configWidget->Load(&linkMsg);

  // set default values
  // TODO: auto-fill them with SDF defaults
  this->configWidget->SetDoubleWidgetValue("inertial::mass", 1.0);
  this->configWidget->SetDoubleWidgetValue("inertial::ixx", 1.0);
  this->configWidget->SetDoubleWidgetValue("inertial::iyy", 1.0);
  this->configWidget->SetDoubleWidgetValue("inertial::izz", 1.0);
  this->configWidget->SetBoolWidgetValue("gravity", true);
  this->configWidget->SetBoolWidgetValue("self_collide", false);
  this->configWidget->SetBoolWidgetValue("kinematic", false);

  this->configWidget->SetWidgetVisible("id", false);
  this->configWidget->SetWidgetVisible("name", false);
  this->configWidget->SetWidgetVisible("canonical", false);
  this->configWidget->SetWidgetVisible("enabled", false);
  this->configWidget->SetWidgetReadOnly("id", true);
  this->configWidget->SetWidgetReadOnly("name", true);
  this->configWidget->SetWidgetReadOnly("canonical", true);
  this->configWidget->SetWidgetReadOnly("enabled", true);

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->configWidget);
  scrollArea->setWidgetResizable(true);

  // Layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->addWidget(scrollArea);

  this->setLayout(generalLayout);
}

/////////////////////////////////////////////////
LinkConfig::~LinkConfig()
{
}

/////////////////////////////////////////////////
void LinkConfig::Update(ConstLinkPtr _linkMsg)
{
  this->configWidget->UpdateFromMsg(_linkMsg.get());
}

/////////////////////////////////////////////////
void LinkConfig::SetPose(const ignition::math::Pose3d &_pose)
{
  this->configWidget->SetPoseWidgetValue("pose", math::Pose(_pose));
}

/////////////////////////////////////////////////
void LinkConfig::SetMass(const double _mass)
{
  this->configWidget->SetDoubleWidgetValue("inertial::mass", _mass);
}

/////////////////////////////////////////////////
void LinkConfig::SetInertiaMatrix(const double _ixx, const double _iyy,
    const double _izz, const double _ixy, const double _ixz, const double _iyz)
{
  this->configWidget->SetDoubleWidgetValue("inertial::ixx", _ixx);
  this->configWidget->SetDoubleWidgetValue("inertial::iyy", _iyy);
  this->configWidget->SetDoubleWidgetValue("inertial::izz", _izz);
  this->configWidget->SetDoubleWidgetValue("inertial::ixy", _ixy);
  this->configWidget->SetDoubleWidgetValue("inertial::ixz", _ixz);
  this->configWidget->SetDoubleWidgetValue("inertial::iyz", _iyz);
}

/////////////////////////////////////////////////
void LinkConfig::SetInertialPose(const ignition::math::Pose3d &_pose)
{
  this->configWidget->SetPoseWidgetValue("inertial::pose", _pose);
}

/////////////////////////////////////////////////
msgs::Link *LinkConfig::GetData() const
{
  return dynamic_cast<msgs::Link *>(this->configWidget->GetMsg());
}
