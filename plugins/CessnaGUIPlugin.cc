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
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/Actions.hh>
#include "CessnaGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(CessnaGUIPlugin)

/////////////////////////////////////////////////
CessnaGUIPlugin::CessnaGUIPlugin()
  : GUIPlugin()
{
  // This is needed to avoid the creation of a black widget with default size.
  this->resize(0, 0);

  // Set the increment or decrement in angle per key pressed.
  this->angleStep.SetFromDegree(1.0);

  // Initialize transport.
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->controlPub =
    this->gzNode->Advertise<msgs::Cessna>("~/cessna_c172/control");

  // Connect hotkeys
  QShortcut *increaseThrust = new QShortcut(QKeySequence("a"), this);
  QObject::connect(increaseThrust, SIGNAL(activated()), this,
      SLOT(OnIncreaseThrust()));

  QShortcut *decreaseThrust = new QShortcut(QKeySequence("z"), this);
  QObject::connect(decreaseThrust, SIGNAL(activated()), this,
      SLOT(OnDecreaseThrust()));

  QShortcut *increaseFlaps = new QShortcut(QKeySequence("s"), this);
  QObject::connect(increaseFlaps, SIGNAL(activated()), this,
      SLOT(OnIncreaseFlaps()));

  QShortcut *decreaseFlaps = new QShortcut(QKeySequence("x"), this);
  QObject::connect(decreaseFlaps, SIGNAL(activated()), this,
      SLOT(OnDecreaseFlaps()));

  QShortcut *increaseElevators = new QShortcut(QKeySequence("d"), this);
  QObject::connect(increaseElevators, SIGNAL(activated()), this,
      SLOT(OnIncreaseElevators()));

  QShortcut *decreaseElevators = new QShortcut(QKeySequence("c"), this);
  QObject::connect(decreaseElevators, SIGNAL(activated()), this,
      SLOT(OnDecreaseElevators()));

  QShortcut *increaseRudder = new QShortcut(QKeySequence("f"), this);
  QObject::connect(increaseRudder, SIGNAL(activated()), this,
      SLOT(OnIncreaseRudder()));

  QShortcut *decreaseRudder = new QShortcut(QKeySequence("v"), this);
  QObject::connect(decreaseRudder, SIGNAL(activated()), this,
      SLOT(OnDecreaseRudder()));
}

/////////////////////////////////////////////////
CessnaGUIPlugin::~CessnaGUIPlugin()
{
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseThrust()
{
  msgs::Cessna msg;
  this->targetThrust = std::min(this->targetThrust + 1, 100);
  msg.set_propeller_speed(this->targetThrust);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseThrust()
{
  msgs::Cessna msg;
  this->targetThrust = std::max(this->targetThrust - 1, 0);
  msg.set_propeller_speed(this->targetThrust);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseFlaps()
{
  msgs::Cessna msg;
  if (this->targetFlaps.Degree() < 30)
    this->targetFlaps += this->angleStep;

  msg.set_left_aileron(this->targetFlaps.Radian());
  msg.set_left_flap(this->targetFlaps.Radian());
  msg.set_right_aileron(this->targetFlaps.Radian());
  msg.set_right_flap(this->targetFlaps.Radian());
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseFlaps()
{
  msgs::Cessna msg;
  if (this->targetFlaps.Degree() > -30)
    this->targetFlaps -= this->angleStep;

  msg.set_left_aileron(this->targetFlaps.Radian());
  msg.set_left_flap(this->targetFlaps.Radian());
  msg.set_right_aileron(this->targetFlaps.Radian());
  msg.set_right_flap(this->targetFlaps.Radian());
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseElevators()
{
  msgs::Cessna msg;
  if (this->targetElevators.Degree() < 30)
    this->targetElevators += this->angleStep;

  msg.set_elevators(this->targetElevators.Radian());
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseElevators()
{
  msgs::Cessna msg;
  if (this->targetElevators.Degree() > -30)
    this->targetElevators -= this->angleStep;

  msg.set_elevators(this->targetElevators.Radian());
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseRudder()
{
  msgs::Cessna msg;
  if (this->targetRudder.Degree() < 30)
    this->targetRudder += this->angleStep;

  msg.set_rudder(this->targetRudder.Radian());
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseRudder()
{
  msgs::Cessna msg;
  if (this->targetRudder.Degree() > -30)
    this->targetRudder -= this->angleStep;

  msg.set_rudder(this->targetRudder.Radian());
  this->controlPub->Publish(msg);
}
