/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif
#include <algorithm>
#include <mutex>
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
  this->stateSub = this->gzNode->Subscribe<msgs::Cessna>(
    "~/cessna_c172/state", &CessnaGUIPlugin::OnState, this);

  // Connect hotkeys.
  QShortcut *increaseThrust = new QShortcut(QKeySequence("w"), this);
  QObject::connect(increaseThrust, SIGNAL(activated()), this,
      SLOT(OnIncreaseThrust()));

  QShortcut *decreaseThrust = new QShortcut(QKeySequence("s"), this);
  QObject::connect(decreaseThrust, SIGNAL(activated()), this,
      SLOT(OnDecreaseThrust()));

  QShortcut *increaseFlaps = new QShortcut(QKeySequence("g"), this);
  QObject::connect(increaseFlaps, SIGNAL(activated()), this,
      SLOT(OnIncreaseFlaps()));

  QShortcut *decreaseFlaps = new QShortcut(QKeySequence("b"), this);
  QObject::connect(decreaseFlaps, SIGNAL(activated()), this,
      SLOT(OnDecreaseFlaps()));

  QShortcut *increaseRoll = new QShortcut(QKeySequence(Qt::Key_Left), this);
  QObject::connect(increaseRoll, SIGNAL(activated()), this,
      SLOT(OnIncreaseRoll()));

  QShortcut *decreaseRoll = new QShortcut(QKeySequence(Qt::Key_Right), this);
  QObject::connect(decreaseRoll, SIGNAL(activated()), this,
      SLOT(OnDecreaseRoll()));

  QShortcut *increaseElevators =
    new QShortcut(QKeySequence(Qt::Key_Down), this);
  QObject::connect(increaseElevators, SIGNAL(activated()), this,
      SLOT(OnIncreaseElevators()));

  QShortcut *decreaseElevators = new QShortcut(QKeySequence(Qt::Key_Up), this);
  QObject::connect(decreaseElevators, SIGNAL(activated()), this,
      SLOT(OnDecreaseElevators()));

  QShortcut *increaseRudder = new QShortcut(QKeySequence("d"), this);
  QObject::connect(increaseRudder, SIGNAL(activated()), this,
      SLOT(OnIncreaseRudder()));

  QShortcut *decreaseRudder = new QShortcut(QKeySequence("a"), this);
  QObject::connect(decreaseRudder, SIGNAL(activated()), this,
      SLOT(OnDecreaseRudder()));

  QShortcut *presetTakeOff = new QShortcut(QKeySequence('1'), this);
  QObject::connect(presetTakeOff, SIGNAL(activated()), this,
      SLOT(OnPresetTakeOff()));

  QShortcut *presetCruise = new QShortcut(QKeySequence('2'), this);
  QObject::connect(presetCruise, SIGNAL(activated()), this,
      SLOT(OnPresetCruise()));

  QShortcut *presetLanding = new QShortcut(QKeySequence('3'), this);
  QObject::connect(presetLanding, SIGNAL(activated()), this,
      SLOT(OnPresetLanding()));
}

/////////////////////////////////////////////////
CessnaGUIPlugin::~CessnaGUIPlugin()
{
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnState(ConstCessnaPtr &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Refresh the state.
  this->state = *_msg;
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseThrust()
{
  float thrust;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust = this->state.cmd_propeller_speed();
  }

  msgs::Cessna msg;
  thrust = std::min(thrust + 0.1f, 1.0f);
  msg.set_cmd_propeller_speed(thrust);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseThrust()
{
  float thrust;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    thrust = this->state.cmd_propeller_speed();
  }

  msgs::Cessna msg;
  thrust = std::max(thrust - 0.1f, 0.0f);
  msg.set_cmd_propeller_speed(thrust);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseFlaps()
{
  math::Angle flap;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    flap.SetFromRadian(this->state.cmd_left_flap());
  }

  msgs::Cessna msg;
  if (flap.Degree() < 30)
  {
    flap += this->angleStep;
    msg.set_cmd_left_flap(flap.Radian());
    msg.set_cmd_right_flap(flap.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseFlaps()
{
  math::Angle flap;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    flap.SetFromRadian(this->state.cmd_left_flap());
  }

  msgs::Cessna msg;
  if (flap.Degree() > -30)
  {
    flap -= this->angleStep;
    msg.set_cmd_left_flap(flap.Radian());
    msg.set_cmd_right_flap(flap.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseRoll()
{
  math::Angle aileron;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    aileron.SetFromRadian(this->state.cmd_left_aileron());
  }

  msgs::Cessna msg;
  if (aileron.Degree() < 30)
  {
    aileron += this->angleStep;
    msg.set_cmd_left_aileron(aileron.Radian());
    msg.set_cmd_right_aileron(-aileron.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseRoll()
{
  math::Angle aileron;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    aileron.SetFromRadian(this->state.cmd_left_aileron());
  }

  msgs::Cessna msg;
  if (aileron.Degree() > -30)
  {
    aileron -= this->angleStep;
    msg.set_cmd_left_aileron(aileron.Radian());
    msg.set_cmd_right_aileron(-aileron.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseElevators()
{
  math::Angle elevators;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    elevators.SetFromRadian(this->state.cmd_elevators());
  }

  msgs::Cessna msg;
  if (elevators.Degree() < 30)
  {
    elevators += this->angleStep;
    msg.set_cmd_elevators(elevators.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseElevators()
{
  math::Angle elevators;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    elevators.SetFromRadian(this->state.cmd_elevators());
  }

  msgs::Cessna msg;
  if (elevators.Degree() > -30)
  {
    elevators -= this->angleStep;
    msg.set_cmd_elevators(elevators.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnIncreaseRudder()
{
  math::Angle rudder;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    rudder.SetFromRadian(this->state.cmd_rudder());
  }

  msgs::Cessna msg;
  if (rudder.Degree() < 30)
  {
    rudder += this->angleStep;
    msg.set_cmd_rudder(rudder.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnDecreaseRudder()
{
  math::Angle rudder;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    rudder.SetFromRadian(this->state.cmd_rudder());
  }

  msgs::Cessna msg;
  if (rudder.Degree() > -30)
  {
    rudder -= this->angleStep;
    msg.set_cmd_rudder(rudder.Radian());
    this->controlPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnPresetTakeOff()
{
  msgs::Cessna msg;
  msg.set_cmd_propeller_speed(0.8);
  msg.set_cmd_left_aileron(-0.017);
  msg.set_cmd_right_aileron(0.017);
  msg.set_cmd_left_flap(0);
  msg.set_cmd_right_flap(0);
  msg.set_cmd_elevators(0.033);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnPresetCruise()
{
  msgs::Cessna msg;
  msg.set_cmd_propeller_speed(0.6);
  msg.set_cmd_left_aileron(0);
  msg.set_cmd_right_aileron(0);
  msg.set_cmd_left_flap(0);
  msg.set_cmd_right_flap(0);
  msg.set_cmd_elevators(0.12);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void CessnaGUIPlugin::OnPresetLanding()
{
  msgs::Cessna msg;
  msg.set_cmd_propeller_speed(0.3);
  msg.set_cmd_left_aileron(0);
  msg.set_cmd_right_aileron(0);
  msg.set_cmd_left_flap(0);
  msg.set_cmd_right_flap(0);
  msg.set_cmd_elevators(0.16);
  msg.set_cmd_rudder(-0.035);
  this->controlPub->Publish(msg);
}
