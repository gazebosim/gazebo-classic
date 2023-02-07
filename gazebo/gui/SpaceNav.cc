/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <gazebo/gazebo_config.h>
#ifdef HAVE_SPNAV
#include <spnav.h>
// Status is defined in X11/Xlib.h which is included by spnav.h
// protobuf define a class named Status. To avoid conflicts, remove here
// the definition of Status.
#undef Status
#endif

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/SpaceNavPrivate.hh"
#include "gazebo/gui/SpaceNav.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
// Copied from libspnav in order to prevent an error message from
// spnav_open() when spnav daemon is not running.
int spnav_test_daemon(void)
{
#ifndef _WIN32
  int s;
  struct sockaddr_un addr;

  if ((s = socket(PF_UNIX, SOCK_STREAM, 0)) == -1)
    return -1;

  memset(&addr, 0, sizeof addr);
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, "/var/run/spnav.sock", sizeof(addr.sun_path));

  if (connect(s, (struct sockaddr*)&addr, sizeof addr) == -1)
  {
    close(s);
    return -1;
  }

  close(s);
#endif
  return 0;
}

/////////////////////////////////////////////////
SpaceNav::SpaceNav()
  : dataPtr(new SpaceNavPrivate)
{
}

/////////////////////////////////////////////////
SpaceNav::~SpaceNav()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
bool SpaceNav::Load()
{
  bool result = true;

  // reset button settings
  this->dataPtr->buttons[0] = 0;
  this->dataPtr->buttons[1] = 0;

#ifdef HAVE_SPNAV
  // Read whether to use spacenav in gui.ini
  bool enableSpacenav = getINIProperty<bool>("spacenav.enable", true);
  if (!enableSpacenav)
  {
    gzlog << "Spacenav functionality is disabled\n";
    return result;
  }

  // Read deadband from [spacenav] in gui.ini
  this->dataPtr->deadbandTrans.X(getINIProperty<double>(
      "spacenav.deadband_x", 0.1));
  this->dataPtr->deadbandTrans.Y(getINIProperty<double>(
      "spacenav.deadband_y", 0.1));
  this->dataPtr->deadbandTrans.Z(getINIProperty<double>(
      "spacenav.deadband_z", 0.1));

  this->dataPtr->deadbandRot.X(getINIProperty<double>(
      "spacenav.deadband_rx", 0.1));
  this->dataPtr->deadbandRot.Y(getINIProperty<double>(
      "spacenav.deadband_ry", 0.1));
  this->dataPtr->deadbandRot.Z(getINIProperty<double>(
      "spacenav.deadband_rz", 0.1));

  // Read topic from [spacenav] in gui.ini
  std::string topic = getINIProperty<std::string>("spacenav.topic",
                                                  "~/user_camera/joy_twist");

  // Get whether the spacename daemon exists and is running.
  int daemonRunning = spnav_test_daemon();

  if (daemonRunning >= 0 && spnav_open() >= 0)
  {
    this->dataPtr->node = transport::NodePtr(new transport::Node());
    this->dataPtr->node->TryInit(common::Time::Maximum());
    this->dataPtr->joyPub = this->dataPtr->node->Advertise<msgs::Joystick>(
        topic);

    this->dataPtr->pollThread = new boost::thread(
        boost::bind(&SpaceNav::Run, this));
  }
  else if (daemonRunning >= 0)
  {
    gzerr << "Unable to open space navigator device."
      << "Please make sure you have run spacenavd as root.\n";
    result = false;
  }
  else
  {
    gzlog << "No spacenav daemon found. Spacenav functionality is disabled\n";
  }
#endif

  return result;
}

/////////////////////////////////////////////////
void SpaceNav::Run()
{
#ifdef HAVE_SPNAV
  spnav_event sev;

  this->dataPtr->stop = false;
  while (!this->dataPtr->stop)
  {
    msgs::Joystick joystickMsg;

    // add button state (26 bouttons)
    for (unsigned int i = 0; i < 26; ++i)
      joystickMsg.add_buttons(this->dataPtr->buttons[i]);

    // bool joyStale = false;
    bool queueEmpty = false;

    switch (spnav_poll_event(&sev))
    {
      // spnav_poll_event returns 0 when no event is present
      case 0:
        queueEmpty = true;
        break;

      case SPNAV_EVENT_BUTTON:
        // update button press
         if (sev.button.bnum < 26)
        {
            this->dataPtr->buttons[sev.button.bnum] = sev.button.press;
            joystickMsg.mutable_buttons()->Set(sev.button.bnum, sev.button.press);
            this->dataPtr->joyPub->Publish(joystickMsg);
        }
        else
        {
            gzlog << "Spacenav bouton unknown id \n";
        }
        break;

      case SPNAV_EVENT_MOTION:
        joystickMsg.mutable_translation()->set_x(
            this->Deadband(this->dataPtr->deadbandTrans.X(),
              sev.motion.z / SCALE));
        joystickMsg.mutable_translation()->set_y(
            this->Deadband(this->dataPtr->deadbandTrans.Y(),
              -sev.motion.x / SCALE));
        joystickMsg.mutable_translation()->set_z(
            this->Deadband(this->dataPtr->deadbandTrans.Z(),
              sev.motion.y / SCALE));

        joystickMsg.mutable_rotation()->set_x(
            this->Deadband(this->dataPtr->deadbandRot.X(),
              sev.motion.rz / SCALE));
        joystickMsg.mutable_rotation()->set_y(
            this->Deadband(this->dataPtr->deadbandRot.Y(),
              -sev.motion.rx / SCALE));
        joystickMsg.mutable_rotation()->set_z(
            this->Deadband(this->dataPtr->deadbandRot.Z(),
              sev.motion.ry / SCALE));

        this->dataPtr->joyPub->Publish(joystickMsg);
        break;

      default:
        break;
    }

    if (queueEmpty)
      common::Time::NSleep(1000000);
  }
#endif
}

/////////////////////////////////////////////////
double SpaceNav::Deadband(double _deadband, double _value) const
{
  double abs = std::abs(_value);

  return abs < _deadband ? 0 :
    (_value - (abs/_value * _deadband)) / (1 - _deadband);
}
