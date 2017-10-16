/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <linux/joystick.h>
#include <fcntl.h>
#include <thread>
#include <sys/stat.h>
#include <ignition/math/Helpers.hh>

#include "plugins/JoyPlugin.hh"

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(JoyPlugin)

/////////////////////////////////////////////////
JoyPlugin::JoyPlugin()
{
}

/////////////////////////////////////////////////
JoyPlugin::~JoyPlugin()
{
  this->stop = true;
  if (this->runThread)
    this->runThread->join();

  close(this->joyFd);
}

/////////////////////////////////////////////////
void JoyPlugin::Load(physics::WorldPtr /*_world*/, sdf::ElementPtr _sdf)
{
  std::string deviceFilename = _sdf->Get<std::string>("dev",
      "/dev/input/js0").first;

  bool opened = false;
  this->joyFd == -1;

  // Attempt to open the joystick
  for (int i = 0; i < 10 && !opened; ++i)
  {
    this->joyFd = open(deviceFilename.c_str(), O_RDONLY);

    if (joyFd != -1)
    {
      // Close and open the device to get a better intitial state.
      close(joyFd);
      joyFd = open(deviceFilename.c_str(), O_RDONLY);
      opened = true;
    }
    else
    {
      gzdbg << "Unable  to open joystick at " << deviceFilename
        << "Attemping again in 1 second\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // Stop if we couldn't open the joystick after N attempts
  if (this->joyFd == -1)
  {
    gzerr << "Unable  to open joystick at " << deviceFilename
          << ". The joystick will not work.\n";
    return;
  }

  this->stickyButtons = _sdf->Get<bool>("sticky_buttons",
                                        this->stickyButtons).first;

  float deadzone = ignition::math::clamp(
      _sdf->Get<float>("dead_zone", 0.05f).first,
      0.0f, 0.9f);

  float intervalRate = _sdf->Get<float>("rate", 0).first;
  float accumulationRate = _sdf->Get<float>("accumulation_rate", 1000).first;

  if (intervalRate <= 0)
    this->interval = 1.0f;
  else
    this->interval = 1.0f / intervalRate;

  if (accumulationRate <= 0)
    this->accumulationInterval = 0.0f;
  else
    this->accumulationInterval = 1.0f / accumulationRate;

  if (this->interval > this->accumulationInterval)
  {
    gzwarn << "Interval rate of [" << 1.0 / this->interval
      << " Hz] is greater than the accumulation rate of ["
      << 1.0 / this->accumulationInterval
      << " Hz]. Timing behavior is ill defined.\n";
  }

  this->unscaledDeadzone = 32767.0f * deadzone;
  this->axisScale = -1.0f / (1.0f - deadzone) / 32767.0f;

  // Create the publisher of joy messages
  this->pub = this->node.Advertise<ignition::msgs::Joy>("/joy");

  // Connect to the world update
  this->runThread = new std::thread(std::bind(&JoyPlugin::Run, this));
}

/////////////////////////////////////////////////
void JoyPlugin::Run()
{
  fd_set set;
  struct timeval tv;
  bool timeoutSet = false;
  bool accumulate = false;
  bool accumulating = false;

  while (!this->stop)
  {
    FD_ZERO(&set);
    FD_SET(this->joyFd, &set);

    int selectOut = select(this->joyFd+1, &set, NULL, NULL, &tv);

    if (selectOut == -1)
    {
      tv.tv_sec = 0;
      tv.tv_usec = 0;

      gzdbg << "Joystick might be closed\n";
      if (!this->stop)
        continue;
      else
        break;
    }
    else if (this->stop)
      break;

    js_event event;

    if (FD_ISSET(this->joyFd, &set))
    {
      if (read(this->joyFd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
      {
        gzdbg << "Joystick read failed, might be closed\n";
        return;
      }

      // Set the time stamp
      gazebo::common::Time time = gazebo::common::Time::GetWallTime();
      joyMsg.mutable_header()->mutable_stamp()->set_sec(time.sec);
      joyMsg.mutable_header()->mutable_stamp()->set_nsec(time.nsec);

      float value = event.value;
      switch(event.type)
      {
        case JS_EVENT_BUTTON:
        case JS_EVENT_BUTTON | JS_EVENT_INIT:
          {
            // Update number of buttons
            if(event.number >= this->joyMsg.buttons_size())
            {
              this->joyMsg.mutable_buttons()->Resize(event.number+1, 0.0f);
              this->lastJoyMsg.mutable_buttons()->Resize(event.number+1, 0.0f);
              this->stickyButtonsJoyMsg.mutable_buttons()->Resize(
                  event.number+1, 0.0f);
            }

            // Update the button
            this->joyMsg.set_buttons(event.number,
                !ignition::math::equal(value, 0.0f) ? 1 : 0);

            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            if (!(event.type & JS_EVENT_INIT))
              accumulate = true;
            else
              accumulate = false;
            break;
          }
        case JS_EVENT_AXIS:
        case JS_EVENT_AXIS | JS_EVENT_INIT:
          {
            if (event.number >= this->joyMsg.axes_size())
            {
              this->joyMsg.mutable_axes()->Resize(event.number+1, 0.0f);
              this->lastJoyMsg.mutable_axes()->Resize(event.number+1, 0.0f);
              this->stickyButtonsJoyMsg.mutable_axes()->Resize(
                  event.number+1, 0.0f);
            }

            // Smooth the deadzone
            if (value < -this->unscaledDeadzone)
              value += this->unscaledDeadzone;
            else if (value > this->unscaledDeadzone)
              value -= this->unscaledDeadzone;
            else
              value = 0.0f;

            this->joyMsg.set_axes(event.number, value * this->axisScale);

            // Will wait a bit before sending to try to combine events.
            accumulate = true;
            break;
          }
        default:
          {
            gzwarn << "Unknown event type: time[" << event.time << "] "
              << "value[" << value << "] "
              << "type[" << event.type << "h] "
              << "number["<< event.number << "]" << std::endl;
            break;
          }
      }
    }
    // Assume that the timer has expired.
    else if (timeoutSet)
      accumulate = false;

    if (!accumulate)
    {
      if (this->stickyButtons)
      {
        // process each button
        for (int i = 0; i < this->joyMsg.buttons_size(); ++i)
        {
          // change button state only on transition from 0 to 1
          if (this->joyMsg.buttons(i) == 1 &&
              this->lastJoyMsg.buttons(i) == 0)
          {
            this->stickyButtonsJoyMsg.set_buttons(i,
              this->stickyButtonsJoyMsg.buttons(i) ? 0 : 1);
          }
        }

        // update last published message
        this->lastJoyMsg = this->joyMsg;

        // Copy the axis
        this->stickyButtonsJoyMsg.mutable_axes()->CopyFrom(this->joyMsg.axes());

        // Publish the stick buttons message
        this->pub.Publish(this->stickyButtonsJoyMsg);
      }
      else
      {
        // Publish the normal joy message
        this->pub.Publish(this->joyMsg);
      }

      timeoutSet = false;
      accumulating = false;
      accumulate = false;
    }

    // If an axis event occurred, start a timer to combine with other
    // events.
    if (!accumulating && accumulate)
    {
      tv.tv_sec = trunc(this->accumulationInterval);
      tv.tv_usec = (this->accumulationInterval - tv.tv_sec) * 1e6;
      accumulating = true;
      timeoutSet = true;
    }

    // Set a timeout for the signal call at the beginning of this loop.
    if (!timeoutSet)
    {
      tv.tv_sec = trunc(this->interval);
      tv.tv_usec = (this->interval - tv.tv_sec) * 1e6;
      timeoutSet = true;
    }
  }
}
