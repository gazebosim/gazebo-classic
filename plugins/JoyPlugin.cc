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
  close(this->joyFd);
}

/////////////////////////////////////////////////
void JoyPlugin::Load(physics::WorldPtr /*_world*/, sdf::ElementPtr /*_sdf*/)
{
  std::string deviceFilename = "/dev/input/js0";

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

  float deadzone = ignition::math::clamp(0.05, 0.0f, 0.9f);
  this->unscaledDeadzone = 32767f * deadzone;
  this->axisScale = -1f / (1f - deadzone) / 32767f;

  // Create the publisher of joy messages
  this->pub = this->node.Advertise<ignition::msgs::Joystick>("/joy");

  // Connect to the world update
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&JoyPlugin::Update, this));
}

/////////////////////////////////////////////////
void JoyPlugin::Update()
{
  // bool publishNow = false;
  // bool publishSoon = false;

  fd_set set;
  struct timeval tv;

  FD_ZERO(&set);
  FD_SET(this->joyFd, &set);

  int selectOut = select(this->joyFd+1, &set, NULL, NULL, &tv);

  if (selectOut == -1)
  {
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    gzdbg << "Joystick might be closed\n";
    return;
  }

  js_event event;

  bool tvSet = false;

  if (FD_ISSET(this->joyFd, &set))
  {
    if (read(this->joyFd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
    {
      gzdbg << "Joystick read failed, might be closed\n";
      return;
    }

    // joyMsg.mutable_header()->set_stamp();
    // this->count++;

    switch(event.type)
    {
      case JS_EVENT_BUTTON:
      case JS_EVENT_BUTTON | JS_EVENT_INIT:
        {
          std::cerr << "Button Event\n";
          /*
          // Update number of buttons
          if(event.number >= this->joyMsg.buttons.size())
          {
            int oldSize = this->joyMsg.buttons.size();
            this->joyMsg.buttons.resize(event.number+1);

            // last_published_joy_msg.buttons.resize(event.number+1);
            // sticky_buttons_joy_msg.buttons.resize(event.number+1);

            for(unsigned int i = oldSize; i< this->joyMsg.buttons.size(); ++i)
            {
              this->joyMsg.buttons[i] = 0.0;
              // last_published_joy_msg.buttons[i] = 0.0;
              // sticky_buttons_joy_msg.buttons[i] = 0.0;
            }
          }

          // Update the button
          this->joyMsg.buttons[event.number] = (event.value ? 1 : 0);

          // For initial events, wait a bit before sending to try to catch
          // all the initial events.
          if (!(event.type & JS_EVENT_INIT))
            publishNow = true;
          else
            publishSoon = true;

            */
          break;
        }

      case JS_EVENT_AXIS:
      case JS_EVENT_AXIS | JS_EVENT_INIT:
        {
          std::cout << "axis event[" << event.value << "]\n";
          float val = event.value;

          if (event.number >= this->joyMsg.axes_size())
          {
            int oldSize = this->joyMsg.axes_size();
            // this->joyMsg.axes.resize(event.number+1);
            // this->lastJoyMsg.axes.resize(event.number+1);
            // this->stickyButtonsJoyMsg.axes.resize(event.number+1);

            for (unsigned int i = oldSize; i <= event.number; ++i)
            {
              this->joyMsg.set_axes(i, 0.0);
              this->lastJoyMsg.set_axes(i, 0.0);
              this->stickyButtonsJoyMsg.set_axes(i, 0.0);
            }
          }

          // Smooth the deadzone
          if (val < -this->unscaledDeadzone)
            val += this->unscaledDeadzone;
          else if (val > this->unscaledDeadzone)
            val -= this->unscaledDeadzone;
          else
            val = 0f;

          this->joyMsg.set_axes(event.number, val * this->axisScale);

          // Will wait a bit before sending to try to combine events.
          publishSoon = true;
          break;
        }
      default:
        {
          gzwarn << "Unknown event type: time[" << event.time << "] "
            << "value[" << event.value << "] "
            << "type[" << event.type << "h] "
            << "number["<< event.number << "]" << std::endl;
          break;
        }
    // End case statement
    }
  }
  // Assume that the timer has expired.
  /*else if (tvSet)
    publishNow = true;
    */


  /*if (publishNow)
  {
    if (stickyButtons)
    {
      // process each button
      for (size_t i = 0; i < joy_msg.buttons.size(); ++i)
      {
        // change button state only on transition from 0 to 1
        if (this->joyMsg.buttons[i] == 1 &&
            this->lastJoyMsg.buttons[i] == 0)
        {
          this->stickyButtonsJoyMsg.buttons[i] =
            this->stickyButtonsJoyMsg.buttons[i] ? 0 : 1;
        }
        else
        {
          // do not change the message sate
        }
      }

      // update last published message
      this->lastJoyMsg = this->joyMsg;

      // fill rest of sticky_buttons_joy_msg (time stamps, axes, etc)
      this->stickyButtonsJoyMsg.header.stamp.nsec = joy_msg.header.stamp.nsec;
      this->stickyButtonsJoyMsg.header.stamp.sec  = joy_msg.header.stamp.sec;
      this->stickyButtonsJoyMsg.header.frame_id   = joy_msg.header.frame_id;

      for(size_t i = 0; i < this->joyMsg.axes.size(); ++i)
      {
        this->stickyButtonsJoyMsg.axes[i] = this->joyMsg.axes[i];
      }

      this->pub.Publish(this->stickyButtonsJoyMsg);
    }
    else
    {
      this->pub.Publish(this->joyMsg);
    }

    publishNow = false;
    tvSet = false;
    publicationPending = false;
    publishSoon = false;
  }

  // If an axis event occurred, start a timer to combine with other
  // events.
  if (!publicationPending && publishSoon)
  {
    tv.tv_sec = trunc(coalesce_interval_);
    tv.tv_usec = (coalesce_interval_ - tv.tv_sec) * 1e6;
    publication_pending = true;
    tvSet = true;
  }

  // If nothing is going on, start a timer to do autorepeat.
  if (!tvSet && autorepeat_rate_ > 0)
  {
    tv.tv_sec = trunc(autorepeat_interval);
    tv.tv_usec = (autorepeat_interval - tv.tv_sec) * 1e6;
    tvSet = true;
  }

  if (!tvSet)
  {
    tv.tv_sec = 1;
    tv.tv_usec = 0;
  }
  */
}
