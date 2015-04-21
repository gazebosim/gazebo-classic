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
#ifndef _GZ_JOY_HH_
#define _GZ_JOY_HH_

#include "gz.hh"

namespace gazebo
{
  namespace util
  {
    class Joystick;
  }

  /// \brief Joystick command
  class JoyCommand : public Command
  {
    /// \brief Constructor
    public: JoyCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    // Documentation inherited
    protected: virtual bool TransportRequired();

    /// \brief Node pointer.
    private: transport::NodePtr node;

    private: transport::PublisherPtr joyPub;

    private: util::Joystick *joy;
  };
}
#endif
