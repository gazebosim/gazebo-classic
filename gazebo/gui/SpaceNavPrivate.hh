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
#ifndef _GAZEBO_SPACENAV_PRIVATE_HH_
#define _GAZEBO_SPACENAV_PRIVATE_HH_

#define SCALE 512.0

#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the SpaceNav class.
    class SpaceNavPrivate
    {
      /// \brief Constructor
      public: SpaceNavPrivate()
      {
        this->pollThread = NULL;
        this->stop = false;
      }

      /// \brief Destructor
      public: virtual ~SpaceNavPrivate()
      {
        this->stop = true;
        if (this->pollThread)
          this->pollThread->join();
        delete this->pollThread;
        this->pollThread = NULL;
      }

      /// \brief Additional thread
      public: boost::thread *pollThread;

      /// \brief Use to stop the additional thread that the plugin uses.
      public: bool stop;

      /// \brief Gazebo communication node pointer.
      public: transport::NodePtr node;

      /// \brief Publisher pointer used to publish the messages.
      public: transport::PublisherPtr joyPub;

      /// \brief Translation values below which joystick values return zero.
      public: math::Vector3 deadbandTrans;

      /// \brief Rotation values below which joystick values return zero.
      public: math::Vector3 deadbandRot;
    };
  }
}
#endif
