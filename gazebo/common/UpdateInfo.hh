/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _UPDATEINFO_HH_
#define _UPDATEINFO_HH_

#include <string>
#include "gazebo/common/Time.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \class UpdateInfo UpdateInfo.hh common/common.hh
    /// \brief Information for use in an update event.
    class GAZEBO_VISIBLE UpdateInfo
    {
      /// \brief Constructor
      public: UpdateInfo() {}

      /// \brief Destructor
      public: virtual ~UpdateInfo() {}

      /// \brief Name of the world.
      public: std::string worldName = "default";

      /// \brief Current simulation time.
      public: common::Time simTime;

      /// \brief Current real time.
      public: common::Time realTime;
    };
  }
}

#endif
