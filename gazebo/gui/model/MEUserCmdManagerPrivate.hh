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
#ifndef _GAZEBO_GUI_MODEL_MEUSERCMD_PRIVATE_HH_
#define _GAZEBO_GUI_MODEL_MEUSERCMD_PRIVATE_HH_

#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/MEUserCmdManager.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the MEUserCmdManager class
    class MEUserCmdPrivate
    {
      /// \brief Unique ID identifying this command in the server.
      public: unsigned int id;

      /// \brief Description for the command.
      public: std::string description;

      /// \brief Type of command, such as MOVING or DELETING.
      public: MEUserCmd::CmdType type;

      /// \brief
      public: sdf::ElementPtr sdf;
      public: std::string scopedName;
      public: std::string jointId;
    };

    /// \internal
    /// \brief Private data for the MEUserCmdManager class
    class MEUserCmdManagerPrivate
    {
      /// \brief Counter to give commands unique ids.
      public: unsigned int idCounter;

      /// \brief list of commands which can be undone.
      public: std::vector<MEUserCmd *> undoCmds;

      /// \brief list of commands which can be redone.
      public: std::vector<MEUserCmd *> redoCmds;
    };
  }
}
#endif

