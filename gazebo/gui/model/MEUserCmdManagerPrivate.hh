/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_GUI_MODEL_MEUSERCMDMANAGER_PRIVATE_HH_
#define _GAZEBO_GUI_MODEL_MEUSERCMDMANAGER_PRIVATE_HH_

#include <memory>
#include <sdf/sdf.hh>

#include "gazebo/gui/model/MEUserCmdManager.hh"
#include "gazebo/gui/model/ModelEditorTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the MEUserCmd class
    class MEUserCmdPrivate
    {
      /// \brief Unique ID identifying this command in the server.
      public: unsigned int id;

      /// \brief Description for the command.
      public: std::string description;

      /// \brief Type of command, such as MOVING or DELETING.
      public: MEUserCmd::CmdType type;

      /// \brief SDF element with information about the entity.
      public: sdf::ElementPtr sdf;

      /// \brief Fully scoped name of the entity involved in the command.
      public: std::string scopedName;

      /// \brief Pose before the command (to be used by undo).
      public: ignition::math::Pose3d poseBefore;

      /// \brief Pose after the command (to be used by redo).
      public: ignition::math::Pose3d poseAfter;

      /// \brief Scale before the command (to be used by undo).
      public: ignition::math::Vector3d scaleBefore;

      /// \brief Scale after the command (to be used by redo).
      public: ignition::math::Vector3d scaleAfter;

      /// \brief If the command is related to a joint, this is its unique Id.
      /// It's different from the scopedName and we need both.
      public: std::string jointId;
    };

    /// \internal
    /// \brief Private data for the MEUserCmdManager class
    class MEUserCmdManagerPrivate
    {
      /// \brief Counter to give commands unique ids.
      public: unsigned int idCounter;

      /// \brief List of commands which can be undone.
      public: std::vector<MEUserCmdPtr> undoCmds;

      /// \brief List of commands which can be redone.
      public: std::vector<MEUserCmdPtr> redoCmds;
    };
  }
}
#endif

