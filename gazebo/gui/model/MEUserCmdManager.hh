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

#ifndef _GAZEBO_GUI_MODEL_MEUSERCMDMANAGER_HH_
#define _GAZEBO_GUI_MODEL_MEUSERCMDMANAGER_HH_

#include <memory>
#include <string>
#include <sdf/sdf.hh>

#include "gazebo/common/SingletonT.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/UserCmdHistory.hh"
#include "gazebo/gui/model/ModelEditorTypes.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class UserCmdHistory;

    // Forward declare private data.
    class MEUserCmdPrivate;
    class MEUserCmdManagerPrivate;

    /// \brief Class which represents a user command, which can be "undone"
    /// and "redone".
    class GZ_GUI_VISIBLE MEUserCmd
    {
      /// \enum CmdType
      /// \brief Types of user commands.
      public: enum CmdType
      {
        /// \brief Insert a link.
        INSERTING_LINK,

        /// \brief Delete a link.
        DELETING_LINK,

        /// \brief Insert a nested model.
        INSERTING_NESTED_MODEL,

        /// \brief Delete a nested model.
        DELETING_NESTED_MODEL,

        /// \brief Insert a joint.
        INSERTING_JOINT,

        /// \brief Delete a joint.
        DELETING_JOINT,

        /// \brief Insert a model plugin.
        INSERTING_MODEL_PLUGIN,

        /// \brief Delete a model plugin.
        DELETING_MODEL_PLUGIN,

        /// \brief Move a link.
        MOVING_LINK,

        /// \brief Scale a link.
        SCALING_LINK,

        /// \brief Move a nested model.
        MOVING_NESTED_MODEL
      };

      /// \brief Constructor
      /// \param[in] _id Unique ID for this command
      /// \param[in] _description Description for the command, such as
      /// "Rotate box", "Delete sphere", etc.
      /// \param[in] _type Type of command, such as MOVING, DELETING, etc.
      public: MEUserCmd(unsigned int _id, const std::string &_description,
          MEUserCmd::CmdType _type);

      /// \brief Destructor
      public: virtual ~MEUserCmd();

      /// \brief Undo this command.
      public: virtual void Undo();

      /// \brief Redo this command.
      public: virtual void Redo();

      /// \brief Return this command's unique ID.
      /// \return Unique ID
      public: unsigned int Id() const;

      /// \brief Return this command's description.
      /// \return Description
      public: std::string Description() const;

      /// \brief Return this command's type.
      /// \return Command type
      public: void SetSDF(sdf::ElementPtr _sdf);
      public: void SetScopedName(const std::string &_name);
      public: void SetPoseChange(const ignition::math::Pose3d &_before,
          const ignition::math::Pose3d &_after);
      public: void SetScaleChange(const ignition::math::Vector3d &_before,
          const ignition::math::Vector3d &_after);
      public: void SetJointId(const std::string &_id);

      /// \internal
      /// \brief Pointer to private data.
      protected: std::shared_ptr<MEUserCmdPrivate> dataPtr;
    };

    /// \brief Class which manages user commands in the model editor. It
    /// combines features of gui::UserCmdHistory and physics::UserCmdManager.
    class GZ_GUI_VISIBLE MEUserCmdManager : public UserCmdHistory,
        public SingletonT<MEUserCmdManager>
    {
      Q_OBJECT

      /// \brief Constructor
      public: MEUserCmdManager();

      /// \brief Destructor
      public: virtual ~MEUserCmdManager();

      /// \brief Initialize the model alignment tool.
      public: void Init();

      /// \brief Clear the model alignment tool. This explicity cleans up the
      /// internal state of the singleton and prepares it for exit.
      public: void Clear();

      /// \brief Reset commands.
      public: void Reset();

      /// \brief Register that a new command has been executed by the user.
      public: MEUserCmdPtr NewCmd(const std::string &_description,
                 const MEUserCmd::CmdType _type);

      /// \brief Qt call back when an undo history action is triggered.
      /// It publishes an undo request message.
      private slots: void OnUndoCommand(QAction *_action);

      /// \brief Qt call back when the undo history button is pressed.
      /// It opens the undo history menu.
      private slots: void OnUndoCmdHistory();

      /// \brief Qt call back when a redo history action is triggered.
      /// It publishes a redo request message.
      private slots: void OnRedoCommand(QAction *_action);

      /// \brief Qt call back when the redo history button is pressed.
      /// It opens the redo history menu.
      private slots: void OnRedoCmdHistory();

      /// \brief Updates the widgets according to the user command stats
      /// message.
      private slots: void UpdateStats();

      /// \brief This is a singleton class.
      private: friend class SingletonT<MEUserCmdManager>;

      /// \internal
      /// \brief Pointer to private data.
      private: std::shared_ptr<MEUserCmdManagerPrivate> dataPtr;
    };
  }
}
#endif

