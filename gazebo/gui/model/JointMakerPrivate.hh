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

#ifndef _GAZEBO_GUI_JOINTMAKER_PRIVATE_HH_
#define _GAZEBO_GUI_JOINTMAKER_PRIVATE_HH_

#include <map>
#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include "gazebo/common/CommonTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/JointMaker.hh"

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class JointCreationDialog;

    /// \brief Private data for the JointMaker class
    class JointMakerPrivate
    {
      /// \brief Type of joint to create
      public: JointMaker::JointType jointType;

      /// \brief Visual that is currently hovered over by the mouse
      public: rendering::VisualPtr hoverVis;

      /// \brief Name of joint that is currently being inspected.
      public: std::string inspectName;

      /// \brief All joints created by joint maker.
      public: std::map<std::string, JointData *> joints;

      /// \brief Joint currently being created.
      public: JointData *newJoint;

      /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief The SDF element pointer to the model that contains the joints.
      public: sdf::ElementPtr modelSDF;

      /// \brief Counter for the number of joints in the model.
      public: int jointCounter;

      /// \brief Qt action for opening the joint inspector.
      public: QAction *inspectAct;

      /// \brief Mutex to protect the list of joints
      public: std::recursive_mutex updateMutex;

      /// \brief A list of selected link visuals.
      public: std::vector<rendering::VisualPtr> selectedJoints;

      /// \brief A list of scoped link names.
      public: std::vector<std::string> scopedLinkedNames;

      /// \brief List of all links currently in the editor. The first string is
      /// the link's fully scoped name and the second is the leaf name.
      public: std::map<std::string, std::string> linkList;

      /// \brief Dialog for creating a new joint.
      public: JointCreationDialog *jointCreationDialog;

      /// \brief Pose of link currently selected to be the parent of the joint
      /// being created, before being selected.
      public: ignition::math::Pose3d parentLinkOriginalPose;

      /// \brief Pose of link currently selected to be the child of the joint
      /// being created, before being selected.
      public: ignition::math::Pose3d childLinkOriginalPose;

      /// \brief Pointer to the user command manager. The pointer's lifetime is
      /// managed by ModelCreator, so we don't need to delete it.
      public: MEUserCmdManager *userCmdManager = nullptr;
    };
  }
}
#endif
