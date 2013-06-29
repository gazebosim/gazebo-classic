/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _JOINTMAKER_HH_
#define _JOINTMAKER_HH_

#include <string>
#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class JointMaker JointMaker.hh
    /// \brief Joint visualization
    class JointMaker
    {
      /// \enum Joint types
      /// \brief Unique identifiers for joint types that can be created.
      public: enum JointType
      {
        /// \brief none
        JOINT_NONE,
        /// \brief Fixed joint
        JOINT_FIXED,
        /// \brief Slider joint
        JOINT_SLIDER,
        /// \brief Hinge joint
        JOINT_HINGE,
        /// \brief Hinge2 joint
        JOINT_HINGE2,
        /// \brief Screw joint
        JOINT_SCREW,
        /// \brief Universal joint
        JOINT_UNIVERSAL,
        /// \brief Revolute joint
        JOINT_BALL
      };

      /// \brief Constructor
      /// \param[in] _name Name of the joint visual
      /// \param[in] _vis Pointer to the parent visual
      public: JointMaker();

      /// \brief Destructor
      public: virtual ~JointMaker();

      /// \brief Create a joint
      /// \param[_type] Type of joint to be created
      public: void CreateJoint(JointType _type);

      /// \brief Mouse event filter callback when mouse button is pressed in
      /// create joint mode.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is moved in create
      /// joint mode.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseMove(const common::MouseEvent &_event);

/*      /// \brief Set parent of joint.
      /// \param[in] _parent Pointer to parent visual.
      /// \param[in] _offset Offset relative to parent origin where the joint
      /// is to be attached to.
      public: void SetParent(rendering::VisualPtr _parent,
          math::Vector3 _offset = math::Vector3::Zero);

      /// \brief Set child of joint.
      /// \param[in] _child Pointer to child visual.
      /// \param[in] _offset Offset relative to child origin where the joint
      /// is to be attached to.
      public: void SetChild(rendering::VisualPtr _child,
          math::Vector3 _offset = math::Vector3::Zero);*/

      /// \brief Visual line used to represent joint connecting parent and child
      private: rendering::DynamicLines *jointLine;

      /// \brief Keep track of joint type that
      private: JointMaker::JointType createJointType;

//      private: rendering::UserCameraPtr userCamera;

      /// \brief Visual that is currently hovered over by the mouse
      private: rendering::VisualPtr hoverVis;

      /// \brief Currently selected visual
      private: rendering::VisualPtr selectedVis;

      /// \brief Visual that is currently being drawn
      private: rendering::VisualPtr jointVis;

      /// \brief Currently selected visual
      private: rendering::VisualPtr parent;
    };
    /// \}
  }
}
#endif
