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
#include <boost/unordered/unordered_map.hpp>

#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class JointData;
    class JointInspector;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class JointMaker JointMaker.hh
    /// \brief Joint visualization
    class JointMaker : public QObject
    {
      Q_OBJECT

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

      /// \brief Update callback on PreRender.
      public: void Update();

      /// \brief Mouse event filter callback when mouse button is pressed .
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is moved.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is double clicked.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseDoubleClick(const common::MouseEvent &_event);

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

      /// \brief Helper method to create hotspot visual for mouse interaction.
      private: void CreateHotSpot();

      /// \brief Qt signal when the joint creation process has ended.
      Q_SIGNALS: void JointCreated();

      /// \brief Qt Callback when joint inspector configurations are to be
      /// applied.
      private slots: void OnApply();

      /// \brief Type of joint to create
      private: JointMaker::JointType jointType;

//      private: rendering::UserCameraPtr userCamera;

      /// \brief Visual that is currently hovered over by the mouse
      private: rendering::VisualPtr hoverVis;

      /// \brief Currently selected visual
      private: rendering::VisualPtr selectedVis;

      /// \brief All joints created by joint maker.
      private: boost::unordered_map<std::string, JointData *> joints;

      /// \brief Joint currently being created.
      private: JointData *mouseJoint;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Flag set to true when a joint has been connected.
      private: bool newJointCreated;

      private: boost::unordered_map<JointMaker::JointType, std::string>
          jointMaterials;

      /// \brief Inspector for configuring joint properties.
      private: JointInspector *inspector;
    };
    /// \}

    /// \class JointData JointData.hh
    /// \brief Helper class to store joint data
    class JointData
    {
      /// \brieft Visual of the dynamic line
      public: rendering::VisualPtr visual;

      /// \brieft Visual of the hotspot
      public: rendering::VisualPtr hotspot;

      /// \brief Parent visual the joint is connected to.
      public: rendering::VisualPtr parent;

      /// \brief Child visual the joint is connected to.
      public: rendering::VisualPtr child;

      /// \brief Visual line used to represent joint connecting parent and child
      public: rendering::DynamicLines *line;

      /// \brief Type of joint
      public: JointMaker::JointType type;

      /// \brief True if the joint visual needs update.
      public: bool dirty;
    };
  }
}
#endif
