/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _MODEL_CREATOR_HH_
#define _MODEL_CREATOR_HH_

#include <list>
#include <string>
#include <vector>

#include <boost/unordered/unordered_map.hpp>

#include <sdf/sdf.hh>

#include "gazebo/common/KeyEvent.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/JointMaker.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace gui
  {
    class PartData;
    class ModelCreatorPrivate;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ModelCreator ModelCreator.hh
    /// \brief Create and manage 3D visuals of a model with parts and joints.
    class ModelCreator : public QObject
    {
      Q_OBJECT

      /// \enum Joint types
      /// \brief Unique identifiers for joint types that can be created.
      public: enum PartType
      {
        /// \brief none
        PART_NONE,
        /// \brief Box
        PART_BOX,
        /// \brief Sphere
        PART_SPHERE,
        /// \brief Cylinder
        PART_CYLINDER,
        /// \brief Custom
        PART_CUSTOM
      };

      /// \brief Constructor
      public: ModelCreator();

      /// \brief Destructor
      public: virtual ~ModelCreator();

      /// \brief Set the name of the model.
      /// \param[in] _modelName Name of the model to set to.
      public: void SetModelName(const std::string &_modelName);

      /// \brief Get the name of the model.
      /// \return Name of model.
      public: std::string GetModelName() const;

      /// \brief Finish the model and create the entity on the gzserver.
       public: void FinishModel();

      /// \brief Add a box to the model.
      /// \param[in] _size Size of the box.
      /// \param[in] _pose Pose of the box.
      /// \return Name of the box that has been added.
      public: std::string AddBox(
          const math::Vector3 &_size = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a sphere to the model.
      /// \param[in] _radius Radius of the sphere.
      /// \param[in] _pose Pose of the sphere.
      /// \return Name of the sphere that has been added.
      public: std::string AddSphere(double _radius = 0.5,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a cylinder to the model.
      /// \param[in] _radius Radius of the cylinder.
      /// \param[in] _length Length of the cylinder.
      /// \param[in] _pose Pose of the cylinder.
      /// \return Name of the cylinder that has been added.
      public: std::string AddCylinder(double _radius = 0.5,
          double _length = 1.0, const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a custom part to the model
      /// \param[in] _name Name of the custom part.
      /// \param[in] _scale Scale of the custom part.
      /// \param[in] _pose Pose of the custom part.
      /// \return Name of the custom that has been added.
      public: std::string AddCustom(const std::string &_name,
          const math::Vector3 &_scale = math::Vector3::One,
          const math::Pose &_pose = math::Pose::Zero);

      /// \brief Add a joint to the model.
      /// \param[in] _type Type of joint to add.
      /// \return Name of the joint that has been added.
      public: void AddJoint(JointMaker::JointType _type);

      /// \brief Remove a part from the model.
      /// \param[in] _partName Name of the part to remove
      public: void RemovePart(const std::string &_partName);

      /// \brief Set the model to be static
      /// \param[in] _static True to make the model static.
      public: void SetStatic(bool _static);

      /// \brief Set the model to allow auto disable at rest.
      /// \param[in] _auto True to allow the model to auto disable.
      public: void SetAutoDisable(bool _auto);

      /// \brief Save model to SDF format.
      /// \param[in] _savePath Path to save the SDF to.
      public: void SaveToSDF(const std::string &_savePath);

      /// \brief Reset the model creator and the SDF.
      public: void Reset();

      /// \brief Stop the process of adding a part or joint to the model.
      public: void Stop();

      /// \brief Get joint maker
      /// \return Joint maker
      public: JointMaker *GetJointMaker() const;

      /// \brief Add a part to the model
      /// \param[in] _type Type of part to be added
      public: void AddPart(PartType _type);

      /// \brief Generate the SDF from model part and joint visuals.
      public: void GenerateSDF();

      /// \brief Mouse event filter callback when mouse is moved.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseMovePart(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is released.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseReleasePart(const common::MouseEvent &_event);

      /// \brief Mouse event filter callback when mouse is double clicked.
      /// \param[in] _event The mouse event.
      /// \return True if the event was handled
      private: bool OnMouseDoubleClickPart(const common::MouseEvent &_event);

      /// \brief Key event filter callback when key is pressed.
      /// \param[in] _event The key event.
      /// \return True if the event was handled
      private: bool OnKeyPressPart(const common::KeyEvent &_event);

      /// \brief Create part with default properties from a visual
      /// \param[in] _visual Visual used to create the part.
      private: void CreatePart(const rendering::VisualPtr &_visual);

      // Documentation inherited
      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      /// \brief Create an empty model.
      /// \return Name of the model created.
      private: std::string CreateModel();

      /// \brief Get a template SDF string of a simple model.
      /// \return Template SDF string of a simple model.
      private: std::string GetTemplateSDFString();

      /// \brief Qt callback when a delete signal has been emitted.
      /// \param[in] _name Name of the part or model to delete.
      private slots: void OnDelete(const std::string &_name="");

      /// \brief Qt signal when the a part has been added.
      Q_SIGNALS: void PartAdded();

      /// \internal
      /// \brief Pointer to private data.
      private: ModelCreatorPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
