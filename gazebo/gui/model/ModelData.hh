/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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
#ifndef _MODEL_DATA_HH_
#define _MODEL_DATA_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/gui/model/LinkInspector.hh"

namespace boost
{
  class recursive_mutex;
}

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace gui
  {
    class LinkInspector;

    class ModelData
    {
      /// \brief Get a template SDF string of a simple model.
      /// \return Template SDF string of a simple model.
      public: static std::string GetTemplateSDFString();

      /// \brief Get the default transparency setting for entities in model
      /// editor.
      public: static double GetEditTransparency();
    };

    /// \class PartData PartData.hh
    /// \brief Helper class to store part data
    class PartData : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartData();

      /// \brief Destructor
      public: ~PartData();

      /// \brief Get the name of the part.
      /// \return Name of part.
      public: std::string GetName() const;

      /// \brief Set the name of the part.
      /// \param[in] _name Name of part.
      public: void SetName(const std::string &_name);

      /// \brief Get the pose of the part.
      /// \return Pose of part.
      public: math::Pose GetPose() const;

      /// \brief Set the pose of the part.
      /// \param[in] _pose Pose of part.
      public: void SetPose(const math::Pose &_pose);

      /// \brief TODO
      public: void SetMass(double _mass);

      /// \brief TODO
      public: void SetInertialPose(const math::Pose &_pose);

      /// TODO
      public: void SetInertiaMatrix(double _ixx, double _ixy, double _ixz,
          double _iyy, double _iyz, double _izz);

      /// \brief Set the scale of the part.
      /// \param[in] _scale Scale of part.
      //public: void SetScale(const math::Vector3 &_scale);

      /// \brief Add a visual to the part.
      /// \param[in] _visual Visual to be added.
      public: void AddVisual(rendering::VisualPtr _visual);

      /// \brief Add a collision to the part.
      /// \param[in] _collision Visual representing the collision.
      public: void AddCollision(rendering::VisualPtr _collisionVis);

      /// \brief Update the inspector widget if necessary.
      public: void UpdateConfig();

      /// \brief Update callback on PreRender.
      private: void Update();

      /// \brief Qt Callback when part inspector configurations are to be
      /// applied.
      private slots: void OnApply();

      /// \brief Qt callback when a new visual is to be added.
      /// \param[in] _name Name of visual.
      private slots: void OnAddVisual(const std::string &_name);

      /// \brief Qt callback when a new collision is to be added.
      /// \param[in] _name Name of collision.
      private slots: void OnAddCollision(const std::string &_name);

      /// \brief Qt callback when a visual is to be removed.
      /// \param[in] _name Name of visual.
      private slots: void OnRemoveVisual(const std::string &_name);

      /// \brief Qt callback when a collision is to be removed.
      /// \param[in] _name Name of collision.
      private slots: void OnRemoveCollision(const std::string &_name);

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Mutex to protect the list of joints
      private: boost::recursive_mutex *updateMutex;

      /// \brief SDF representing the part data.
      public: sdf::ElementPtr partSDF;

      /// \brief Scale of part.
      public: math::Vector3 scale;

      /// \brief Visual representing this part.
      public: rendering::VisualPtr partVisual;

      /// \brief Visuals of the part.
      public: std::map<rendering::VisualPtr, msgs::Visual> visuals;

      /// \brief Msgs for updating visuals.
      public: std::vector<msgs::Visual *> visualUpdateMsgs;

      /// \brief Msgs for updating collision visuals.
      public: std::vector<msgs::Collision *> collisionUpdateMsgs;

      /// \brief Collisions of the part.
      public: std::map<rendering::VisualPtr, msgs::Collision> collisions;

      /// \brief Inspector for configuring part properties.
      public: LinkInspector *inspector;
    };
  }
}

#endif
