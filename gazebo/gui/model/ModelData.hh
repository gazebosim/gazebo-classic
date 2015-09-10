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
#ifndef _GAZEBO_MODEL_DATA_HH_
#define _GAZEBO_MODEL_DATA_HH_

#include <map>
#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/gui/model/LinkInspector.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
  namespace gui
  {
    class LinkInspector;
    class ModelPluginInspector;

    class GZ_GUI_VISIBLE ModelData
    {
      /// \brief Get a template SDF string of a simple model.
      /// \return Template SDF string of a simple model.
      public: static std::string GetTemplateSDFString();

      /// \brief Get the default transparency setting for entities in model
      /// editor.
      public: static double GetEditTransparency();
    };

    /// \brief Helper class to store nested models data.
    class GZ_GUI_VISIBLE NestedModelData
    {
      /// \brief Set the name of the model.
      /// \param[in] _name Name of model.
      public: void SetName(const std::string &_name);

      /// \brief Set the pose of the model.
      /// \param[in] _pose Pose of model.
      public: void SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Get the pose of the nested model.
      /// \return Pose of nested model.
      public: ignition::math::Pose3d Pose() const;

      /// \brief Get the depth of the nested model. The root model has level 1.
      /// \return Depth of nested model. Returns -1 if depth cannot be found.
      public: int Depth() const;

      /// \brief SDF representing the model data.
      public: sdf::ElementPtr modelSDF;

      /// \brief Visual representing this model.
      public: rendering::VisualPtr modelVisual;
    };

    /// \class LinkData LinkData.hh
    /// \brief Helper class to store link data
    class GZ_GUI_VISIBLE LinkData : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: LinkData();

      /// \brief Destructor
      public: ~LinkData();

      /// \brief Get the name of the link.
      /// \return Name of link.
      public: std::string GetName() const;

      /// \brief Set the name of the link.
      /// \param[in] _name Name of link.
      public: void SetName(const std::string &_name);

      /// \brief Get the pose of the link.
      /// \return Pose of link.
      public: ignition::math::Pose3d Pose() const;

      /// \brief Set the pose of the link.
      /// \param[in] _pose Pose of link.
      public: void SetPose(const ignition::math::Pose3d &_pose3d);

      /// \brief Load the link with data from SDF.
      /// \param[in] _sdf Link SDF element.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief Get the scale of the link.
      /// \return Scale of link.
      public: ignition::math::Vector3d Scale() const;

      /// \brief Set the scale of the link.
      /// \param[in] _scale Scale of link.
      public: void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Add a visual to the link.
      /// \param[in] _visual Visual to be added.
      public: void AddVisual(rendering::VisualPtr _visual);

      /// \brief Add a collision to the link.
      /// \param[in] _collisionVis Visual representing the collision.
      public: void AddCollision(rendering::VisualPtr _collisionVis);

      /// \brief Update the inspector widget if necessary.
      public: void UpdateConfig();

      /// \brief Clone the link data.
      /// \param[in] _newName Name to give to the cloned link.
      /// \return A clone of this link data.
      public: LinkData *Clone(const std::string &_newName);

      /// \brief Update callback on PreRender.
      private: void Update();

      /// \brief Apply inspector configurations.
      /// \return True if successful.
      private: bool Apply();

      /// \brief Qt Callback when link inspector configurations are to be
      /// applied and inspector should be closed.
      private slots: void OnAccept();

      /// \brief Qt Callback when link inspector configurations are to be
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

      /// \brief Mutex to protect visual and collision update messages.
      private: boost::recursive_mutex *updateMutex;

      /// \brief SDF representing the link data.
      public: sdf::ElementPtr linkSDF;

      /// \brief mass.
      private: double mass;

      /// \brief Inertia ixx.
      private: double inertiaIxx;

      /// \brief Inertia iyy.
      private: double inertiaIyy;

      /// \brief Inertia izz.
      private: double inertiaIzz;

      /// \brief Scale of link.
      public: ignition::math::Vector3d scale;

      /// \brief Visual representing this link.
      public: rendering::VisualPtr linkVisual;

      /// \brief Visuals of the link.
      public: std::map<rendering::VisualPtr, msgs::Visual> visuals;

      /// \brief Msgs for updating visuals.
      public: std::vector<msgs::Visual *> visualUpdateMsgs;

      /// \brief Msgs for updating collision visuals.
      public: std::vector<msgs::Collision *> collisionUpdateMsgs;

      /// \brief Collisions of the link.
      public: std::map<rendering::VisualPtr, msgs::Collision> collisions;

      /// \brief Inspector for configuring link properties.
      public: LinkInspector *inspector;
    };

    /// \brief Helper class to store model plugin data
    class GZ_GUI_VISIBLE ModelPluginData : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: ModelPluginData();

      /// \brief Destructor
      public: ~ModelPluginData();

      /// \brief Load data from the plugin SDF
      /// \param[in] _pluginElem SDF element.
      public: void Load(sdf::ElementPtr _pluginElem);

      /// \brief Inspector for configuring model plugin properties.
      public: ModelPluginInspector *inspector;

      /// \brief SDF representing the model plugin data.
      public: sdf::ElementPtr modelPluginSDF;
    };
  }
}

#endif
