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

#ifndef _COLLISION_CONFIG_HH_
#define _COLLISION_CONFIG_HH_

#include <map>
#include <string>

#include "gazebo/math/Pose.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/model/ModelData.hh"

namespace gazebo
{
  namespace gui
  {
    class ConfigWidget;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class CollisionConfigData CollisionConfig.hh
    /// \brief A class of widgets used for configuring collision properties.
    class CollisionConfigData : public QWidget
    {
      Q_OBJECT

      /// \brief Restore the widget's data to how it was when first opened.
      public slots: void RestoreOriginalData();

      /// \brief Qt callback when this item's button has been pressed.
      /// \param[in] _checked Whether it was checked or unchecked.
      private slots: void OnToggleItem(bool _checked);

      /// \brief Callback for geometry changes.
      private slots: void OnGeometryChanged();

      /// \brief Signal to indicate a collision change.
      /// \param[in] _name Name of the collision changed.
      /// \param[in] _type Type of change ("geometry", etc).
      Q_SIGNALS: void CollisionChanged(const std::string &_name,
          const std::string &_type);

      /// \brief Unique ID of this collision config.
      public: int id;

      /// \brief Name of the collision.
      public: std::string name;

      /// \brief Config widget for configuring collision properties.
      public: ConfigWidget *configWidget;

      /// \brief Widget associated with this data.
      public: QWidget *widget;

      /// \brief Message containing the data which was in the widget when first
      /// open.
      public: msgs::Collision originalDataMsg;
    };

    /// \class CollisionConfig CollisionConfig.hh
    /// \brief A tab for configuring collision properties of a link.
    class GZ_GUI_VISIBLE CollisionConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: CollisionConfig();

      /// \brief Destructor
      public: ~CollisionConfig();

      /// \brief Add a collision widget to the tab.
      /// \param[in] _name Name of collision added.
      /// \param[in] _collisionMsg Msg containing information of the collision
      /// to be added.
      public: void AddCollision(const std::string &_name,
          const msgs::Collision *_collisionMsg = NULL);

      /// \brief Update a collision widget from a collision msg.
      /// \param[in] _name Name of collision to be updated.
      /// \param[in] _collisionMsg Msg used to update the collision widget
      /// values.
      public: void UpdateCollision(const std::string &_name,
          ConstCollisionPtr _collisionMsg);

      /// \brief Reset the collision tab.
      public: void Reset();

      /// \brief Get the number of collisions.
      /// \return Number of collisions.
      public: unsigned int GetCollisionCount() const;

      /// \brief Get the msg containing all collision data.
      /// \param[in] _name Name of collision.
      /// \return Collision msg.
      public: msgs::Collision *GetData(const std::string &_name) const;

      /// \brief Set the geometry data of a collision
      /// \param[in] _name Name of collision.
      /// \param[in] _size Size to set the geometry to.
      /// \param[in] _uri URI of the geometry.
      public: void SetGeometry(const std::string &_name,
          const ignition::math::Vector3d &_size, const std::string &_uri = "");

      /// \brief Get the geometry data of a collision
      /// \param[in] _name Name of collision.
      /// \param[in] _size Size of the geometry.
      /// \param[in] _uri URI of the geometry.
      public: void Geometry(const std::string &_name,
          ignition::math::Vector3d &_size, std::string &_uri) const;

      /// \brief Get collision configuration data
      public: std::map<int, const CollisionConfigData *> GetConfigData() const;

      /// \brief Get collision configuration data by collision name.
      /// \param[in] _name Name of the collision.
      /// \return The matching configuration or NULL if not found.
      public: const CollisionConfigData *GetConfigData(
          const std::string &_name) const;

      /// \brief Qt signal emitted when a collision is removed.
      /// \param[in] _name Name of collision removed.
      Q_SIGNALS: void CollisionRemoved(const std::string &_name);

      /// \brief Qt signal emitted when a collision is added.
      /// \param[in] _name Name of collision added.
      Q_SIGNALS: void CollisionAdded(const std::string &_name);

      /// \brief Qt signal emitted to indicate that changes should be applied.
      Q_SIGNALS: void Applied();

      /// \brief Qt callback when a collision is to be added.
      private slots: void OnAddCollision();

      /// \brief Qt callback when a collision is to be removed.
      /// \param[in] _id Id of item to be removed.
      private slots: void OnRemoveCollision(int _id);

/// \brief Qt callback when a pose value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New value.
      private slots: void OnPoseChanged(const QString &_name,
          const ignition::math::Pose3d &_value);

/// \brief Qt callback when a geometry value has changed.
      /// \param[in] _name of widget in the config widget that emitted the
      /// signal.
      /// \param[in] _value New geometry value.
      /// \param[in] _dimensions New dimensions.
      /// \param[in] _uri New uri, for meshes.
      private slots: void OnGeometryChanged(const std::string &_name,
          const std::string &_value,
          const ignition::math::Vector3d &_dimensions,
          const std::string &_uri);

      /// \brief Callback for handling collision changes.
      /// \param[in] _name Name of collision changed.
      /// \param[in] _type Type of change.
      private slots: void OnCollisionChanged(
          const std::string &_name, const std::string &_type);

      /// \brief Signal to indicate a collision change.
      /// \param[in] _name Name of collision changed.
      /// \param[in] _type Type of change.
      Q_SIGNALS: void CollisionChanged(
          const std::string &_name, const std::string &_type);

      /// \brief Map of id to collision config widget.
      private: std::map<int, CollisionConfigData *> configs;

      /// \brief Counter for the number of collisions.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief A map of collision items to their id.
      private: std::map<int, QTreeWidgetItem *> collisionItems;

      /// \brief Layout which holds all collision items.
      private: QVBoxLayout *listLayout;
    };
    /// \}
  }
}
#endif
