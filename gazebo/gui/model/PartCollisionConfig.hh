/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _PART_COLLISION_TAB_HH_
#define _PART_COLLISION_TAB_HH_

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

    /// \class CollisionConfigData PartCollisionConfig.hh
    /// \brief A class of widgets used for configuring collision properties.
    class CollisionConfigData
    {
      /// \brief Unique ID of this collision config.
      public: int id;

      /// \brief Name of the collision.
      public: std::string name;

      /// \brief config widget for configuring collision properties.
      public: ConfigWidget *configWidget;

      /// \brief Tree item associated with the configWidget.
      public: QTreeWidgetItem *treeItem;
    };

    /*/// \class CollisionDataWidget PartCollisionConfig.hh
    /// \brief A class of widgets used for configuring collision properties.
    class CollisionDataWidget : public QObject
    {
      Q_OBJECT

      /// \brief Unique id for this widget.
      public: int id;

      /// \brief Label for displaying the collision's name.
      public: QLabel *collisionNameLabel;

      /// \brief Combo box for for specifying the geometry of the collision.
      public: QComboBox *geometryComboBox;

      /// \brief Spin box for configuring size X of a box geom.
      public: QDoubleSpinBox *geomSizeXSpinBox;

      /// \brief Spin box for configuring size Y of a box geom.
      public: QDoubleSpinBox *geomSizeYSpinBox;

      /// \brief Spin box for configuring size Z of a box geom.
      public: QDoubleSpinBox *geomSizeZSpinBox;

      /// \brief Spin box for configuring radius of a cylinder or sphere geom.
      public: QDoubleSpinBox *geomRadiusSpinBox;

      /// \brief Spin box for configuring length of a cylinder geom.
      public: QDoubleSpinBox *geomLengthSpinBox;

      /// \brief Widget for configuring geometry dimensions.
      public: QStackedWidget *geomDimensionWidget;

      /// \brief Label for the geometry length.
      public: QLabel *geomLengthLabel;

      /// \brief Spin box for configuring the X position of the collision.
      public: QDoubleSpinBox *posXSpinBox;

      /// \brief Spin box for configuring the Y position of the part.
      public: QDoubleSpinBox *posYSpinBox;

      /// \brief Spin box for configuring the Z position of the part.
      public: QDoubleSpinBox *posZSpinBox;

      /// \brief Spin box for configuring the roll of the part.
      public: QDoubleSpinBox *rotRSpinBox;

      /// \brief Spin box for configuring the pitch of the part.
      public: QDoubleSpinBox *rotPSpinBox;

      /// \brief Spin box for configuring the yaw of the part.
      public: QDoubleSpinBox *rotYSpinBox;

      /// \brief Spin box for configuring the laser retro value.
      public: QDoubleSpinBox *laserRetroSpinBox;

      /// \brief Spin box for configuring the maximum number of contacts.
      public: QSpinBox *maxContactsSpinBox;

      /// \brief Qt signal emitted when a collision is added.
      private slots: void GeometryChanged(const QString _text);
    };*/

    /// \class PartCollisionConfig PartCollisionConfig.hh
    /// \brief A tab for configuring collision properties of a part.
    class PartCollisionConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartCollisionConfig();

      /// \brief Destructor
      public: ~PartCollisionConfig();

      /// \brief Add a collision widget to the tab.
      /// \param[in] _name Name of collision added.
      public: void AddCollision(const std::string &_name,
          const msgs::Collision *_collisionMsg = NULL);

      /// \brief Update a collision widget from a collision msg.
      /// \param[in] _name Name of collision to be updated.
      /// \param[in] _collisionMsg Msg used to update the collision widget values.
      public: void UpdateCollision(const std::string &_name,
          const msgs::Collision *_collisionMsg);

      /// \brief Reset the collision tab.
      public: void Reset();

      /// \brief Get the number of collisions.
      /// \return Number of collisions.
      public: unsigned int GetCollisionCount() const;

      /// \brief Get the msg containing all collision data.
      /// \param[in] _name Name of collision.
      /// \return Collision msg.
      public: msgs::Collision *GetData(const std::string &_name) const;

      /// \brief Set the geometry size a collision
      /// \param[in] _size Size to set the geometry to.
      public: void SetGeometrySize(const std::string &_name,
          const math::Vector3 &_size);

      /// \brief Map of id to collision config widget.
      private: std::map<int, CollisionConfigData *> configs;

      /// \brief Widget that display collisions' properties.
      private: QTreeWidget *collisionsTreeWidget;

      /// \brief Counter for the number of collisions.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief A map of collision items to their id.
      private: std::map<int, QTreeWidgetItem *> collisionItems;

      /// \brief Qt signal emitted when a collision is removed.
      /// \param[in] _name Name of collision removed.
      Q_SIGNALS: void CollisionRemoved(const std::string &_name);

      /// \brief Qt signal emitted when a collision is added.
      Q_SIGNALS: void CollisionAdded(const std::string &_name);

      /// \brief Qt callback when a collision is to be added.
      /// \param[in] _name Name of collision added.
      private slots: void OnAddCollision();

      /// \brief Qt callback when a collision is to be removed.
      /// \param[in] _item Item to be removed.
      private slots: void OnRemoveCollision(int);

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);

      /*Q_OBJECT

      /// \brief Constructor
      public: PartCollisionConfig();

      /// \brief Destructor
      public: ~PartCollisionConfig();

      /// \brief Add a collision widget to the tab.
      public: void AddCollision();

      /// \brief Reset the collision tab.
      public: void Reset();

      /// \brief Get the number of collisions.
      /// \return Number of collisions.
      public: unsigned int GetCollisionCount() const;

      /// \brief Set the name of the collision.
      /// \param[in] _index Index of collision.
      /// \param[in] _name Name of collision.
      public: void SetName(unsigned int _index, const std::string &_name);

      /// \brief Get the name of the collision.
      /// \param[in] _index Index of collision.
      /// \return Name of collision.
      public: std::string GetName(unsigned int _index) const;

      /// \brief Set the pose of the collision.
      /// \param[in] _index Index of collision.
      /// \param[in] _pose Pose to set the collision to.
      public: void SetPose(unsigned int _index, const math::Pose &_pose);

      /// \brief Get the pose of the collision.
      /// \param[in] _index Index of collision.
      /// \return Pose of the collision.
      public: math::Pose GetPose(unsigned int _index) const;

      /// \brief Set the geometry of the collision.
      /// \param[in] _index Index of collision
      /// \param[in] _geometry Geometry type to set to.
      public: void SetGeometry(unsigned int _index,
          const std::string &_geometry);

      /// \brief Get the geometry of the collision.
      /// \param[in] _index Index of collision
      /// \return Geometry type.
      public: std::string GetGeometry(unsigned int _index) const;

      /// \brief Set the geometry size of the collision.
      /// \param[in] _index Index of collision
      /// \param[in] _length Size to set the geometry to.
      public: void SetGeometrySize(unsigned int _index,
          const math::Vector3 &_size);

      /// \brief Get the geometry length of the collision.
      /// \param[in] _index Index of collision
      /// \return Geometry size.
      public: math::Vector3 GetGeometrySize(unsigned int _index) const;

      /// \brief Set the geometry radius of the collision.
      /// \param[in] _index Index of collision
      /// \param[in] _length Radius to set the geometry to.
      public: void SetGeometryRadius(unsigned int _index,
          double _radius);

      /// \brief Get the geometry radius of the collision.
      /// \param[in] _index Index of collision
      /// \return Geometry radius.
      public: double GetGeometryRadius(unsigned int _index) const;

      /// \brief Set the geometry length of the collision.
      /// \param[in] _index Index of collision
      /// \param[in] _length Length to set the geometry to.
      public: void SetGeometryLength(unsigned int _index,
          double _length);

      /// \brief Get the geometry length of the collision.
      /// \param[in] _index Index of collision
      /// \return Geometry length.
      public: double GetGeometryLength(unsigned int _index) const;

      /// \brief Set the scale of the geometry.
      /// \param[in] _index Index of collision
      /// \param[in] _scale Geometry scale.
      public: void SetGeometryScale(unsigned int _index,
          const math::Vector3 &_scale);

      /// \brief Get the scale of the geometry.
      /// \param[in] _index Index of collision
      /// \return Geometry scale.
      public: math::Vector3 GetGeometryScale(unsigned int _index) const;

      /// \brief Set the laser retro value
      /// \param[in] _index Index of collision
      /// \param[in] _retro Laser retro.
      public: void SetLaserRetro(unsigned int _index,
          double _retro);

      /// \brief Get the laser retro of the collision.
      /// \param[in] _index Index of collision
      /// \return Laser retro.
      public: double GetLaserRetro(unsigned int _index) const;

      /// \brief Get the maximum number of contacts of the collision.
      /// \param[in] _index Index of collision
      /// \return Maximum number of contacts.
      public: double GetMaxContacts(unsigned int _index) const;

      /// \brief Set the maximum number of contacts of the collision.
      /// \param[in] _index Index of collision
      /// \param[in] _maxContacts Maximum number of contacts.
      public: void SetMaxContacts(unsigned int _index,
          double _maxContacts);

      /// \brief List of collision widgets for configuring collision properties.
      private: std::vector<CollisionDataWidget *> dataWidgets;

      /// \brief Widget that display collision' properties.
      private: QTreeWidget *collisionsTreeWidget;

      /// \brief Counter for the number of collision.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief A map of collision items to their id.
      private: std::map<int, QTreeWidgetItem *> collisionItems;

      /// \brief Qt signal emitted when a collision is removed.
      /// \param[in] _name Name of collision removed.
      Q_SIGNALS: void CollisionRemoved(const std::string &_name);

      /// \brief Qt signal emitted when a collision is added.
      Q_SIGNALS: void CollisionAdded();

      /// \brief Qt callback when a collision is to be added.
      private slots: void OnAddCollision();

      /// \brief Qt callback when a collision is to be removed.
      /// \param[in] _item Item to be removed.
      private slots: void OnRemoveCollision(int);

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);*/
    };
  }
}
#endif
