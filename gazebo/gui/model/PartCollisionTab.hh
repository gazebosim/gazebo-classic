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
    /// \addtogroup gazebo_gui
    /// \{

    /// \class CollisionDataWidget PartCollisionTab.hh
    /// \brief A class of widgets used for configuring collision properties.
    class CollisionDataWidget
    {
      /// \brief Unique id for this widget.
      public: int id;

      /// \brief Label for displaying the visual's name.
      public: QLabel *collisionNameLabel;

      /// \brief Combo box for for specifying the geometry of the collision.
      public: QComboBox *geometryComboBox;

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
    };

    /// \class PartCollisionTab PartCollisionTab.hh
    /// \brief A tab for configuring visual properties of a part.
    class PartCollisionTab : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartCollisionTab();

      /// \brief Destructor
      public: ~PartCollisionTab();

      /// \brief Add a collision widget to the tab.
      public: void AddCollision();

      /// \brief Reset the collision tab.
      public: void Reset();

      /// \brief Get the number of collisions.
      /// \return Number of visuals.
      public: unsigned int GetCollisionCount() const;

      /// \brief Set the name of the collision.
      /// \param[in] _index Index of collision.
      /// \param[in] _name Name of collision.
      public: void SetName(unsigned int _index, const std::string &_name);

      /// \brief Get the name of the collision.
      /// \param[in] _index Index of collision.
      /// \return Name of visual.
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
      public: void SetGeometry(unsigned int _index,
          const std::string &_geometry);

      /// \brief Get the geometry of the collision.
      /// \param[in] _index Index of collision
      /// \return Geometry type.
      public: std::string GetGeometry(unsigned int _index) const;

      /// \brief List of visual widgets for configuring collision properties.
      private: std::vector<CollisionDataWidget *> dataWidgets;

      /// \brief Widget that display collision' properties.
      private: QTreeWidget *collisionsTreeWidget;

      /// \brief Counter for the number of collision.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief A map of visual items to their id.
      private: std::map<int, QTreeWidgetItem *> collisionItems;

      /// \brief Qt signal emitted when a visual is removed.
      /// \param[in] _name Name of visual removed.
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
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);
    };
  }
}
#endif
