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

#ifndef _PART_VISUAL_TAB_HH_
#define _PART_VISUAL_TAB_HH_

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

    /// \class VisualDataWidget PartVisualTab.hh
    /// \brief A class of widgets used for configuring visual properties.
    class VisualDataWidget
    {
      /// \brief Unique id for this widget.
      public: int id;

      /// \brief Label for displaying the visual's name.
      public: QLabel *visualNameLabel;

      /// \brief Combo box for for specifying the geometry of the visual.
      public: QComboBox *geometryComboBox;

      /// \brief Spin box for configuring the transparency of the visual.
      public: QDoubleSpinBox *transparencySpinBox;

      /// \brief Editable line for specifying the material of the visual.
      public: QLineEdit *materialLineEdit;

      /// \brief Spin box for configuring the X position of the visual.
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

    /// \class PartVisualTab PartVisualTab.hh
    /// \brief A tab for configuring visual properties of a part.
    class PartVisualTab : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartVisualTab();

      /// \brief Destructor
      public: ~PartVisualTab();

      /// \brief Add a visual widget to the tab.
      public: void AddVisual();

      /// \brief Reset the visual tab.
      public: void Reset();

      /// \brief Get the number of visuals.
      /// \return Number of visuals.
      public: unsigned int GetVisualCount() const;

      /// \brief Set the name of the visual.
      /// \param[in] _index Index of visual.
      /// \param[in] _name Name of visual.
      public: void SetName(unsigned int _index, const std::string &_name);

      /// \brief Get the name of the visual.
      /// \param[in] _index Index of visual.
      /// \return Name of visual.
      public: std::string GetName(unsigned int _index) const;

      /// \brief Set the pose of the visual.
      /// \param[in] _index Index of visual.
      /// \param[in] _pose Pose to set the visual to.
      public: void SetPose(unsigned int _index, const math::Pose &_pose);

      /// \brief Get the pose of the visual.
      /// \param[in] _index Index of visual.
      /// \return Pose of the visual.
      public: math::Pose GetPose(unsigned int _index) const;

      /// \brief Set the transparency of the visual.
      /// \param[in] _index Index of visual
      /// \param[in] _transparency Transparency of visual
      public: void SetTransparency(unsigned int _index, double _transparency);

      /// \brief Get the transparency of the visual.
      /// \param[in] _index Index of visual.
      public: double GetTransparency(unsigned int _index) const;

      /// \brief Get the material of the visual.
      /// \param[in] _index Index of visual.
      /// \return Name of the material.
      public: std::string GetMaterial(unsigned int _index) const;

      /// \brief Set the material of the visual.
      /// \param[in] _index Index of visual.
      /// \param[in] _material Material name.
      public: void SetMaterial(unsigned int _index,
          const std::string &_material);

      /// \brief Set the geometry of the visual.
      /// \param[in] _index Index of visual
      public: void SetGeometry(unsigned int _index,
          const std::string &_geometry);

      /// \brief Get the geometry of the visual.
      /// \param[in] _index Index of visual
      /// \return Geometry type.
      public: std::string GetGeometry(unsigned int _index) const;

      /// \brief List of visual widgets for configuring visual properties.
      private: std::vector<VisualDataWidget *> dataWidgets;

      /// \brief Widget that display visuals' properties.
      private: QTreeWidget *visualsTreeWidget;

      /// \brief Counter for the number of visuals.
      private: int counter;

      /// \brief Qt signal mapper for mapping remove button signals.
      private:  QSignalMapper *signalMapper;

      /// \brief A map of visual items to their id.
      private: std::map<int, QTreeWidgetItem *> visualItems;

      /// \brief Qt signal emitted when a visual is removed.
      /// \param[in] _name Name of visual removed.
      Q_SIGNALS: void VisualRemoved(const std::string &_name);

      /// \brief Qt signal emitted when a visual is added.
      Q_SIGNALS: void VisualAdded();

      /// \brief Qt callback when a visual is to be added.
      private slots: void OnAddVisual();

      /// \brief Qt callback when a visual is to be removed.
      /// \param[in] _item Item to be removed.
      private slots: void OnRemoveVisual(int);

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);
    };
  }
}
#endif
