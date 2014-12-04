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

#ifndef _PART_VISUAL_CONFIG_HH_
#define _PART_VISUAL_CONFIG_HH_

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

    /// \class VisualDataWidget PartVisualConfig.hh
    /// \brief A class of widgets used for configuring visual properties.
    class VisualConfigData
    {
      /// \brief Unique ID of this visual config.
      public: int id;

      /// \brief Name of the visual.
      public: std::string name;

      /// \brief config widget for configuring visual properties.
      public: ConfigWidget *configWidget;

      /// \brief Tree item associated with the configWidget.
      public: QTreeWidgetItem *treeItem;
    };

    /*/// \class VisualDataWidget PartVisualConfig.hh
    /// \brief A class of widgets used for configuring visual properties.
    class VisualDataWidget : public QObject
    {
      Q_OBJECT

      /// \brief Unique id for this widget.
      public: int id;

      /// \brief Label for displaying the visual's name.
      public: QLabel *visualNameLabel;

      /// \brief Combo box for for specifying the geometry of the visual.
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

      /// \brief Qt signal emitted when a visual is added.
      private slots: void GeometryChanged(const QString _text);
    };*/

    /// \class PartVisualConfig PartVisualConfig.hh
    /// \brief A tab for configuring visual properties of a part.
    class PartVisualConfig : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartVisualConfig();

      /// \brief Destructor
      public: ~PartVisualConfig();

      /// \brief Add a visual widget to the tab.
      /// \param[in] _name Name of visual added.
      public: void AddVisual(const std::string &_name,
          const msgs::Visual *_visualMsg = NULL);

      /// \brief Update a visual widget from a visual msg.
      /// \param[in] _name Name of visual to be updated.
      /// \param[in] _visualMsg Msg used to update the visual widget values.
      public: void UpdateVisual(const std::string &_name,
          const msgs::Visual *_visualMsg);

      /// \brief Reset the visual tab.
      public: void Reset();

      /// \brief Get the number of visuals.
      /// \return Number of visuals.
      public: unsigned int GetVisualCount() const;

      /// \brief Get the msg containing all visual data.
      /// \param[in] _name Name of visual.
      /// \return Visual msg.
      public: msgs::Visual *GetData(const std::string &_name) const;

      /*/// \brief Set the name of the visual.
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
      /// \param[in] _geometry Geometry type to set to.
      public: void SetGeometry(unsigned int _index,
          const std::string &_geometry);

      /// \brief Get the geometry of the visual.
      /// \param[in] _index Index of visual
      /// \return Geometry type.
      public: std::string GetGeometry(unsigned int _index) const;

      /// \brief Set the geometry size of the visual.
      /// \param[in] _index Index of visual
      /// \param[in] _length Size to set the geometry to.
      public: void SetGeometrySize(unsigned int _index,
          const math::Vector3 &_size);

      /// \brief Get the geometry length of the visual.
      /// \param[in] _index Index of visual
      /// \return Geometry size.
      public: math::Vector3 GetGeometrySize(unsigned int _index) const;

      /// \brief Set the geometry radius of the visual.
      /// \param[in] _index Index of visual
      /// \param[in] _length Radius to set the geometry to.
      public: void SetGeometryRadius(unsigned int _index,
          double _radius);

      /// \brief Get the geometry radius of the visual.
      /// \param[in] _index Index of visual
      /// \return Geometry radius.
      public: double GetGeometryRadius(unsigned int _index) const;

      /// \brief Set the geometry length of the visual.
      /// \param[in] _index Index of visual
      /// \param[in] _length Length to set the geometry to.
      public: void SetGeometryLength(unsigned int _index,
          double _length);

      /// \brief Get the geometry length of the visual.
      /// \param[in] _index Index of visual
      /// \return Geometry length.
      public: double GetGeometryLength(unsigned int _index) const;

      /// \brief Set the scale of the geometry.
      /// \param[in] _index Index of visual
      /// \param[in] _dimensions Geometry scale.
      public: void SetGeometryScale(unsigned int _index,
          const math::Vector3 &_scale);

      /// \brief Get the scale of the geometry.
      /// \param[in] _index Index of visual
      /// \return Geometry scale.
      public: math::Vector3 GetGeometryScale(unsigned int _index) const;

      /// \brief List of visual widgets for configuring visual properties.
      private: std::vector<VisualDataWidget *> dataWidgets;*/

      /// \brief List of config widgets for configuring visual properties.
      //private: std::vector<ConfigWidget *> configWidgets;

      /// \brief Map of id to visual config widget.
      private: std::map<int, VisualConfigData *> configs;

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
      Q_SIGNALS: void VisualAdded(const std::string &_name);

      /// \brief Qt callback when a visual is to be added.
      /// \param[in] _name Name of visual added.
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
