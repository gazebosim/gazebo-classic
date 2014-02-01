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

#ifndef _EDITOR_ITEM_HH_
#define _EDITOR_ITEM_HH_

#include <string>
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class EditorItem EditorItem.hh
    /// \brief Base class of an item in the editor.
    class EditorItem : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: EditorItem();

      /// \brief Destructor
      public: ~EditorItem();

      /// \brief Get the size of the item in pixels.
      /// \return Size in pixels.
      public: virtual QVector3D GetSize() const;

      /// \brief Get the scene position of editor item.
      /// \return Scene position in pixel coordinates.
      public: virtual QVector3D GetScenePosition() const;

      /// \brief Get the scene rotation of the editor item.
      /// \return Scene rotation in degrees.
      public: virtual double GetSceneRotation() const;

      /// \brief Get the type of the editor item.
      /// \return Type of the item.
      public: virtual std::string GetType() const;

      /// \brief Get the name of the editor item.
      /// \return Name of the item.
      public: virtual std::string GetName() const;

      /// \brief Set the name of this editor item.
      /// \param[in] _name Name to set the editor item to.
      public: virtual void SetName(const std::string &_name);

      /// \brief Qt signal emitted when the editor item size has changed.
      /// \param[in] _width Width of item in pixels.
      /// \param[in] _depth Depth of item in pixels.
      /// \param[in] _height Height of item in pixels.
      Q_SIGNALS: void SizeChanged(double _width, double _depth,
          double _height);

      /// \brief Qt signal emitted when the editor item pose has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void PoseChanged(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Qt signal emitted when the editor item pose origin has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void PoseOriginTransformed(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Qt signal emitted when the editor item position has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      Q_SIGNALS: void PositionChanged(double _x, double _y, double _z);

      /// \brief Qt signal emitted when the editor item rotation has changed.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void RotationChanged(double _roll, double _pitch, double _yaw);

      /// \brief Qt signal emitted when the editor item width has changed.
      /// \param[in] _width Width of item in pixels.
      Q_SIGNALS: void WidthChanged(double _width);

      /// \brief Qt signal emitted when the editor item depth has changed.
      /// \param[in] _depth Depth of item in pixels.
      Q_SIGNALS: void DepthChanged(double _depth);

      /// \brief Qt signal emitted when the editor item height has changed.
      /// \param[in] _height Height of item in pixels.
      Q_SIGNALS: void HeightChanged(double _height);

      /// \brief Qt signal emitted when the editor item's X position has
      /// changed.
      /// \param[in] _x X position of item in pixels.
      Q_SIGNALS: void PosXChanged(double _posX);

      /// \brief Qt signal emitted when the editor item's Y position has
      /// changed.
      /// \param[in] _y Y position of item in pixels.
      Q_SIGNALS: void PosYChanged(double _posY);

      /// \brief Qt signal emitted when the editor item's Z position has
      /// changed.
      /// \param[in] _z Z position of item in pixels.
      Q_SIGNALS: void PosZChanged(double _posZ);

      /// \brief Qt signal emitted when the editor item yaw rotation has
      /// changed.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void YawChanged(double _yaw);

      /// \brief Qt signal emitted when the editor item is being deleted.
      Q_SIGNALS: void ItemDeleted();

      /// \brief Type of editor item.
      protected: std::string editorType;

      /// \brief Name of editor item.
      protected: std::string name;
    };
    /// \}
  }
}

#endif
