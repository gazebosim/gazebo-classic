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

#ifndef _EDITOR_ITEM_HH_
#define _EDITOR_ITEM_HH_

#include <string>
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace gui
  {
    class EditorItem : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: EditorItem();

      /// \brief Destructor
      public: ~EditorItem();

      /// \brief Get the size of item in pixels.
      /// \return The item size in pixels.
      public: virtual QVector3D GetSize() const;

      /// \brief Get the scene position of item in pixels.
      /// \return The scene position in pixels.
      public: virtual QVector3D GetScenePosition() const;

      /// \brief Get the scene rotation of item in degrees.
      /// \return The scene rotation in degrees.
      public: virtual double GetSceneRotation() const;

      /// \brief Get the type of this editor item.
      /// \return The type of the item.
      public: virtual std::string GetType() const;

      /// \brief Get the name of this editor item.
      /// \return The name of the item.
      public: virtual std::string GetName() const;

      /// \brief Set the name of this editor item.
      /// \param[in] _name Name to set the editor item to.
      public: virtual void SetName(const std::string &_name);

      /// \brief Qt signal to emit when the editor item size has changed.
      /// \param[in] _width Width of item in pixels.
      /// \param[in] _depth Depth of item in pixels.
      /// \param[in] _height Height of item in pixels.
      Q_SIGNALS: void sizeChanged(double _width, double _depth,
          double _height);

      /// \brief Qt signal to emit when the editor item pose has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void poseChanged(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Qt signal to emit when the editor item pose origin has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void poseOriginTransformed(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      /// \brief Qt signal to emit when the editor item position has changed.
      /// \param[in] _x X position of item in pixels.
      /// \param[in] _y Y position of item in pixels.
      /// \param[in] _z Z position of item in pixels.
      Q_SIGNALS: void positionChanged(double _x, double _y, double _z);

      /// \brief Qt signal to emit when the editor item rotation has changed.
      /// \param[in] _roll Roll rotation of item in degrees.
      /// \param[in] _pitch Pitch rotation of item in degrees.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void rotationChanged(double _roll, double _pitch, double _yaw);

      /// \brief Qt signal to emit when the editor item width has changed.
      /// \param[in] _width Width of item in pixels.
      Q_SIGNALS: void widthChanged(double _width);

      /// \brief Qt signal to emit when the editor item depth has changed.
      /// \param[in] _depth Depth of item in pixels.
      Q_SIGNALS: void depthChanged(double _depth);

      /// \brief Qt signal to emit when the editor item height has changed.
      /// \param[in] _height Height of item in pixels.
      Q_SIGNALS: void heightChanged(double _height);

      /// \brief Qt signal to emit when the editor item X position has changed.
      /// \param[in] _x X position of item in pixels.
      Q_SIGNALS: void posXChanged(double _posX);

      /// \brief Qt signal to emit when the editor item Y position has changed.
      /// \param[in] _y Y position of item in pixels.
      Q_SIGNALS: void posYChanged(double _posY);

      /// \brief Qt signal to emit when the editor item Z position has changed.
      /// \param[in] _z Z position of item in pixels.
      Q_SIGNALS: void posZChanged(double _posZ);

      /// \brief Qt signal to emit when the editor item yaw rotation has
      /// changed.
      /// \param[in] _yaw Yaw rotation of item in degrees.
      Q_SIGNALS: void yawChanged(double _yaw);

      /// \brief Qt signal to emit when the editor item is being deleted
      Q_SIGNALS: void itemDeleted();

      /// \brief Type of editor item
      protected: std::string editorType;

      /// \brief Name of editor item
      protected: std::string name;
    };
  }
}

#endif
