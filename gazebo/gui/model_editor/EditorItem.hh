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

      public: EditorItem();

      public: ~EditorItem();

      public: virtual QVector3D GetSize() const;

      public: virtual QVector3D GetScenePosition() const;

      public: virtual double GetSceneRotation() const;

      public: virtual std::string GetType() const;

      public: virtual std::string GetName() const;

      public: virtual void SetName(const std::string &_name);

      Q_SIGNALS: void sizeChanged(double _width, double _depth,
          double _height);

      Q_SIGNALS: void poseChanged(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      Q_SIGNALS: void poseOriginTransformed(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      Q_SIGNALS: void positionChanged(double _x, double _y, double _z);

      Q_SIGNALS: void rotationChanged(double _roll, double _pitch, double _yaw);

      Q_SIGNALS: void widthChanged(double _width);

      Q_SIGNALS: void depthChanged(double _depth);

      Q_SIGNALS: void heightChanged(double _height);

      Q_SIGNALS: void posXChanged(double _posX);

      Q_SIGNALS: void posYChanged(double _posY);

      Q_SIGNALS: void posZChanged(double _posX);

      Q_SIGNALS: void yawChanged(double _yaw);

      Q_SIGNALS: void originChanged(double _xRatio, double _yRatio,
          double _zRatio);

      Q_SIGNALS: void itemDeleted();

      protected: std::string editorType;

      protected: std::string name;
    };
  }
}

#endif
