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
#ifndef _MODEL_MANIP_HH_
#define _MODEL_MANIP_HH_

#include <string>
#include <vector>
#include "gazebo/gui/qt.h"
#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace gui
  {
    class BuildingMaker;

    class ModelManip : public QObject
    {
      Q_OBJECT
      public: ModelManip();

      public: virtual ~ModelManip();

      public: void SetName(const std::string &_name);

      public: void SetVisual(const rendering::VisualPtr &_visual);

      public: std::string GetName() const;

      public: rendering::VisualPtr GetVisual() const;

      public: void SetMaker(BuildingMaker *_maker);

      public: ModelManip *GetParent() const;

      public: void AttachObject(ModelManip *_object);

      public: void DetachObject(ModelManip *_object);

      public: void SetAttachedTo(ModelManip *_parent);

      public: void DetachFromParent();

      public: ModelManip *GetAttachedObject(unsigned int _index) const;

      public: unsigned int GetAttachedObjectCount() const;

      public: bool IsAttached() const;

      public: math::Vector3 Convert(const math::Vector3 &_vector);

      public: void SetPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      public: void SetPosition(double _x, double _y, double _z);

      public: void SetRotation(double _roll, double _pitch, double _yaw);

      public: void SetSize(double _width, double _depth, double _height);

      private slots: void OnSizeChanged(double _width, double _depth,
          double _height);

      private slots: void OnPoseChanged(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      private slots: void OnPoseOriginTransformed(double _x, double _y,
          double _z, double _roll, double _pitch, double _yaw);

      private slots: void OnPositionChanged(double _x, double _y, double _z);

      private slots: void OnRotationChanged(double _roll, double _pitch,
          double _yaw);

      private slots: void OnWidthChanged(double _width);

      private slots: void OnHeightChanged(double _height);

      private slots: void OnDepthChanged(double _depth);

      private slots: void OnPosXChanged(double _posX);

      private slots: void OnPosYChanged(double _posY);

      private slots: void OnPosZChanged(double _posZ);

      private slots: void OnYawChanged(double _yaw);

      private slots: void OnItemDeleted();

      private: std::string name;

      private: rendering::VisualPtr visual;

      private: math::Vector3 size;

      private: math::Pose pose;

      private: math::Pose originTransform;

      private: BuildingMaker *maker;

      private: std::vector<ModelManip *> attachedObjects;

      private: ModelManip * parent;
    };
  }
}
#endif
