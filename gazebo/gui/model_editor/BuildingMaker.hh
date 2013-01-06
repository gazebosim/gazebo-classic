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
#ifndef _BUILDING_MAKER_HH_
#define _BUILDING_MAKER_HH_

#include <list>
#include <string>

#include "gazebo/sdf/sdf.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/gui/qt.h"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace gui
  {
    class EntityMaker;

    class EditorItem;

    class ModelManip;

    class BuildingMaker : public EntityMaker
    {
      public: BuildingMaker();

      public: virtual ~BuildingMaker();

      public: void SetModelName(const std::string &_modelName);

      public: void FinishModel();

      public: std::string AddPart(const std::string &_type,
          const QVector3D &_size, const QVector3D &_pos, double _angle);

      public: std::string AddWall(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      public: std::string AddWindow(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      public: std::string AddDoor(const QVector3D &_size, const QVector3D &_pos,
          double _angle);

      public: std::string AddStairs(const QVector3D &_size,
          const QVector3D &_pos, double _angle, int _steps);

      public: std::string AddFloor(const QVector3D &_size,
          const QVector3D &_pos, double _angle);

      public: void RemovePart(const std::string &_partName);

      public: void RemoveWall(const std::string &_wallName);

      public: void ConnectItem(const std::string &_partName,
          const EditorItem *_item);

      public: void AttachObject(const std::string &_child,
          const std::string &_parent);

      public: void DetachObject(const std::string &_child,
          const std::string &_parent);

      public: static math::Vector3 ConvertSize(const QVector3D &_size);

      public: static math::Vector3 ConvertSize(double _width, double _depth,
          double _height);

      public: static math::Pose ConvertPose(const QVector3D &_pos,
          const QVector3D &_rot);

      public: static math::Pose ConvertPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      public: static double Convert(double _value);

      public: static double ConvertAngle(double _angle);

      public: void SaveToSDF(const std::string &_savePath);

      /// \brief
      public: void Reset();

      /// \brief
      public: virtual void Start(const rendering::UserCameraPtr _camera);

      /// \brief
      public: virtual void Stop();

      public: void GenerateSDF();

      /// \brief Checks if entity is active
      public: virtual bool IsActive() const;

      private: virtual void CreateTheEntity();

      /// \brief Internal init function.
      private: bool Init();

      private: std::string CreateModel();

      private: void GenerateSDFWithCSG();

      private: std::string GetTemplateSDFString();

      private: static bool PointCompareY(const QPointF &_a, const QPointF &_b);

      private: static bool RectCompareX(const QRectF &_a, const QRectF &_b);

      private: static bool RectCompareY(const QRectF &_a, const QRectF &_b);

      private: void SubdivideRectSurface(const QRectF &_surface,
        const std::vector<QRectF> &_holes, std::vector<QRectF> &_subdivisions);

      public: static double conversionScale;

      private: std::map<std::string, ModelManip *> allItems;

      private: sdf::SDFPtr modelSDF;

      private: sdf::SDFPtr modelTemplateSDF;

      private: std::string modelName;

      private: std::string savePath;

      private: rendering::VisualPtr modelVisual;

      private: math::Pose modelPose;

      private: int wallCounter;

      private: int windowCounter;

      private: int doorCounter;

      private: int stairsCounter;

      private: int floorCounter;
    };
  }
}
#endif
