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

#include "sdf/sdf.hh"
#include "common/Events.hh"
#include "gui/EntityMaker.hh"
#include "gui/qt.h"

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

      public: void SetModelName(std::string _modelName);

      public: void FinishModel();

      public: std::string AddPart(std::string _type, QVector3D _size,
          QVector3D _pos, double _angle);

      public: std::string AddWall(QVector3D _size, QVector3D _pos,
          double _angle);

      public: std::string AddWindow(QVector3D _size, QVector3D _pos,
          double _angle);

      public: std::string AddDoor(QVector3D _size, QVector3D _pos,
          double _angle);

      public: std::string AddStairs(QVector3D _size, QVector3D _pos,
          double _angle, int _steps);

      public: std::string AddFloor(QVector3D _size, QVector3D _pos,
          double _angle);

      public: void RemovePart(std::string _partName);

      public: void RemoveWall(std::string _wallName);

      public: void ConnectItem(std::string _partName, EditorItem *_item);

      public: void AttachObject(std::string _child, std::string _parent);

      public: void DetachObject(std::string _child, std::string _parent);

      public: static math::Vector3 ConvertSize(QVector3D _size);

      public: static math::Vector3 ConvertSize(double _width, double _depth,
          double _height);

      public: static math::Pose ConvertPose(QVector3D _pos, QVector3D _rot);

      public: static math::Pose ConvertPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      public: static double Convert(double _value);

      public: static double ConvertAngle(double _angle);

      /// \brief
      public: void Reset();

      /// \brief
      public: virtual void Start(const rendering::UserCameraPtr _camera);

      /// \brief
      public: virtual void Stop();

      /// \brief Checks if entity is active
      public: virtual bool IsActive() const;

      public: void SaveToSDF(std::string _savePath);

      /// \brief Internal init function.
      private: bool Init();

      private: std::string CreateModel();

      private: void GenerateSDF();

      private: void GenerateSDFWithCSG();

      private: virtual void CreateTheEntity();

      private: std::string GetTemplateSDFString();

      private: static bool PointCompareY(const QPointF &_a, const QPointF &_b);

      private: static bool RectCompareX(const QRectF &_a, const QRectF &_b);

      private: static bool RectCompareY(const QRectF &_a, const QRectF &_b);

      private: void SubdivideRectSurface(const QRectF _surface,
        const std::vector<QRectF> _holes, std::vector<QRectF> &_subdivisions);

      private: std::map<std::string, ModelManip *> allItems;

//      private: std::map<std::string, ModelManip *> walls;

//      private: std::map<std::string, ModelManip *> windows;

//      private: std::map<std::string, ModelManip *> stairs;

      private: sdf::SDFPtr modelSDF;

      private: sdf::SDFPtr modelTemplateSDF;

      private: std::string modelName;

      private: std::string savePath;

      private: rendering::VisualPtr modelVisual;

//      private: std::list<rendering::VisualPtr> visuals;

      private: math::Pose modelPose;

      public: static double conversionScale;

      private: int wallCounter;

      private: int windowCounter;

      private: int doorCounter;

      private: int stairsCounter;

      private: int floorCounter;
    };
  }
}
#endif
