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

    class BuildingMaker;

    class EditorItem;

    class ModelManip : public QObject
    {
      Q_OBJECT
      public: ModelManip();

      public: ~ModelManip();

      public: void SetName(std::string _name);

      public: void SetVisual(rendering::VisualPtr _visual);

      public: std::string GetName();

      public: rendering::VisualPtr GetVisual();

      public: void SetMaker(BuildingMaker *_maker);

      public: void AttachObject(ModelManip *_object);

      public: void DetachObject(ModelManip *_object);

      public: void SetAttachedTo(ModelManip *_parent);

      public: ModelManip *GetAttachedObject(unsigned int _index);

      public: unsigned int GetAttachedObjectCount();

      public: bool IsAttached();

      public: math::Vector3 Convert(math::Vector3 _vector);

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

      public: void RemovePart(std::string _partName);

      public: void RemoveWall(std::string _wallName);

      public: void ConnectItem(std::string _partName, EditorItem *_item);

      public: static math::Vector3 ConvertSize(QVector3D _size);

      public: static math::Vector3 ConvertSize(double _width, double _depth,
          double _height);

      public: static math::Pose ConvertPose(QVector3D _pos, QVector3D _rot);

      public: static math::Pose ConvertPose(double _x, double _y, double _z,
          double _roll, double _pitch, double _yaw);

      public: static double Convert(double _value);

      public: static double ConvertAngle(double _angle);

      /// \brief
      public: virtual void Start(const rendering::UserCameraPtr _camera);

      /// \brief
      public: virtual void Stop();

      /// \brief Checks if entity is active
      public: virtual bool IsActive() const;

      public: void SaveToSDF(std::string _savePath);

      /// \brief Internal init function.
      private: bool Init();

      private: std::string CreateModel(math::Pose _pose);

      private: void GenerateSDF();

      private: void GenerateSDFWithCSG();

      private: virtual void CreateTheEntity();

      private: std::string GetTemplateSDFString();

      private: void SubdivideRectSurface(const QRect _surface,
        const std::vector<QRect> _holes, std::vector<QRect> &_subdivisions);

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
    };
  }
}
#endif
