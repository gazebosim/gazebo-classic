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
    class BoxMaker;
    class EntityMaker;

    class WallManip : public QObject
    {
      Q_OBJECT
      public: WallManip();

    };

    class BuildingMaker : public EntityMaker
    {
      public: BuildingMaker();

      public: virtual ~BuildingMaker();

      public: std::string MakeModel(math::Pose _pose);

      public: void AddPart(std::string _type, math::Vector3 _size,
          math::Pose _pose);

      public: std::string AddWall(math::Vector3 _size, math::Pose _pose);

      public: std::string AddWindow(math::Vector3 _size, math::Pose _pose);

      public: std::string AddDoor(math::Vector3 _size, math::Pose _pose);

      public: void SetPose(std::string _visualName, math::Pose _pose);

      public: void SetSize(std::string _visualName, math::Vector3 _size);

/*      public: virtual void OnMousePush(const common::MouseEvent &_event);
      public: virtual void OnMouseRelease(const common::MouseEvent &_event);
      public: virtual void OnMouseDrag(const common::MouseEvent &_event);
      public: virtual void OnMouseMove(const common::MouseEvent &_event);*/

      /// \brief
      public: virtual void Start(const rendering::UserCameraPtr _camera);
      /// \brief
      public: virtual void Stop();

      /// \brief Checks if entity is active
      public: virtual bool IsActive() const;

      /// \brief Internal init function.
      private: bool Init();

      private: virtual void CreateTheEntity();

      private: void OnCreateBuildingPart(std::string _partType);

      private: void OnSetBuildingPartPose(std::string partName,
          math::Pose _pose);

      private: void OnSetBuildingPartSize(std::string partName,
          math::Vector3 _size);

      private: std::list<rendering::VisualPtr> wallVisuals;
      private: std::list<rendering::VisualPtr> windowVisuals;
      private: std::list<rendering::VisualPtr> doorVisuals;
      private: sdf::SDFPtr modelSDF;

      private: std::string modelName;

      private: BoxMaker* boxMaker;

      private: rendering::VisualPtr modelVisual;
      private: std::list<rendering::VisualPtr> visuals;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
