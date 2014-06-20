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
#ifndef _MODEL_BUILDER_WIDGET_HH_
#define _MODEL_BUILDER_WIDGET_HH_

#include "gazebo/gui/qt.h"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/math/Vector3.hh"

namespace gazebo
{
  class GLWidget;

  namespace gui
  {
    class ModelBuilderWidget : public QWidget
    {
      Q_OBJECT
      public: ModelBuilderWidget(QWidget *_parent = 0);
      public: virtual ~ModelBuilderWidget();

      public: void Init();

      private slots: void CreateBox();
      private slots: void CreateSphere();
      private slots: void CreateCylinder();

      private: void OnBoxCreate(const math::Vector3 &pos,
                                const math::Vector3 &scale);

      private: void OnSphereCreate(const math::Vector3 &pos,
                                   const math::Vector3 &scale);

      private: void OnCylinderCreate(const math::Vector3 &pos,
                                     const math::Vector3 &scale);

      private: GLWidget *glWidget;
      private: QAction *boxCreateAct, *sphereCreateAct, *cylinderCreateAct;

      private: physics::WorldPtr world;

      protected: transport::NodePtr node;
      protected: transport::PublisherPtr factoryPub;
    };
  }
}

#endif


