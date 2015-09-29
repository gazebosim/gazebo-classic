/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <boost/bind.hpp>
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsIface.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/ModelBuilderWidget.hh"

using namespace gazebo;
using namespace gui;

ModelBuilderWidget::ModelBuilderWidget(QWidget *_parent)
  : QWidget(parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;
  QSplitter *splitter = new QSplitter;

  QFrame *renderFrame = new QFrame;
  renderFrame->setLineWidth(1);
  renderFrame->setFrameShadow(QFrame::Sunken);
  renderFrame->setFrameShape(QFrame::Box);
  renderFrame->show();

  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->glWidget = new GLWidget(renderFrame);
  rendering::ScenePtr scene = rendering::create_scene("model_builder", true);
  this->glWidget->ViewScene(scene);

  frameLayout->addWidget(this->glWidget);
  renderFrame->setLayout(frameLayout);
  renderFrame->layout()->setContentsMargins(4, 4, 4, 4);

  QFrame *rightFrame = new QFrame;
  rightFrame->setLineWidth(1);
  rightFrame->setFrameShadow(QFrame::Sunken);
  rightFrame->setFrameShape(QFrame::Box);

  QToolBar *toolbar = new QToolBar(this);

  this->boxCreateAct = new QAction(QIcon(":/images/box.png"), tr("Box"), this);
  this->boxCreateAct->setStatusTip(tr("Create a box"));
  connect(this->boxCreateAct, SIGNAL(triggered()), this, SLOT(CreateBox()));
  toolbar->addAction(boxCreateAct);

  this->sphereCreateAct = new QAction(QIcon(":/images/sphere.png"),
      tr("Sphere"), this);
  this->sphereCreateAct->setStatusTip(tr("Create a sphere"));
  connect(this->sphereCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateSphere()));
  toolbar->addAction(sphereCreateAct);

  this->cylinderCreateAct = new QAction(QIcon(":/images/cylinder.png"),
      tr("Cylinder"), this);
  this->cylinderCreateAct->setStatusTip(tr("Create a sphere"));
  connect(this->cylinderCreateAct, SIGNAL(triggered()), this,
      SLOT(CreateCylinder()));
  toolbar->addAction(cylinderCreateAct);

  splitter->addWidget(renderFrame);
  splitter->addWidget(rightFrame);
  QList<int> sizes;
  sizes.push_back(splitter->width());
  sizes.push_back(0);
  splitter->setSizes(sizes);

  mainLayout->addWidget(toolbar);
  mainLayout->addWidget(splitter);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  // TODO: Use messages so that the gui doesn't depend upon physics
  physics::init();
  this->world = physics::create_world("model_builder");
  this->world->Load(sdf::ElementPtr());
  this->world->Init();
  this->world->SetPaused(true);

  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<?xml version ='1.0'?>";

  newModelStr << "<model name ='my_new_model'>\
    <static>true</static>\
    <pose>0 0 0 0 0 0</pose>\
    <link name ='body'>\
      <collision name ='geom'>\
        <geometry>\
          <box size ='1 1 1'/>\
        </geometry>\
        <mass>0.5</mass>\
      </collision>\
      <visual>\
        <geometry>\
          <box size ='1 1 1'/>\
        </geometry>\
        <material name ='Gazebo/Grey'/>\
        <cast_shadows>true</cast_shadows>\
        <shader>pixel</shader>\
      </visual>\
    </link>\
  </model>";

  msg.set_sdf(newModelStr.str());

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("model_builder");

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->factoryPub->Publish(msg);
}

ModelBuilderWidget::~ModelBuilderWidget()
{
  delete glWidget;
}

void ModelBuilderWidget::Init()
{
  this->glWidget->show();
}

void ModelBuilderWidget::CreateBox()
{
  this->glWidget->CreateEntity("box",
      boost::bind(&ModelBuilderWidget::OnBoxCreate, this, _1, _2));
}

void ModelBuilderWidget::CreateSphere()
{
  this->glWidget->CreateEntity("sphere",
      boost::bind(&ModelBuilderWidget::OnSphereCreate, this, _1, _2));
}

void ModelBuilderWidget::CreateCylinder()
{
  this->glWidget->CreateEntity("cylinder",
      boost::bind(&ModelBuilderWidget::OnCylinderCreate, this, _1, _2));
}

void ModelBuilderWidget::OnBoxCreate(const math::Vector3 &/*pos*/,
                                     const math::Vector3 &/*scale*/)
{
}

void ModelBuilderWidget::OnSphereCreate(const math::Vector3 &/*pos*/,
                                     const math::Vector3 &/*scale*/)
{
}

void ModelBuilderWidget::OnCylinderCreate(const math::Vector3 &/*pos*/,
                                     const math::Vector3 &/*scale*/)
{
}
