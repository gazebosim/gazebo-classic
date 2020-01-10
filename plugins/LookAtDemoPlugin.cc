/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <sstream>

#include <gazebo/gui/Actions.hh>
#include "LookAtDemoPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(LookAtDemoPlugin)

/////////////////////////////////////////////////
LookAtDemoPlugin::LookAtDemoPlugin()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame {"
        "background-color : rgba(0, 0, 0, 0);"
        "color : white;"
        "font-size: 18px;"
      "}");

  // eye
  auto eyeLabel = new QLabel("Eye");

  this->eyeX = new QDoubleSpinBox();
  this->eyeX->setRange(-1000, 1000);
  this->eyeX->setSingleStep(0.1);
  this->connect(this->eyeX, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  this->eyeY = new QDoubleSpinBox();
  this->eyeY->setRange(-1000, 1000);
  this->eyeY->setSingleStep(0.1);
  this->connect(this->eyeY, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  this->eyeZ = new QDoubleSpinBox();
  this->eyeZ->setRange(-1000, 1000);
  this->eyeZ->setSingleStep(0.1);
  this->connect(this->eyeZ, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  // target
  auto targetLabel = new QLabel("Target");

  this->targetX = new QDoubleSpinBox();
  this->targetX->setRange(-1000, 1000);
  this->targetX->setSingleStep(0.1);
  this->targetX->setValue(1.0);
  this->connect(this->targetX, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  this->targetY = new QDoubleSpinBox();
  this->targetY->setRange(-1000, 1000);
  this->targetY->setSingleStep(0.1);
  this->connect(this->targetY, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  this->targetZ = new QDoubleSpinBox();
  this->targetZ->setRange(-1000, 1000);
  this->targetZ->setSingleStep(0.1);
  this->connect(this->targetZ, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  // up
  auto upLabel = new QLabel("Up");

  this->upX = new QDoubleSpinBox();
  this->upX->setRange(-1000, 1000);
  this->upX->setSingleStep(0.1);
  this->connect(this->upX, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  this->upY = new QDoubleSpinBox();
  this->upY->setRange(-1000, 1000);
  this->upY->setSingleStep(0.1);
  this->connect(this->upY, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  this->upZ = new QDoubleSpinBox();
  this->upZ->setRange(-1000, 1000);
  this->upZ->setSingleStep(0.1);
  this->upZ->setValue(1.0);
  this->connect(this->upZ, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  // Frame
  auto frameLayout = new QGridLayout();
  frameLayout->addWidget(new QLabel("X"), 1, 0);
  frameLayout->addWidget(new QLabel("Y"), 2, 0);
  frameLayout->addWidget(new QLabel("Z"), 3, 0);
  frameLayout->addWidget(eyeLabel, 0, 1);
  frameLayout->addWidget(eyeX, 1, 1);
  frameLayout->addWidget(eyeY, 2, 1);
  frameLayout->addWidget(eyeZ, 3, 1);
  frameLayout->addWidget(targetLabel, 0, 2);
  frameLayout->addWidget(targetX, 1, 2);
  frameLayout->addWidget(targetY, 2, 2);
  frameLayout->addWidget(targetZ, 3, 2);
  frameLayout->addWidget(upLabel, 0, 3);
  frameLayout->addWidget(upX, 1, 3);
  frameLayout->addWidget(upY, 2, 3);
  frameLayout->addWidget(upZ, 3, 3);
  frameLayout->setContentsMargins(4, 4, 4, 4);

  // Main
  auto mainFrame = new QFrame();
  mainFrame->setLayout(frameLayout);

  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->resize(300, 100);
}

/////////////////////////////////////////////////
LookAtDemoPlugin::~LookAtDemoPlugin()
{
  this->modelModifyPub.reset();
  this->node->Fini();
}

/////////////////////////////////////////////////
void LookAtDemoPlugin::Load(sdf::ElementPtr /*_elem*/)
{
  // Publisher
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->modelModifyPub =
      this->node->Advertise<msgs::Model>("~/model/modify");
}

/////////////////////////////////////////////////
void LookAtDemoPlugin::OnChange(const double /*_newValue*/)
{
  auto eye = ignition::math::Vector3d(this->eyeX->value(),
                                      this->eyeY->value(),
                                      this->eyeZ->value());
  auto target = ignition::math::Vector3d(this->targetX->value(),
                                         this->targetY->value(),
                                         this->targetZ->value());
  auto up = ignition::math::Vector3d(this->upX->value(),
                                     this->upY->value(),
                                     this->upZ->value());
  auto lookat = ignition::math::Matrix4d::LookAt(eye, target, up).Pose();

  // Publish model modify messages
  msgs::Model msg;
  msg.set_name("frame");
  msgs::Set(msg.mutable_pose(), lookat);

  this->modelModifyPub->Publish(msg);

  msg.set_name("target");
  msgs::Set(msg.mutable_pose(), ignition::math::Pose3d(target,
      ignition::math::Quaterniond::Identity));

  this->modelModifyPub->Publish(msg);

  auto desZ = ignition::math::Matrix4d::LookAt(eye, up).Pose();
  msg.set_name("desired_z");
  msgs::Set(msg.mutable_pose(), desZ);

  this->modelModifyPub->Publish(msg);
}

