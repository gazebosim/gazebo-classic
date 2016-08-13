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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <sstream>
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
        "font-size: 24px;"
      "}");

  // eye
  auto eyeLabel = new QLabel("Eye");
  this->eyeX = new QDoubleSpinBox();
  this->connect(this->eyeX, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));
  this->eyeY = new QDoubleSpinBox();
  this->connect(this->eyeY, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));
  this->eyeZ = new QDoubleSpinBox();
  this->connect(this->eyeZ, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  // target
  auto targetLabel = new QLabel("Target");
  this->targetX = new QDoubleSpinBox();
  this->connect(this->targetX, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));
  this->targetY = new QDoubleSpinBox();
  this->connect(this->targetY, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));
  this->targetZ = new QDoubleSpinBox();
  this->connect(this->targetZ, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  // up
  auto upLabel = new QLabel("Up");
  this->upX = new QDoubleSpinBox();
  this->connect(this->upX, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));
  this->upY = new QDoubleSpinBox();
  this->connect(this->upY, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));
  this->upZ = new QDoubleSpinBox();
  this->connect(this->upZ, SIGNAL(valueChanged(double)), this,
      SLOT(OnChange(double)));

  // Frame
  auto frameLayout = new QGridLayout();
  frameLayout->addWidget(eyeLabel, 0, 0);
  frameLayout->addWidget(eyeX, 1, 0);
  frameLayout->addWidget(eyeY, 2, 0);
  frameLayout->addWidget(eyeZ, 3, 0);
  frameLayout->addWidget(targetLabel, 0, 1);
  frameLayout->addWidget(targetX, 1, 1);
  frameLayout->addWidget(targetY, 2, 1);
  frameLayout->addWidget(targetZ, 3, 1);
  frameLayout->addWidget(upLabel, 0, 2);
  frameLayout->addWidget(upX, 1, 2);
  frameLayout->addWidget(upY, 2, 2);
  frameLayout->addWidget(upZ, 3, 2);
  frameLayout->setContentsMargins(4, 4, 4, 4);

  auto mainFrame = new QFrame();
  mainFrame->setLayout(frameLayout);

  // Main
  auto mainLayout = new QVBoxLayout;
  mainLayout->addWidget(mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->resize(300, 100);

  // Visual
  auto scene = rendering::get_scene();
  if (!scene)
  {
    gzerr << "No scene!" << std::endl;
    return;
  }

  this->vis = scene->GetVisual("frame");
  {
    gzerr << "No frame visual!" << std::endl;
    return;
  }
}

/////////////////////////////////////////////////
void LookAtDemoPlugin::Load(sdf::ElementPtr /*_elem*/)
{
}

/////////////////////////////////////////////////
void LookAtDemoPlugin::OnChange(const double _newValue)
{
  auto eye = ignition::math::Vector3d(this->eyeX->value(),
                                      this->eyeY->value(),
                                      this->eyeZ->value()
                                     );
  auto target = ignition::math::Vector3d(this->targetX->value(),
                                         this->targetY->value(),
                                         this->targetZ->value()
                                        );
  auto up = ignition::math::Vector3d(this->upX->value(),
                                     this->upY->value(),
                                     this->upZ->value()
                                    );
  this->vis->SetWorldPose(
      ignition::math::Matrix4d::LookAt(eye, target, up).Pose());
}

