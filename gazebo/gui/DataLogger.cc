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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/DataLogger.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DataLogger::DataLogger(QWidget *_parent)
  : QDialog(_parent)
{
  // This name is used in the qt style sheet
  this->setObjectName("dataLogger");
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Data Logger"));

  // Create the main layout for this widget
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  QPushButton *recordButton = new QPushButton("R");
  connect(recordButton, SIGNAL(clicked()),
          this, SLOT(OnRecord()));

  buttonLayout->addWidget(recordButton);

/*

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;


  frameLayout->setContentsMargins(4, 4, 4, 4);
  frame->setLayout(frameLayout);

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton("Cancel");
  connect(cancelButton, SIGNAL(clicked()),
          this, SLOT(OnCancel()));

  this->okayButton = new QPushButton("Okay");
  this->okayButton->setEnabled(false);
  connect(this->okayButton, SIGNAL(clicked()),
          this, SLOT(OnOkay()));

  buttonLayout->addWidget(cancelButton);
  buttonLayout->addStretch(2);
  buttonLayout->addWidget(this->okayButton);
  */

  //mainLayout->addWidget(frame);
  mainLayout->addLayout(buttonLayout);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->pub = this->node->Advertise<msgs::LogControl>("~/log/control");
}

/////////////////////////////////////////////////
DataLogger::~DataLogger()
{
}

/////////////////////////////////////////////////
void DataLogger::OnRecord()
{
  printf("On Record\n");
  msgs::LogControl msg;
  msg.set_start(true);
  this->pub->Publish(msg);
}
