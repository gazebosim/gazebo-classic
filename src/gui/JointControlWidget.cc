/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include "transport/Node.hh"
#include "transport/Transport.hh"
#include "gui/JointControlWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointForceControl::JointForceControl(const std::string &_name, QWidget *_parent)
  : QWidget(_parent), name(_name)
{
  QHBoxLayout *mainLayout = new QHBoxLayout;

  QDoubleSpinBox *forceSpin = new QDoubleSpinBox;
  forceSpin->setRange(-100.0, 100.0);
  forceSpin->setSingleStep(0.001);
  forceSpin->setValue(0.000);

  mainLayout->addWidget(new QLabel(tr(_name.c_str())), 0);
  mainLayout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum));
  mainLayout->addWidget(forceSpin, 0);
  mainLayout->addWidget(new QLabel("N-m", this),0);
  this->setLayout(mainLayout);

  connect(forceSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
}

/////////////////////////////////////////////////
void JointForceControl::OnChanged(double _value)
{
  emit changed(_value, this->name);
}

/////////////////////////////////////////////////
JointPIDControl::JointPIDControl(const std::string &_name, QWidget *_parent)
  : QWidget(_parent), name(_name)
{
  QHBoxLayout *mainLayout = new QHBoxLayout;

  QDoubleSpinBox *posSpin = new QDoubleSpinBox;
  posSpin->setRange(-3.1415, 3.1415);
  posSpin->setSingleStep(0.001);
  posSpin->setValue(0.000);

  mainLayout->addWidget(new QLabel(tr(_name.c_str())), 0);
  mainLayout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum));
  mainLayout->addWidget(posSpin, 0);
  mainLayout->addWidget(new QLabel("radians", this), 0);
  this->setLayout(mainLayout);

  connect(posSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
}

/////////////////////////////////////////////////
void JointPIDControl::OnChanged(double _value)
{
  emit changed(_value, this->name);
}

/////////////////////////////////////////////////
JointControlWidget::JointControlWidget(const std::string &_modelName,
                                       QWidget *_parent)
  : QWidget(_parent)
{
  this->setWindowTitle("Joint Control");
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->jointPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + _modelName + "/joint_cmd");

  this->requestMsg = msgs::CreateRequest("entity_info");
  this->requestMsg->set_data(_modelName);

  msgs::Response response = transport::request("default", *this->requestMsg);

  msgs::Model modelMsg;
  modelMsg.ParseFromString(response.serialized_data());


  QScrollArea *scrollArea = new QScrollArea;
  QScrollArea *pidScrollArea = new QScrollArea;

  QFrame *frame = new QFrame;
  frame->setLineWidth(0);
  frame->setFrameShadow(QFrame::Sunken);
  frame->setFrameShape(QFrame::Box);

  QVBoxLayout *vLayout = new QVBoxLayout;
  for (int i = 0; i < modelMsg.joint_size(); ++i)
  {
    std::string jointName =modelMsg.joint(i).name();
    JointForceControl *slider = new JointForceControl(jointName, this);
    this->sliders[jointName] = slider;
    vLayout->addWidget(slider);
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnChanged(double, const std::string &)));
  }

  frame->setLayout(vLayout);
  frame->layout()->setContentsMargins(4, 0, 0, 0);
  frame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  scrollArea->setWidget(frame);
  scrollArea->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QTabWidget *tabWidget = new QTabWidget;
  tabWidget->addTab(scrollArea, tr("Force"));
  tabWidget->addTab(pidScrollArea, tr("PID"));

  // std::string title = "Model: ";
  // title += _modelName;
  // mainLayout->addWidget(new QLabel(tr(title.c_str())));
  //mainLayout->addWidget(scrollArea);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(tabWidget);  
  mainLayout->setContentsMargins(4, 4, 4, 4);

  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);
}

/////////////////////////////////////////////////
JointControlWidget::~JointControlWidget()
{
}

/////////////////////////////////////////////////
void JointControlWidget::OnChanged(double _value, const std::string &_name)
{
  std::map<std::string, JointForceControl*>::iterator iter;
  iter = this->sliders.find(_name);
  if (iter != this->sliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.set_force(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::Load(const std::string &/*_modelName*/)
{
}
