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
MySlider::MySlider(const std::string &_name, QWidget *_parent)
  : QWidget(_parent), name(_name)
{
  QHBoxLayout *mainLayout = new QHBoxLayout;

  this->multiplier = new QDoubleSpinBox;
  this->multiplier->setRange(-100.0, 100.0);
  this->multiplier->setSingleStep(0.01);
  this->multiplier->setValue(0.00);

  mainLayout->addWidget(new QLabel(tr(_name.c_str())), 0);
  mainLayout->addItem(new QSpacerItem(20, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum));
  mainLayout->addWidget(this->multiplier, 0);
  mainLayout->addWidget(new QLabel("N-m", this),0);
  this->setLayout(mainLayout);

  connect(this->multiplier, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
}

/////////////////////////////////////////////////
void MySlider::OnChanged(double _value)
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

  QVBoxLayout *mainLayout = new QVBoxLayout;
  QFrame *frame = new QFrame;
  frame->setLineWidth(0);
  frame->setFrameShadow(QFrame::Sunken);
  frame->setFrameShape(QFrame::Box);

  QVBoxLayout *vLayout = new QVBoxLayout;
  for (int i = 0; i < modelMsg.joint_size(); ++i)
  {
    std::string jointName =modelMsg.joint(i).name();
    MySlider *slider = new MySlider(jointName, this);
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

  std::string title = "Model: ";
  title += _modelName;
  mainLayout->addWidget(new QLabel(tr(title.c_str())));
  mainLayout->addWidget(scrollArea);
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
  std::map<std::string, MySlider*>::iterator iter;
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
