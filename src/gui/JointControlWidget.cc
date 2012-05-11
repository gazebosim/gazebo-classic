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
JointForceControl::JointForceControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent)
  : QWidget(_parent), name(_name)
{

  QDoubleSpinBox *forceSpin = new QDoubleSpinBox;
  forceSpin->setRange(-100.0, 100.0);
  forceSpin->setSingleStep(0.001);
  forceSpin->setDecimals(3);
  forceSpin->setValue(0.000);

  int r = _layout->rowCount()-1;
  _layout->addWidget(forceSpin, r, 2);

  connect(forceSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
}

/////////////////////////////////////////////////
void JointForceControl::OnChanged(double _value)
{
  emit changed(_value, this->name);
}

/////////////////////////////////////////////////
JointPIDPosControl::JointPIDPosControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent)
  : QWidget(_parent), name(_name)
{
  this->posSpin = new QDoubleSpinBox;
  this->posSpin->setRange(-360, 360);
  this->posSpin->setSingleStep(0.001);
  this->posSpin->setDecimals(3);
  this->posSpin->setValue(0.000);

  QDoubleSpinBox *pGainSpin = new QDoubleSpinBox;
  pGainSpin->setMinimum(0.0);
  pGainSpin->setSingleStep(0.01);
  pGainSpin->setDecimals(3);
  pGainSpin->setValue(1.000);

  QDoubleSpinBox *iGainSpin = new QDoubleSpinBox;
  iGainSpin->setMinimum(0.0);
  iGainSpin->setSingleStep(0.01);
  iGainSpin->setDecimals(3);
  iGainSpin->setValue(0.100);

  QDoubleSpinBox *dGainSpin = new QDoubleSpinBox;
  dGainSpin->setMinimum(0.0);
  dGainSpin->setSingleStep(0.01);
  dGainSpin->setDecimals(3);
  dGainSpin->setValue(0.010);

  int r = _layout->rowCount()-1;
  _layout->addWidget(this->posSpin, r, 2);
  _layout->addWidget(pGainSpin, r, 3);
  _layout->addWidget(iGainSpin, r, 4);
  _layout->addWidget(dGainSpin, r, 5);

  connect(this->posSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
  connect(pGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnPChanged(double)));
  connect(iGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnIChanged(double)));
  connect(dGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnDChanged(double)));

  this->radians = true;
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetToDegrees()
{
  if (this->radians)
  {
    this->radians = false;
    this->posSpin->setValue(GZ_RTOD(this->posSpin->value()));
  }
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetToRadians()
{
  if (!this->radians)
  {
    this->radians = true;
    this->posSpin->setValue(GZ_DTOR(this->posSpin->value()));
  }
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnChanged(double _value)
{
  if (this->radians)
    emit changed(_value, this->name);
  else
    emit changed(GZ_DTOR(_value), this->name);
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnPChanged(double _value)
{
  emit pChanged(_value, this->name);
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnIChanged(double _value)
{
  emit iChanged(_value, this->name);
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnDChanged(double _value)
{
  emit dChanged(_value, this->name);
}

/////////////////////////////////////////////////
JointPIDVelControl::JointPIDVelControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent)
  : QWidget(_parent), name(_name)
{
  QDoubleSpinBox *posSpin = new QDoubleSpinBox;
  posSpin->setRange(-360, 360);
  posSpin->setSingleStep(0.001);
  posSpin->setDecimals(3);
  posSpin->setValue(0.000);

  QDoubleSpinBox *pGainSpin = new QDoubleSpinBox;
  pGainSpin->setMinimum(0.0);
  pGainSpin->setSingleStep(0.01);
  pGainSpin->setDecimals(3);
  pGainSpin->setValue(1.000);

  QDoubleSpinBox *iGainSpin = new QDoubleSpinBox;
  iGainSpin->setMinimum(0.0);
  iGainSpin->setSingleStep(0.01);
  iGainSpin->setDecimals(3);
  iGainSpin->setValue(0.100);

  QDoubleSpinBox *dGainSpin = new QDoubleSpinBox;
  dGainSpin->setMinimum(0.0);
  dGainSpin->setSingleStep(0.01);
  dGainSpin->setDecimals(3);
  dGainSpin->setValue(0.010);

  int r = _layout->rowCount()-1;
  _layout->addWidget(posSpin, r, 2);
  _layout->addWidget(pGainSpin, r, 3);
  _layout->addWidget(iGainSpin, r, 4);
  _layout->addWidget(dGainSpin, r, 5);

  connect(posSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
  connect(pGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnPChanged(double)));
  connect(iGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnIChanged(double)));
  connect(dGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnDChanged(double)));
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnChanged(double _value)
{
  emit changed(_value, this->name);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnPChanged(double _value)
{
  emit pChanged(_value, this->name);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnIChanged(double _value)
{
  emit iChanged(_value, this->name);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnDChanged(double _value)
{
  emit dChanged(_value, this->name);
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


  // Create the Force control scroll area
  QScrollArea *scrollArea = new QScrollArea;
  QFrame *frame = new QFrame;
  frame->setLineWidth(1);
  frame->setFrameShape(QFrame::NoFrame);

  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum), 0, 0, 2);
  gridLayout->addWidget(new QLabel("Newton-meter", this), 0, 2);
  for (int i = 0; i < modelMsg.joint_size(); ++i)
  {
    std::string jointName =modelMsg.joint(i).name();
    gridLayout->addWidget(new QLabel(tr(jointName.c_str())), i+1, 0);
    gridLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                   QSizePolicy::Minimum), i+1, 1);

    JointForceControl *slider = new JointForceControl(jointName, gridLayout,
                                                      this);
    this->sliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnForceChanged(double, const std::string &)));
  }

  frame->setLayout(gridLayout);
  frame->layout()->setContentsMargins(4, 0, 0, 0);
  frame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  scrollArea->setWidget(frame);
  scrollArea->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);


  // Create a PID Pos scroll area
  QScrollArea *pidPosScrollArea = new QScrollArea;
  QFrame *pidPosFrame = new QFrame;
  pidPosFrame->setLineWidth(0);
  gridLayout = new QGridLayout;

  gridLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum), 0, 0, 2);
  QComboBox *unitsCombo = new QComboBox;
  unitsCombo->addItem("Radians");
  unitsCombo->addItem("Degrees");
  connect(unitsCombo, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnPIDPosUnitsChanged(int)));

  gridLayout->addWidget(unitsCombo, 0, 2);
  gridLayout->addWidget(new QLabel("P Gain", this), 0, 3);
  gridLayout->addWidget(new QLabel("I Gain", this), 0, 4);
  gridLayout->addWidget(new QLabel("D Gain", this), 0, 5);

  for (int i = 0; i < modelMsg.joint_size(); ++i)
  {
    std::string jointName = modelMsg.joint(i).name();
    gridLayout->addWidget(new QLabel(tr(jointName.c_str())), i+1, 0);
    gridLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                   QSizePolicy::Minimum), i+1, 1);

    JointPIDPosControl *slider = new JointPIDPosControl(jointName,
        gridLayout, this);

    this->pidPosSliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnPIDPosChanged(double, const std::string &)));
    connect(slider, SIGNAL(pChanged(double, const std::string &)),
            this, SLOT(OnPPosGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(iChanged(double, const std::string &)),
            this, SLOT(OnIPosGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(dChanged(double, const std::string &)),
            this, SLOT(OnDPosGainChanged(double, const std::string &)));
  }

  pidPosFrame->setLayout(gridLayout);
  pidPosFrame->layout()->setContentsMargins(4, 0, 0, 0);
  pidPosFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  pidPosScrollArea->setWidget(pidPosFrame);
  pidPosScrollArea->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  pidPosScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
 


  // Create a PID Vel scroll area
  QScrollArea *pidVelScrollArea = new QScrollArea;
  QFrame *pidVelFrame = new QFrame;
  pidVelFrame->setLineWidth(0);
  gridLayout = new QGridLayout;

  gridLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum), 0, 0, 2);
  gridLayout->addWidget(new QLabel("m/s", this), 0, 2);
  gridLayout->addWidget(new QLabel("P Gain", this), 0, 3);
  gridLayout->addWidget(new QLabel("I Gain", this), 0, 4);
  gridLayout->addWidget(new QLabel("D Gain", this), 0, 5);

  for (int i = 0; i < modelMsg.joint_size(); ++i)
  {
    std::string jointName = modelMsg.joint(i).name();
    gridLayout->addWidget(new QLabel(tr(jointName.c_str())), i+1, 0);
    gridLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                   QSizePolicy::Minimum), i+1, 1);

    JointPIDVelControl *slider =
      new JointPIDVelControl(jointName, gridLayout, this);

    this->pidVelSliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnPIDVelChanged(double, const std::string &)));
    connect(slider, SIGNAL(pChanged(double, const std::string &)),
            this, SLOT(OnPVelGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(iChanged(double, const std::string &)),
            this, SLOT(OnIVelGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(dChanged(double, const std::string &)),
            this, SLOT(OnDVelGainChanged(double, const std::string &)));
  }

  pidVelFrame->setLayout(gridLayout);
  pidVelFrame->layout()->setContentsMargins(4, 0, 0, 0);
  pidVelFrame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  pidVelScrollArea->setWidget(pidVelFrame);
  pidVelScrollArea->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  pidVelScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

  QTabWidget *tabWidget = new QTabWidget;
  tabWidget->addTab(scrollArea, tr("Force"));
  tabWidget->addTab(pidPosScrollArea, tr("Position"));
  tabWidget->addTab(pidVelScrollArea, tr("Velocity"));

  // mainLayout->addWidget(scrollArea);

  // Add the the force and pid scroll areas to the tab
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *hboxLayout = new QHBoxLayout;
  std::string title = std::string("Model: ") + _modelName;
  hboxLayout->addWidget(new QLabel(tr(title.c_str())));

  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnReset()));

  hboxLayout->addWidget(resetButton);

  mainLayout->addLayout(hboxLayout);
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
void JointControlWidget::OnReset()
{
  std::map<std::string, JointForceControl*>::iterator iter;

  for (iter = this->sliders.begin(); iter != this->sliders.end(); ++iter)
  {
    msgs::JointCmd msg;
    msg.set_name(iter->first);
    msg.set_reset(true);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnForceChanged(double _value, const std::string &_name)
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
void JointControlWidget::OnPIDPosChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->pidPosSliders.find(_name);
  if (iter != this->pidPosSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_target(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPPosGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->pidPosSliders.find(_name);
  if (iter != this->pidPosSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_p_gain(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnDPosGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->pidPosSliders.find(_name);
  if (iter != this->pidPosSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_d_gain(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnIPosGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->pidPosSliders.find(_name);
  if (iter != this->pidPosSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_i_gain(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::Load(const std::string &/*_modelName*/)
{
}
 
/////////////////////////////////////////////////
void JointControlWidget::OnPIDPosUnitsChanged(int _index)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  for (iter = this->pidPosSliders.begin(); iter != this->pidPosSliders.end(); ++iter)
  {
    if (_index == 0)
      iter->second->SetToRadians();
    else
      iter->second->SetToDegrees();
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPIDVelChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->pidVelSliders.find(_name);
  if (iter != this->pidVelSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_target(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPVelGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->pidVelSliders.find(_name);
  if (iter != this->pidVelSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_p_gain(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnDVelGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->pidVelSliders.find(_name);
  if (iter != this->pidVelSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_d_gain(_value);
    this->jointPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnIVelGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->pidVelSliders.find(_name);
  if (iter != this->pidVelSliders.end())
  {
    msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_i_gain(_value);
    this->jointPub->Publish(msg);
  }
}
