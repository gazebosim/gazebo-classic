/*
 * Copyright 2011 Nate Koenig
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
    QGridLayout *_layout, QWidget *_parent, int _index)
  : QWidget(_parent), name(_name)
{
  this->forceSpin = new QDoubleSpinBox;
  this->forceSpin->setRange(-100.0, 100.0);
  this->forceSpin->setSingleStep(0.001);
  this->forceSpin->setDecimals(3);
  this->forceSpin->setValue(0.000);

  _layout->addWidget(forceSpin, _index, 2);

  connect(this->forceSpin, SIGNAL(valueChanged(double)),
          this, SLOT(OnChanged(double)));
}

/////////////////////////////////////////////////
JointForceControl::~JointForceControl()
{
  this->hide();
}

/////////////////////////////////////////////////
void JointForceControl::Reset()
{
  this->forceSpin->setValue(0.0);
}

/////////////////////////////////////////////////
void JointForceControl::OnChanged(double _value)
{
  emit changed(_value, this->name);
}

/////////////////////////////////////////////////
JointPIDPosControl::JointPIDPosControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent, int _index)
  : QWidget(_parent), name(_name)
{
  this->posSpin = new QDoubleSpinBox;
  this->posSpin->setRange(-360, 360);
  this->posSpin->setSingleStep(0.001);
  this->posSpin->setDecimals(3);
  this->posSpin->setValue(0.000);

  this->pGainSpin = new QDoubleSpinBox;
  this->pGainSpin->setMinimum(0.0);
  this->pGainSpin->setSingleStep(0.01);
  this->pGainSpin->setDecimals(3);
  this->pGainSpin->setValue(1.000);

  this->iGainSpin = new QDoubleSpinBox;
  this->iGainSpin->setMinimum(0.0);
  this->iGainSpin->setSingleStep(0.01);
  this->iGainSpin->setDecimals(3);
  this->iGainSpin->setValue(0.100);

  this->dGainSpin = new QDoubleSpinBox;
  this->dGainSpin->setMinimum(0.0);
  this->dGainSpin->setSingleStep(0.01);
  this->dGainSpin->setDecimals(3);
  this->dGainSpin->setValue(0.010);

  _layout->addWidget(this->posSpin, _index, 2);
  _layout->addWidget(pGainSpin, _index, 3);
  _layout->addWidget(iGainSpin, _index, 4);
  _layout->addWidget(dGainSpin, _index, 5);

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
void JointPIDPosControl::Reset()
{
  this->posSpin->setValue(0.0);
  this->pGainSpin->setValue(1.000);
  this->iGainSpin->setValue(0.100);
  this->dGainSpin->setValue(0.010);
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
    QGridLayout *_layout, QWidget *_parent, int _index)
  : QWidget(_parent), name(_name)
{
  this->posSpin = new QDoubleSpinBox;
  posSpin->setRange(-360, 360);
  posSpin->setSingleStep(0.001);
  posSpin->setDecimals(3);
  posSpin->setValue(0.000);

  this->pGainSpin = new QDoubleSpinBox;
  pGainSpin->setMinimum(0.0);
  pGainSpin->setSingleStep(0.01);
  pGainSpin->setDecimals(3);
  pGainSpin->setValue(1.000);

  this->iGainSpin = new QDoubleSpinBox;
  iGainSpin->setMinimum(0.0);
  iGainSpin->setSingleStep(0.01);
  iGainSpin->setDecimals(3);
  iGainSpin->setValue(0.100);

  this->dGainSpin = new QDoubleSpinBox;
  dGainSpin->setMinimum(0.0);
  dGainSpin->setSingleStep(0.01);
  dGainSpin->setDecimals(3);
  dGainSpin->setValue(0.010);

  _layout->addWidget(posSpin, _index, 2);
  _layout->addWidget(pGainSpin, _index, 3);
  _layout->addWidget(iGainSpin, _index, 4);
  _layout->addWidget(dGainSpin, _index, 5);

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
void JointPIDVelControl::Reset()
{
  this->posSpin->setValue(0.0);
  this->pGainSpin->setValue(1.000);
  this->iGainSpin->setValue(0.100);
  this->dGainSpin->setValue(0.010);
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
void JointControlWidget::SetModelName(const std::string &_modelName)
{
  if (this->jointPub)
    this->jointPub.reset();

  this->modelLabel->setText(QString::fromStdString(
        std::string("Model: ") + _modelName));

  this->jointPub = this->node->Advertise<msgs::JointCmd>(
      std::string("~/") + _modelName + "/joint_cmd");

  this->requestMsg = msgs::CreateRequest("entity_info");
  this->requestMsg->set_data(_modelName);

  msgs::Response response = transport::request("default", *this->requestMsg);

  msgs::Model modelMsg;
  modelMsg.ParseFromString(response.serialized_data());

  this->LayoutForceTab(this->forceGridLayout, modelMsg);

  this->LayoutPositionTab(this->positionGridLayout, modelMsg);

  this->LayoutVelocityTab(this->velocityGridLayout, modelMsg);
}

/////////////////////////////////////////////////
JointControlWidget::JointControlWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("jointControl");

  this->setWindowTitle("Joint Control");
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->tabWidget = new QTabWidget;
  this->tabWidget->setObjectName("embeddedTab");

  // Create the Force control scroll area
  this->forceGridLayout = new QGridLayout;
  this->forceScrollArea = new QScrollArea(this);
  this->forceFrame = new QFrame();
  this->AddScrollTab(this->tabWidget, this->forceGridLayout,
                     this->forceScrollArea, this->forceFrame, tr("Force"));

  // Create a PID Pos scroll area
  this->positionGridLayout = new QGridLayout;
  this->positionScrollArea = new QScrollArea(this);
  this->positionFrame = new QFrame();
  this->AddScrollTab(this->tabWidget, this->positionGridLayout,
                     this->positionScrollArea, this->positionFrame,
                     tr("Position"));

  // Create a PID Vel scroll area
  this->velocityGridLayout = new QGridLayout;
  this->velocityScrollArea = new QScrollArea(this);
  this->velocityFrame = new QFrame();
  this->AddScrollTab(this->tabWidget, this->velocityGridLayout,
                     this->velocityScrollArea, this->velocityFrame,
                     tr("Velocity"));

  // Add the the force and pid scroll areas to the tab
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *hboxLayout = new QHBoxLayout;

  this->modelLabel = new QLabel(tr("Model: "));
  hboxLayout->addWidget(this->modelLabel);

  hboxLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnReset()));

  hboxLayout->addWidget(resetButton);

  mainLayout->addLayout(hboxLayout);
  mainLayout->addWidget(this->tabWidget);
  mainLayout->setContentsMargins(4, 4, 4, 4);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
JointControlWidget::~JointControlWidget()
{
}

/////////////////////////////////////////////////
void JointControlWidget::OnReset()
{
  for (std::map<std::string, JointForceControl*>::iterator iter =
       this->sliders.begin(); iter != this->sliders.end(); ++iter)
  {
    msgs::JointCmd msg;
    msg.set_name(iter->first);
    msg.set_reset(true);
    this->jointPub->Publish(msg);
    iter->second->Reset();
  }

  for (std::map<std::string, JointPIDPosControl*>::iterator iter =
       this->pidPosSliders.begin(); iter != this->pidPosSliders.end(); ++iter)
  {
    iter->second->Reset();
  }

  for (std::map<std::string, JointPIDVelControl*>::iterator iter =
       this->pidVelSliders.begin(); iter != this->pidVelSliders.end(); ++iter)
  {
    iter->second->Reset();
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
void JointControlWidget::OnPIDPosUnitsChanged(int _index)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  for (iter = this->pidPosSliders.begin();
       iter != this->pidPosSliders.end(); ++iter)
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

/////////////////////////////////////////////////
void JointControlWidget::AddScrollTab(QTabWidget *_tabPane,
    QGridLayout *_tabLayout, QScrollArea *_scrollArea, QFrame *_frame,
    const QString &_name)
{
  _tabLayout->setSizeConstraint(QLayout::SetMinimumSize);

  _scrollArea->setLineWidth(0);
  _scrollArea->setFrameShape(QFrame::NoFrame);
  _scrollArea->setFrameShadow(QFrame::Plain);
  _scrollArea->setSizePolicy(QSizePolicy::Maximum,
                             QSizePolicy::Maximum);

  _frame->setObjectName("pidControl");
  _frame->setLineWidth(0);
  _frame->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  _frame->setLayout(_tabLayout);

  _scrollArea->setWidget(_frame);
  _tabPane->addTab(_scrollArea, _name);
}

/////////////////////////////////////////////////
void JointControlWidget::LayoutForceTab(QGridLayout *_tabLayout,
                                        msgs::Model &_modelMsg)
{
  // Remove the old widgets;
  QLayoutItem *wItem;
  while ((wItem = _tabLayout->takeAt(0)) != NULL)
  {
    if (wItem->widget())
      delete wItem->widget();
    delete wItem;
  }
  this->sliders.clear();

  _tabLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum), 0, 0, 2);
  _tabLayout->addWidget(new QLabel("Newton-meter", this), 0, 2);
  for (int i = 0; i < _modelMsg.joint_size(); ++i)
  {
    std::string jointName = _modelMsg.joint(i).name();

    // Get the joint name minus the model name
    int modelNameIndex = jointName.find("::") + 2;
    std::string shortName = jointName.substr(modelNameIndex,
                            jointName.size() - modelNameIndex);

    _tabLayout->addWidget(new QLabel(tr(shortName.c_str())), i+1, 0);
    _tabLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                   QSizePolicy::Minimum), i+1, 1);

    JointForceControl *slider = new JointForceControl(jointName, _tabLayout,
                                                      this, i+1);
    this->sliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnForceChanged(double, const std::string &)));
  }
}

/////////////////////////////////////////////////
void JointControlWidget::LayoutPositionTab(QGridLayout *_tabLayout,
                                           msgs::Model &_modelMsg)
{
  // Remove the old widgets;
  QLayoutItem *wItem;
  while ((wItem = _tabLayout->takeAt(0)) != NULL)
  {
    if (wItem->widget())
      delete wItem->widget();
    delete wItem;
  }
  this->pidPosSliders.clear();

  _tabLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum), 0, 0, 2);
  QComboBox *unitsCombo = new QComboBox;
  unitsCombo->addItem("Radians");
  unitsCombo->addItem("Degrees");
  connect(unitsCombo, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnPIDPosUnitsChanged(int)));

  _tabLayout->addWidget(unitsCombo, 0, 2);
  _tabLayout->addWidget(new QLabel("P Gain", this), 0, 3);
  _tabLayout->addWidget(new QLabel("I Gain", this), 0, 4);
  _tabLayout->addWidget(new QLabel("D Gain", this), 0, 5);

  for (int i = 0; i < _modelMsg.joint_size(); ++i)
  {
    std::string jointName = _modelMsg.joint(i).name();

    // Get the joint name minus the model name
    int modelNameIndex = jointName.find("::") + 2;
    std::string shortName = jointName.substr(modelNameIndex,
                            jointName.size() - modelNameIndex);

    _tabLayout->addWidget(new QLabel(tr(shortName.c_str())), i+1, 0);
    _tabLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                   QSizePolicy::Minimum), i+1, 1);

    JointPIDPosControl *slider = new JointPIDPosControl(jointName,
        _tabLayout, this, i+1);

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
}

/////////////////////////////////////////////////
void JointControlWidget::LayoutVelocityTab(QGridLayout *_tabLayout,
                                           msgs::Model &_modelMsg)
{
  // Remove the old widgets;
  QLayoutItem *wItem;
  while ((wItem = _tabLayout->takeAt(0)) != NULL)
  {
    if (wItem->widget())
      delete wItem->widget();
    delete wItem;
  }
  this->pidVelSliders.clear();

  _tabLayout->addItem(new QSpacerItem(10, 27, QSizePolicy::Expanding,
                                      QSizePolicy::Minimum), 0, 0, 2);

  // Set fixed height for the label to make the tabs stay a consistent size.
  QLabel *label = new QLabel("m/s", this);
  label->setFixedHeight(27);

  _tabLayout->addWidget(label, 0, 2);
  _tabLayout->addWidget(new QLabel("P Gain", this), 0, 3);
  _tabLayout->addWidget(new QLabel("I Gain", this), 0, 4);
  _tabLayout->addWidget(new QLabel("D Gain", this), 0, 5);

  for (int i = 0; i < _modelMsg.joint_size(); ++i)
  {
    std::string jointName = _modelMsg.joint(i).name();

    // Get the joint name minus the model name
    int modelNameIndex = jointName.find("::") + 2;
    std::string shortName = jointName.substr(modelNameIndex,
                            jointName.size() - modelNameIndex);

    _tabLayout->addWidget(new QLabel(tr(shortName.c_str())), i+1, 0);
    _tabLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                                   QSizePolicy::Minimum), i+1, 1);

    JointPIDVelControl *slider =
      new JointPIDVelControl(jointName, _tabLayout, this, i+1);

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
}
