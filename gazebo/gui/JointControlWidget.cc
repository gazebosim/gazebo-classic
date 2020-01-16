/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/algorithm/string.hpp>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/JointControlWidget.hh"
#include "gazebo/gui/JointControlWidgetPrivate.hh"

using namespace gazebo;
using namespace gui;

const QString activeStyle =
"QDoubleSpinBox{background: #404040; color: #ffffff;}";
const QString inactiveStyle =
"QDoubleSpinBox{background: #737373; color: #353535;}";

/////////////////////////////////////////////////
JointForceControl::JointForceControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent, int _index)
  : QWidget(_parent),
    dataPtr(new JointForceControlPrivate())
{
  this->dataPtr->name = _name;
  this->dataPtr->forceSpin = new QDoubleSpinBox;
  this->dataPtr->forceSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->forceSpin->setSingleStep(0.001);
  this->dataPtr->forceSpin->setDecimals(5);
  this->dataPtr->forceSpin->setValue(0.000);

  _layout->addWidget(this->dataPtr->forceSpin, _index, 2);

  connect(this->dataPtr->forceSpin, SIGNAL(valueChanged(double)),
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
  this->SetShowActive(false);
  bool blockSignalsPrev = this->dataPtr->forceSpin->blockSignals(true);
  this->dataPtr->forceSpin->setValue(0.0);
  this->dataPtr->forceSpin->blockSignals(blockSignalsPrev);
}

/////////////////////////////////////////////////
void JointForceControl::SetForce(const double _force)
{
  this->SetShowActive(true);
  this->dataPtr->forceSpin->setValue(_force);
}

/////////////////////////////////////////////////
void JointForceControl::SetShowActive(const bool _active)
{
  this->dataPtr->forceSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
}

/////////////////////////////////////////////////
void JointForceControl::OnChanged(double _value)
{
  this->SetShowActive(true);
  emit changed(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
JointPIDPosControl::JointPIDPosControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent, int _index)
  : QWidget(_parent),
    dataPtr(new JointPIDPosControlPrivate())
{
  this->dataPtr->name = _name;
  this->dataPtr->posSpin = new QDoubleSpinBox;
  this->dataPtr->posSpin->setRange(-ignition::math::MAX_D,
                                    ignition::math::MAX_D);
  this->dataPtr->posSpin->setSingleStep(0.001);
  this->dataPtr->posSpin->setDecimals(5);
  this->dataPtr->posSpin->setValue(0.000);

  this->dataPtr->pGainSpin = new QDoubleSpinBox;
  this->dataPtr->pGainSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->pGainSpin->setSingleStep(0.01);
  this->dataPtr->pGainSpin->setDecimals(5);
  this->dataPtr->pGainSpin->setValue(1.000);

  this->dataPtr->iGainSpin = new QDoubleSpinBox;
  this->dataPtr->iGainSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->iGainSpin->setSingleStep(0.01);
  this->dataPtr->iGainSpin->setDecimals(5);
  this->dataPtr->iGainSpin->setValue(0.100);

  this->dataPtr->dGainSpin = new QDoubleSpinBox;
  this->dataPtr->dGainSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->dGainSpin->setSingleStep(0.01);
  this->dataPtr->dGainSpin->setDecimals(5);
  this->dataPtr->dGainSpin->setValue(0.010);

  _layout->addWidget(this->dataPtr->posSpin, _index, 2);
  _layout->addWidget(this->dataPtr->pGainSpin, _index, 3);
  _layout->addWidget(this->dataPtr->iGainSpin, _index, 4);
  _layout->addWidget(this->dataPtr->dGainSpin, _index, 5);
  this->SetShowActive(false);

  connect(this->dataPtr->posSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
  connect(this->dataPtr->pGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnPChanged(double)));
  connect(this->dataPtr->iGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnIChanged(double)));
  connect(this->dataPtr->dGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnDChanged(double)));

  this->dataPtr->radians = true;
}

/////////////////////////////////////////////////
JointPIDPosControl::~JointPIDPosControl()
{
}

/////////////////////////////////////////////////
void JointPIDPosControl::Reset()
{
  this->SetShowActive(false);
  bool blockSignalsPrev = this->dataPtr->posSpin->blockSignals(true);
  this->dataPtr->posSpin->setValue(0.0);
  this->dataPtr->posSpin->blockSignals(blockSignalsPrev);
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetPositionTarget(const double _target)
{
  this->SetShowActive(true);
  this->dataPtr->posSpin->setValue(_target);
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetPGain(const double _pGain)
{
  this->dataPtr->pGainSpin->setValue(_pGain);
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetIGain(const double _iGain)
{
  this->dataPtr->iGainSpin->setValue(_iGain);
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetDGain(const double _dGain)
{
  this->dataPtr->dGainSpin->setValue(_dGain);
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetShowActive(const bool _active)
{
  this->dataPtr->posSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
  this->dataPtr->pGainSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
  this->dataPtr->iGainSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
  this->dataPtr->dGainSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetToDegrees()
{
  if (this->dataPtr->radians)
  {
    this->dataPtr->radians = false;
    this->dataPtr->posSpin->setValue(IGN_RTOD(this->dataPtr->posSpin->value()));
  }
}

/////////////////////////////////////////////////
void JointPIDPosControl::SetToRadians()
{
  if (!this->dataPtr->radians)
  {
    this->dataPtr->radians = true;
    this->dataPtr->posSpin->setValue(IGN_DTOR(this->dataPtr->posSpin->value()));
  }
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnChanged(double _value)
{
  this->SetShowActive(true);
  if (this->dataPtr->radians)
    emit changed(_value, this->dataPtr->name);
  else
    emit changed(IGN_DTOR(_value), this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnPChanged(double _value)
{
  emit pChanged(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnIChanged(double _value)
{
  emit iChanged(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointPIDPosControl::OnDChanged(double _value)
{
  emit dChanged(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
JointPIDVelControl::JointPIDVelControl(const std::string &_name,
    QGridLayout *_layout, QWidget *_parent, int _index)
  : QWidget(_parent),
    dataPtr(new JointPIDVelControlPrivate())
{
  this->dataPtr->name = _name;
  this->dataPtr->posSpin = new QDoubleSpinBox;
  this->dataPtr->posSpin->setRange(-ignition::math::MAX_D,
                                    ignition::math::MAX_D);
  this->dataPtr->posSpin->setSingleStep(0.001);
  this->dataPtr->posSpin->setDecimals(5);
  this->dataPtr->posSpin->setValue(0.000);

  this->dataPtr->pGainSpin = new QDoubleSpinBox;
  this->dataPtr->pGainSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->pGainSpin->setSingleStep(0.01);
  this->dataPtr->pGainSpin->setDecimals(5);
  this->dataPtr->pGainSpin->setValue(1.000);

  this->dataPtr->iGainSpin = new QDoubleSpinBox;
  this->dataPtr->iGainSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->iGainSpin->setSingleStep(0.01);
  this->dataPtr->iGainSpin->setDecimals(5);
  this->dataPtr->iGainSpin->setValue(0.100);

  this->dataPtr->dGainSpin = new QDoubleSpinBox;
  this->dataPtr->dGainSpin->setRange(-ignition::math::MAX_D,
                                      ignition::math::MAX_D);
  this->dataPtr->dGainSpin->setSingleStep(0.01);
  this->dataPtr->dGainSpin->setDecimals(5);
  this->dataPtr->dGainSpin->setValue(0.010);

  _layout->addWidget(this->dataPtr->posSpin, _index, 2);
  _layout->addWidget(this->dataPtr->pGainSpin, _index, 3);
  _layout->addWidget(this->dataPtr->iGainSpin, _index, 4);
  _layout->addWidget(this->dataPtr->dGainSpin, _index, 5);
  this->SetShowActive(false);

  connect(this->dataPtr->posSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnChanged(double)));
  connect(this->dataPtr->pGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnPChanged(double)));
  connect(this->dataPtr->iGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnIChanged(double)));
  connect(this->dataPtr->dGainSpin, SIGNAL(valueChanged(double)),
        this, SLOT(OnDChanged(double)));
}

/////////////////////////////////////////////////
JointPIDVelControl::~JointPIDVelControl()
{
}

/////////////////////////////////////////////////
void JointPIDVelControl::Reset()
{
  this->SetShowActive(false);
  bool blockSignalsPrev = this->dataPtr->posSpin->blockSignals(true);
  this->dataPtr->posSpin->setValue(0.0);
  this->dataPtr->posSpin->blockSignals(blockSignalsPrev);
}

/////////////////////////////////////////////////
void JointPIDVelControl::SetVelocityTarget(const double _target)
{
  this->SetShowActive(true);
  this->dataPtr->posSpin->setValue(_target);
}

/////////////////////////////////////////////////
void JointPIDVelControl::SetPGain(const double _pGain)
{
  this->dataPtr->pGainSpin->setValue(_pGain);
}

/////////////////////////////////////////////////
void JointPIDVelControl::SetIGain(const double _iGain)
{
  this->dataPtr->iGainSpin->setValue(_iGain);
}

/////////////////////////////////////////////////
void JointPIDVelControl::SetDGain(const double _dGain)
{
  this->dataPtr->dGainSpin->setValue(_dGain);
}

/////////////////////////////////////////////////
void JointPIDVelControl::SetShowActive(const bool _active)
{
  this->dataPtr->posSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
  this->dataPtr->pGainSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
  this->dataPtr->iGainSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
  this->dataPtr->dGainSpin->setStyleSheet(
      _active ? activeStyle : inactiveStyle);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnChanged(double _value)
{
  this->SetShowActive(true);
  emit changed(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnPChanged(double _value)
{
  emit pChanged(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnIChanged(double _value)
{
  emit iChanged(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointPIDVelControl::OnDChanged(double _value)
{
  emit dChanged(_value, this->dataPtr->name);
}

/////////////////////////////////////////////////
void JointControlWidget::SetModelName(const std::string &_modelName)
{
  // The selected model has not changed.
  if (_modelName == this->dataPtr->modelName)
  {
    return;
  }

  msgs::Model modelMsg;

  this->dataPtr->modelLabel->setText(
      QString::fromStdString(std::string("Model: ")));

  // Only request info if the model has a name.
  if (!_modelName.empty())
  {
    std::string topic = "/" + _modelName + "/joint_cmd";
    boost::replace_all(topic, "::", "/");
    this->dataPtr->jointPub =
        this->dataPtr->node.Advertise<ignition::msgs::JointCmd>(topic);
    if (!this->dataPtr->jointPub)
    {
      gzerr << "Error advertising topic [" << topic << "]\n";
    }

    boost::shared_ptr<msgs::Response> response = transport::request(
        gui::get_world(), "entity_info", _modelName);

    if (response->response() != "error" &&
        response->type() == modelMsg.GetTypeName())
    {
      modelMsg.ParseFromString(response->serialized_data());
    }
  }

  this->dataPtr->modelLabel->setText(QString::fromStdString(
      std::string("Model: ") + modelMsg.name()));

  this->LayoutForceTab(modelMsg);

  this->LayoutPositionTab(modelMsg);

  this->LayoutVelocityTab(modelMsg);

  if (!_modelName.empty())
  {
    // Get joint controller parameters and use them to populate the sliders.
    ignition::msgs::StringMsg req;
    std::string service = "/" + _modelName + "/joint_cmd_req";
    boost::replace_all(service, "::", "/");

    std::function<void(const ignition::msgs::JointCmd &, const bool)> callback =
        std::bind(&JointControlWidget::ResponseCallback, this, _modelName,
        std::placeholders::_1, std::placeholders::_2);

    // Force, PID vel, and PID pos controls should all have the same joints, so
    // we can request info for all joints by looping over the force controls.
    for (auto &slider : this->dataPtr->sliders)
    {
      req.set_data(slider.first);
      this->dataPtr->node.Request(service, req, callback);
    }
  }

  this->dataPtr->modelName = _modelName;
}

/////////////////////////////////////////////////
JointControlWidget::JointControlWidget(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new JointControlWidgetPrivate())
{
  this->setObjectName("jointControl");

  this->setWindowTitle("Joint Control");

  this->dataPtr->tabWidget = new QTabWidget;
  this->dataPtr->tabWidget->setObjectName("embeddedTab");

  // Create the Force control scroll area
  this->dataPtr->forceGridLayout = new QGridLayout;
  this->AddScrollTab(
      this->dataPtr->tabWidget, this->dataPtr->forceGridLayout, tr("Force"));

  // Create a PID Pos scroll area
  this->dataPtr->positionGridLayout = new QGridLayout;
  this->AddScrollTab(this->dataPtr->tabWidget,
      this->dataPtr->positionGridLayout, tr("Position"));

  // Create a PID Vel scroll area
  this->dataPtr->velocityGridLayout = new QGridLayout;
  this->AddScrollTab(this->dataPtr->tabWidget,
      this->dataPtr->velocityGridLayout, tr("Velocity"));

  // Add the the force and pid scroll areas to the tab
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *hboxLayout = new QHBoxLayout;

  this->dataPtr->modelLabel = new QLabel(tr("Model: "));
  hboxLayout->addWidget(this->dataPtr->modelLabel);

  hboxLayout->addItem(new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Minimum));
  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(OnReset()));

  hboxLayout->addWidget(resetButton);

  mainLayout->addLayout(hboxLayout);
  mainLayout->addWidget(this->dataPtr->tabWidget);
  mainLayout->setContentsMargins(4, 4, 4, 4);

  this->setLayout(mainLayout);

  connect(this, SIGNAL(repReceived(const std::string &,
      const ignition::msgs::JointCmd &)),
      this, SLOT(OnResponse(const std::string &,
      const ignition::msgs::JointCmd &)),
      Qt::QueuedConnection);
}

/////////////////////////////////////////////////
JointControlWidget::~JointControlWidget()
{
}

/////////////////////////////////////////////////
void JointControlWidget::ResponseCallback(const std::string &_modelName,
    const ignition::msgs::JointCmd &_rep, const bool _result)
{
  if (_result)
  {
    // Signal itself to make the update. This lets Qt schedule the update
    // in its own thread, which prevents some synchronization issues.
    emit repReceived(_modelName, _rep);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnResponse(const std::string &_modelName,
    const ignition::msgs::JointCmd &_rep)
{
  if (_modelName == this->dataPtr->modelName)
  {
    if (_rep.has_force())
    {
      auto slider = this->dataPtr->sliders[_rep.name()];
      GZ_ASSERT(slider, "Joint force controller is null");
      slider->SetForce(_rep.force());
    }
    if (_rep.has_position())
    {
      auto slider = this->dataPtr->pidPosSliders[_rep.name()];
      GZ_ASSERT(slider, "Joint PID position controller is null");
      if (_rep.position().has_target())
      {
        slider->SetPositionTarget(_rep.position().target());
      }
      if (_rep.position().has_p_gain())
      {
        slider->SetPGain(_rep.position().p_gain());
      }
      if (_rep.position().has_i_gain())
      {
        slider->SetIGain(_rep.position().i_gain());
      }
      if (_rep.position().has_d_gain())
      {
        slider->SetDGain(_rep.position().d_gain());
      }
    }
    if (_rep.has_velocity())
    {
      auto slider = this->dataPtr->pidVelSliders[_rep.name()];
      GZ_ASSERT(slider, "Joint PID velocity controller is null");
      if (_rep.velocity().has_target())
      {
        slider->SetVelocityTarget(_rep.velocity().target());
      }
      if (_rep.velocity().has_p_gain())
      {
        slider->SetPGain(_rep.velocity().p_gain());
      }
      if (_rep.velocity().has_i_gain())
      {
        slider->SetIGain(_rep.velocity().i_gain());
      }
      if (_rep.velocity().has_d_gain())
      {
        slider->SetDGain(_rep.velocity().d_gain());
      }
    }
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnReset()
{
  for (std::map<std::string, JointForceControl*>::iterator iter =
       this->dataPtr->sliders.begin();
       iter != this->dataPtr->sliders.end(); ++iter)
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(iter->first);
    msg.set_reset(true);
    this->dataPtr->jointPub.Publish(msg);

    iter->second->Reset();
  }

  for (std::map<std::string, JointPIDPosControl*>::iterator iter =
       this->dataPtr->pidPosSliders.begin();
       iter != this->dataPtr->pidPosSliders.end(); ++iter)
  {
    iter->second->Reset();
  }

  for (std::map<std::string, JointPIDVelControl*>::iterator iter =
       this->dataPtr->pidVelSliders.begin();
       iter != this->dataPtr->pidVelSliders.end(); ++iter)
  {
    iter->second->Reset();
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnForceChanged(double _value, const std::string &_name)
{
  std::map<std::string, JointForceControl*>::iterator iter;
  iter = this->dataPtr->sliders.find(_name);
  if (iter != this->dataPtr->sliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.set_force(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPIDPosChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->dataPtr->pidPosSliders.find(_name);
  if (iter != this->dataPtr->pidPosSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_target(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPPosGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->dataPtr->pidPosSliders.find(_name);
  if (iter != this->dataPtr->pidPosSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_p_gain(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnDPosGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->dataPtr->pidPosSliders.find(_name);
  if (iter != this->dataPtr->pidPosSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_d_gain(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnIPosGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  iter = this->dataPtr->pidPosSliders.find(_name);
  if (iter != this->dataPtr->pidPosSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_position()->set_i_gain(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPIDPosUnitsChanged(int _index)
{
  std::map<std::string, JointPIDPosControl*>::iterator iter;
  for (iter = this->dataPtr->pidPosSliders.begin();
       iter != this->dataPtr->pidPosSliders.end(); ++iter)
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
  iter = this->dataPtr->pidVelSliders.find(_name);
  if (iter != this->dataPtr->pidVelSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_target(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnPVelGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->dataPtr->pidVelSliders.find(_name);
  if (iter != this->dataPtr->pidVelSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_p_gain(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnDVelGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->dataPtr->pidVelSliders.find(_name);
  if (iter != this->dataPtr->pidVelSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_d_gain(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::OnIVelGainChanged(double _value,
    const std::string &_name)
{
  std::map<std::string, JointPIDVelControl*>::iterator iter;
  iter = this->dataPtr->pidVelSliders.find(_name);
  if (iter != this->dataPtr->pidVelSliders.end())
  {
    ignition::msgs::JointCmd msg;
    msg.set_name(_name);
    msg.mutable_velocity()->set_i_gain(_value);
    this->dataPtr->jointPub.Publish(msg);
  }
}

/////////////////////////////////////////////////
void JointControlWidget::AddScrollTab(QTabWidget *_tabPane,
    QGridLayout *_tabLayout, const QString &_name)
{
  _tabLayout->setSizeConstraint(QLayout::SetMinimumSize);

  QScrollArea *scrollArea = new QScrollArea(this);
  scrollArea->setLineWidth(0);
  scrollArea->setFrameShape(QFrame::NoFrame);
  scrollArea->setFrameShadow(QFrame::Plain);
  scrollArea->setSizePolicy(QSizePolicy::Minimum,
                            QSizePolicy::Minimum);

  // Make the scroll area automatically resize to the containing widget.
  scrollArea->setWidgetResizable(true);

  QFrame *frame = new QFrame(scrollArea);
  frame->setObjectName("pidControl");
  frame->setLineWidth(0);
  frame->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
  frame->setLayout(_tabLayout);

  scrollArea->setWidget(frame);
  _tabPane->addTab(scrollArea, _name);
}

/////////////////////////////////////////////////
void JointControlWidget::LayoutForceTab(msgs::Model &_modelMsg)
{
  // Remove the old widgets;
  QLayoutItem *wItem;
  while ((wItem = this->dataPtr->forceGridLayout->takeAt(0)) != NULL)
  {
    if (wItem->widget())
      delete wItem->widget();
    delete wItem;
  }
  this->dataPtr->sliders.clear();

  // Don't add any widget if there are no joints
  if (_modelMsg.joint_size() == 0)
    return;

  this->dataPtr->forceGridLayout->addItem(new QSpacerItem(
      10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 0, 2);
  this->dataPtr->forceGridLayout->addWidget(new QLabel(
      "Newton-meter", this), 0, 2);

  int i = 0;
  for (; i < _modelMsg.joint_size(); ++i)
  {
    std::string jointName = _modelMsg.joint(i).name();

    // Get the joint name minus the model name
    int modelNameIndex = jointName.find("::") + 2;
    std::string shortName = jointName.substr(modelNameIndex,
                            jointName.size() - modelNameIndex);

    this->dataPtr->forceGridLayout->addWidget(new QLabel(
        tr(shortName.c_str())), i+1, 0);
    this->dataPtr->forceGridLayout->addItem(new QSpacerItem(
        10, 20, QSizePolicy::Expanding, QSizePolicy::Minimum), i+1, 1);

    JointForceControl *slider = new JointForceControl(jointName,
        this->dataPtr->forceGridLayout, this, i+1);

    this->dataPtr->sliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnForceChanged(double, const std::string &)));
  }

  // Add a space at the bottom of the grid layout to consume extra space.
  this->dataPtr->forceGridLayout->addItem(new QSpacerItem(
      10, 20, QSizePolicy::Expanding, QSizePolicy::Expanding), i+1, 0);
  this->dataPtr->forceGridLayout->setRowStretch(i+1, 2);
}

/////////////////////////////////////////////////
void JointControlWidget::LayoutPositionTab(msgs::Model &_modelMsg)
{
  // Remove the old widgets;
  QLayoutItem *wItem;
  while ((wItem = this->dataPtr->positionGridLayout->takeAt(0)) != NULL)
  {
    if (wItem->widget())
      delete wItem->widget();
    delete wItem;
  }
  this->dataPtr->pidPosSliders.clear();

  // Don't add any widget if there are no joints
  if (_modelMsg.joint_size() == 0)
    return;

  this->dataPtr->positionGridLayout->addItem(new QSpacerItem(10, 20,
        QSizePolicy::Expanding, QSizePolicy::Minimum), 0, 0, 2);

  QComboBox *unitsCombo = new QComboBox;
  unitsCombo->addItem("Radians");
  unitsCombo->addItem("Degrees");
  connect(unitsCombo, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnPIDPosUnitsChanged(int)));

  this->dataPtr->positionGridLayout->addWidget(unitsCombo, 0, 2);
  this->dataPtr->positionGridLayout->addWidget(new QLabel(
      "P Gain", this), 0, 3);
  this->dataPtr->positionGridLayout->addWidget(new QLabel(
      "I Gain", this), 0, 4);
  this->dataPtr->positionGridLayout->addWidget(new QLabel(
      "D Gain", this), 0, 5);

  int i = 0;
  for (; i < _modelMsg.joint_size(); ++i)
  {
    std::string jointName = _modelMsg.joint(i).name();

    // Get the joint name minus the model name
    int modelNameIndex = jointName.find("::") + 2;
    std::string shortName = jointName.substr(modelNameIndex,
                            jointName.size() - modelNameIndex);

    this->dataPtr->positionGridLayout->addWidget(
        new QLabel(tr(shortName.c_str())), i+1, 0);
    this->dataPtr->positionGridLayout->addItem(
        new QSpacerItem(10, 20, QSizePolicy::Expanding,
                        QSizePolicy::Minimum), i+1, 1);

    JointPIDPosControl *slider = new JointPIDPosControl(jointName,
        this->dataPtr->positionGridLayout, this, i+1);

    this->dataPtr->pidPosSliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnPIDPosChanged(double, const std::string &)));
    connect(slider, SIGNAL(pChanged(double, const std::string &)),
            this, SLOT(OnPPosGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(iChanged(double, const std::string &)),
            this, SLOT(OnIPosGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(dChanged(double, const std::string &)),
            this, SLOT(OnDPosGainChanged(double, const std::string &)));
  }

  // Add a space at the bottom of the grid layout to consume extra space.
  this->dataPtr->positionGridLayout->addItem(
      new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Expanding), i+1, 0);
  this->dataPtr->positionGridLayout->setRowStretch(i+1, 2);
}

/////////////////////////////////////////////////
void JointControlWidget::LayoutVelocityTab(msgs::Model &_modelMsg)
{
  // Remove the old widgets;
  QLayoutItem *wItem;
  while ((wItem = this->dataPtr->velocityGridLayout->takeAt(0)) != NULL)
  {
    if (wItem->widget())
      delete wItem->widget();
    delete wItem;
  }
  this->dataPtr->pidVelSliders.clear();

  // Don't add any widget if there are no joints
  if (_modelMsg.joint_size() == 0)
    return;

  this->dataPtr->velocityGridLayout->addItem(
      new QSpacerItem(10, 27, QSizePolicy::Expanding,
                      QSizePolicy::Minimum), 0, 0, 2);

  // Set fixed height for the label to make the tabs stay a consistent size.
  QLabel *label = new QLabel("m/s", this);
  label->setFixedHeight(27);

  this->dataPtr->velocityGridLayout->addWidget(label, 0, 2);
  this->dataPtr->velocityGridLayout->addWidget(new QLabel(
      "P Gain", this), 0, 3);
  this->dataPtr->velocityGridLayout->addWidget(new QLabel(
      "I Gain", this), 0, 4);
  this->dataPtr->velocityGridLayout->addWidget(new QLabel(
      "D Gain", this), 0, 5);

  int i = 0;
  for (; i < _modelMsg.joint_size(); ++i)
  {
    std::string jointName = _modelMsg.joint(i).name();

    // Get the joint name minus the model name
    int modelNameIndex = jointName.find("::") + 2;
    std::string shortName = jointName.substr(modelNameIndex,
                            jointName.size() - modelNameIndex);

    this->dataPtr->velocityGridLayout->addWidget(
        new QLabel(tr(shortName.c_str())), i+1, 0);
    this->dataPtr->velocityGridLayout->addItem(
        new QSpacerItem(10, 20, QSizePolicy::Expanding,
                        QSizePolicy::Minimum), i+1, 1);

    JointPIDVelControl *slider = new JointPIDVelControl(
        jointName, this->dataPtr->velocityGridLayout, this, i+1);

    this->dataPtr->pidVelSliders[jointName] = slider;
    connect(slider, SIGNAL(changed(double, const std::string &)),
            this, SLOT(OnPIDVelChanged(double, const std::string &)));
    connect(slider, SIGNAL(pChanged(double, const std::string &)),
            this, SLOT(OnPVelGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(iChanged(double, const std::string &)),
            this, SLOT(OnIVelGainChanged(double, const std::string &)));
    connect(slider, SIGNAL(dChanged(double, const std::string &)),
            this, SLOT(OnDVelGainChanged(double, const std::string &)));
  }

  // Add a space at the bottom of the grid layout to consume extra space.
  this->dataPtr->velocityGridLayout->addItem(
      new QSpacerItem(10, 20, QSizePolicy::Expanding,
                      QSizePolicy::Expanding), i+1, 0);
  this->dataPtr->velocityGridLayout->setRowStretch(i+1, 2);
}
