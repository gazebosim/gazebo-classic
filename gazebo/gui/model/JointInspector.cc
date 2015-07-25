/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/JointInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointInspector::JointInspector(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("JointInspectorDialog");
  this->setWindowTitle(tr("Joint Inspector"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  QVBoxLayout *generalLayout = new QVBoxLayout;

  this->configWidget = new ConfigWidget;
  msgs::Joint jointMsg;
  configWidget->Load(&jointMsg);

  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(configWidget);
  scrollArea->setWidgetResizable(true);

  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // fill them with SDF default values
  sdf::ElementPtr jointElem = msgs::JointToSDF(jointMsg);
  sdf::ElementPtr axisElem = jointElem->GetElement("axis");
  sdf::ElementPtr axisLimitElem = axisElem->GetElement("limit");
  sdf::ElementPtr odeElem = jointElem->GetElement("physics")->GetElement("ode");
  for (unsigned int i = 0; i < 2u; ++i)
  {
    std::stringstream axis;
    axis << "axis" << i+1;
    std::string axisStr = axis.str();
    this->configWidget->SetVector3WidgetValue(axisStr + "::xyz",
        axisElem->Get<math::Vector3>("xyz"));
    this->configWidget->SetDoubleWidgetValue(axisStr + "::limit_lower",
        axisLimitElem->Get<double>("lower"));
    this->configWidget->SetDoubleWidgetValue(axisStr + "::limit_upper",
        axisLimitElem->Get<double>("upper"));
    this->configWidget->SetDoubleWidgetValue(axisStr + "::limit_effort",
        axisLimitElem->Get<double>("effort"));
    this->configWidget->SetDoubleWidgetValue(axisStr + "::limit_velocity",
        axisLimitElem->Get<double>("velocity"));
    this->configWidget->SetDoubleWidgetValue(axisStr + "::damping",
        axisElem->GetElement("dynamics")->Get<double>("damping"));
    this->configWidget->SetDoubleWidgetValue(axisStr + "::friction",
        axisElem->GetElement("dynamics")->Get<double>("friction"));
    this->configWidget->SetBoolWidgetValue(axisStr + "::parent_model_frame",
        axisElem->Get<bool>("use_parent_model_frame"));
  }

  this->configWidget->SetDoubleWidgetValue("cfm",
      odeElem->Get<double>("cfm"));
  this->configWidget->SetDoubleWidgetValue("bounce",
      odeElem->Get<double>("bounce"));
  this->configWidget->SetDoubleWidgetValue("velocity",
      odeElem->Get<double>("velocity"));
  this->configWidget->SetDoubleWidgetValue("fudge_factor",
      odeElem->Get<double>("fudge_factor"));
  this->configWidget->SetDoubleWidgetValue("limit_cfm",
      odeElem->GetElement("limit")->Get<double>("cfm"));
  this->configWidget->SetDoubleWidgetValue("limit_erp",
      odeElem->GetElement("limit")->Get<double>("erp"));
  this->configWidget->SetDoubleWidgetValue("suspension_cfm",
      odeElem->GetElement("suspension")->Get<double>("cfm"));
  this->configWidget->SetDoubleWidgetValue("suspension_erp",
      odeElem->GetElement("suspension")->Get<double>("erp"));

  // joint type specific properties.
  this->configWidget->SetStringWidgetValue("gearbox::gearbox_reference_body",
      jointElem->HasElement("gearbox_reference_body") ?
      jointElem->Get<std::string>("gearbox_reference_body") : "");
  this->configWidget->SetDoubleWidgetValue("gearbox::gearbox_ratio",
      jointElem->Get<double>("gearbox_ratio"));
  this->configWidget->SetDoubleWidgetValue("screw::thread_pitch",
      jointElem->Get<double>("thread_pitch"));

  this->configWidget->SetWidgetVisible("id", false);
  this->configWidget->SetWidgetVisible("parent_id", false);
  this->configWidget->SetWidgetVisible("child_id", false);

  this->configWidget->SetWidgetReadOnly("id", true);
  this->configWidget->SetWidgetReadOnly("parent_id", true);
  this->configWidget->SetWidgetReadOnly("child_id", true);
  this->configWidget->SetWidgetReadOnly("parent", true);
  this->configWidget->SetWidgetReadOnly("child", true);

  QObject::connect(this->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)), this,
      SLOT(OnJointTypeChanged(const QString &, const QString &)));

  this->OnJointTypeChanged("type",
      tr(msgs::Joint_Type_Name(jointMsg.type()).c_str()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *applyButton = new QPushButton(tr("Apply"));
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));
  QPushButton *OKButton = new QPushButton(tr("OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
JointInspector::~JointInspector()
{
}

/////////////////////////////////////////////////
void JointInspector::Update(ConstJointPtr _jointMsg)
{
  this->configWidget->UpdateFromMsg(_jointMsg.get());
}

/////////////////////////////////////////////////
void JointInspector::SetPose(const math::Pose &_pose)
{
  this->configWidget->SetPoseWidgetValue("pose", _pose);
}

/////////////////////////////////////////////////
msgs::Joint *JointInspector::GetData() const
{
  return dynamic_cast<msgs::Joint *>(this->configWidget->GetMsg());
}

/////////////////////////////////////////////////
void JointInspector::OnJointTypeChanged(const QString &/*_name*/,
    const QString &_value)
{
  std::string valueStr = _value.toLower().toStdString();
  unsigned int axisCount = JointMaker::GetJointAxisCount(
      JointMaker::ConvertJointType(valueStr));

  for (unsigned int i = 0; i < axisCount; ++i)
  {
    std::stringstream axis;
    axis << "axis" << i+1;
    std::string axisStr = axis.str();

    this->configWidget->SetWidgetVisible(axisStr, true);
    this->configWidget->SetWidgetReadOnly(axisStr, false);
    this->configWidget->UpdateFromMsg(this->configWidget->GetMsg());
  }

  for (unsigned int i = axisCount; i < 2u; ++i)
  {
    std::stringstream axis;
    axis << "axis" << i+1;
    std::string axisStr = axis.str();

    this->configWidget->SetWidgetVisible(axisStr, false);
    this->configWidget->SetWidgetReadOnly(axisStr, true);
    this->configWidget->UpdateFromMsg(this->configWidget->GetMsg());
  }

  // toggle field visibility according to joint type.
  bool isGearbox = valueStr == "gearbox";
  bool isScrew = valueStr == "screw";
  this->configWidget->SetWidgetVisible("gearbox", isGearbox);
  this->configWidget->SetWidgetReadOnly("gearbox", !isGearbox);
  this->configWidget->SetWidgetVisible("screw", isScrew);
  this->configWidget->SetWidgetReadOnly("screw", !isScrew);
}

/////////////////////////////////////////////////
void JointInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void JointInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnOK()
{
  emit Applied();
  this->accept();
}

/////////////////////////////////////////////////
void JointInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}
