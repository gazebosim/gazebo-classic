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
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointInspector::JointInspector(JointMaker *_jointMaker, QWidget *_parent)
    : QDialog(_parent), jointMaker(_jointMaker)
{
  this->setObjectName("JointInspectorDialog");
  this->setWindowTitle(tr("Joint Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Style sheets
  this->normalStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::bgColors[0] + ";\
          color: #4c4c4c;\
        }\
        QLabel\
        {\
          color: #d0d0d0;\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::widgetColors[0] +
        "}";

  this->warningStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::bgColors[0] + ";\
          color: " + ConfigWidget::redColor + ";\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::widgetColors[0] +
        "}";

  // ConfigWidget
  this->configWidget = new ConfigWidget;
  msgs::Joint jointMsg;
  this->configWidget->Load(&jointMsg);

  // Fill with SDF default values
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

  // Hide fields
  this->configWidget->SetWidgetVisible("id", false);
  this->configWidget->SetWidgetVisible("parent_id", false);
  this->configWidget->SetWidgetVisible("child_id", false);
  this->configWidget->SetWidgetVisible("parent", false);
  this->configWidget->SetWidgetVisible("child", false);

  this->configWidget->SetWidgetReadOnly("id", true);
  this->configWidget->SetWidgetReadOnly("parent_id", true);
  this->configWidget->SetWidgetReadOnly("child_id", true);
  this->configWidget->SetWidgetReadOnly("parent", true);
  this->configWidget->SetWidgetReadOnly("child", true);

  // Get name widget
  this->nameWidget = this->configWidget->ConfigChildWidgetByName("name");
  if (!this->nameWidget)
    gzerr << "Name widget not found" << std::endl;

  // Custom parent / child widgets
  // Parent
  std::vector<std::string> links;
  this->parentLinkWidget =
      this->configWidget->CreateEnumWidget("parent", links, 0);
  this->parentLinkWidget->setStyleSheet(this->normalStyleSheet);
  this->configWidget->AddConfigChildWidget("parentCombo",
      this->parentLinkWidget);

  // Child
  this->childLinkWidget =
      this->configWidget->CreateEnumWidget("child", links, 0);
  this->childLinkWidget->setStyleSheet(this->normalStyleSheet);
  this->configWidget->AddConfigChildWidget("childCombo", this->childLinkWidget);

  // Swap button
  QToolButton *swapButton = new QToolButton();
  swapButton->setText("Swap");
  swapButton->setMinimumWidth(60);
  swapButton->setStyleSheet(
      "QToolButton\
      {\
        background-color: " + ConfigWidget::bgColors[0] +
      "}");
  connect(swapButton, SIGNAL(clicked()), this, SLOT(OnSwap()));

  // Links layout
  QGridLayout *linksLayout = new QGridLayout();
  linksLayout->setContentsMargins(0, 0, 0, 0);
  linksLayout->addWidget(this->parentLinkWidget, 0, 0);
  linksLayout->addWidget(this->childLinkWidget, 1, 0);
  linksLayout->addWidget(swapButton, 0, 1, 2, 1);

  // Insert on the top of config widget's layout
  this->configWidget->InsertLayout(linksLayout, 0);

  // Connect all enum value changes, which includes type, parent and child
  QObject::connect(this->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)), this,
      SLOT(OnEnumChanged(const QString &, const QString &)));

  // Connect pose value changes, for joint pose
  QObject::connect(this->configWidget,
      SIGNAL(PoseValueChanged(const QString &,
      const ignition::math::Pose3d &)), this,
      SLOT(OnPoseChanged(const QString &,
      const ignition::math::Pose3d &)));

  // Connect vector value changes, for axes
  QObject::connect(this->configWidget,
      SIGNAL(Vector3dValueChanged(const QString &,
      const ignition::math::Vector3d &)), this,
      SLOT(OnVector3dChanged(const QString &,
      const ignition::math::Vector3d &)));

  // Connect string value changes, for name
  QObject::connect(this->configWidget,
      SIGNAL(StringValueChanged(const QString &,
      const std::string &)), this,
      SLOT(OnStringChanged(const QString &,
      const std::string &)));

  // Set initial joint type
  this->OnJointTypeChanged(tr(msgs::Joint_Type_Name(jointMsg.type()).c_str()));

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->configWidget);
  scrollArea->setWidgetResizable(true);

  // General layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Buttons
  QToolButton *removeButton = new QToolButton(this);
  removeButton->setFixedSize(QSize(30, 30));
  removeButton->setToolTip("Remove joint");
  removeButton->setIcon(QPixmap(":/images/trashcan.png"));
  removeButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  removeButton->setIconSize(QSize(16, 16));
  removeButton->setCheckable(false);
  connect(removeButton, SIGNAL(clicked()), this, SLOT(OnRemove()));

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  this->okButton = new QPushButton(tr("OK"));
  this->okButton->setEnabled(true);
  this->okButton->setDefault(true);
  connect(this->okButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(removeButton);
  buttonsLayout->addStretch(5);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(this->okButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);

  this->setLayout(mainLayout);

  // Qt signal / slot connections
  connect(this->jointMaker, SIGNAL(EmitLinkRemoved(std::string)), this,
      SLOT(OnLinkRemoved(std::string)));
  connect(this->jointMaker, SIGNAL(EmitLinkInserted(std::string)), this,
      SLOT(OnLinkInserted(std::string)));
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
  std::string currentParent =
      this->configWidget->GetEnumWidgetValue("parentCombo");

  std::string currentChild =
      this->configWidget->GetEnumWidgetValue("childCombo");

  if (currentParent == currentChild)
  {
    gzerr << "Parent link equal to child link - not updating joint."
        << std::endl;
    return NULL;
  }

  // Get updated message from widget
  msgs::Joint *msg = dynamic_cast<msgs::Joint *>(this->configWidget->GetMsg());

  // Use parent / child from our custom widget
  msg->set_parent(currentParent);
  msg->set_child(currentChild);

  return msg;
}

/////////////////////////////////////////////////
void JointInspector::OnEnumChanged(const QString &_name,
    const QString &_value)
{
  if (_name == "type")
    this->OnJointTypeChanged(_value);
  else if (_name == "parentCombo" || _name == "childCombo")
    this->OnLinksChanged(_value);
}

/////////////////////////////////////////////////
void JointInspector::OnPoseChanged(const QString &/*_name*/,
    const ignition::math::Pose3d &/*_pose*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnVector3dChanged(const QString &/*_name*/,
    const ignition::math::Vector3d &/*_vec*/)
{
  emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnStringChanged(const QString &_name,
    const std::string &_str)
{
  // Handle joint name
  if (_name != "name")
    return;

  this->validJointName = !_str.empty();

  // Warning if joint name is invalid
  if (!this->validJointName)
  {
    this->nameWidget->setStyleSheet(this->warningStyleSheet);
  }
  else
  {
    this->nameWidget->setStyleSheet(this->normalStyleSheet);
  }

  this->UpdateValid();
}

/////////////////////////////////////////////////
void JointInspector::OnJointTypeChanged(const QString &_value)
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

  emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnLinksChanged(const QString &/*_linkName*/)
{
  std::string currentParent =
      this->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->configWidget->GetEnumWidgetValue("childCombo");

  // Warning if parent and child are equal
  if (currentParent == currentChild)
  {
    this->parentLinkWidget->setStyleSheet(this->warningStyleSheet);
    this->childLinkWidget->setStyleSheet(this->warningStyleSheet);
  }
  else
  {
    this->parentLinkWidget->setStyleSheet(this->normalStyleSheet);
    this->childLinkWidget->setStyleSheet(this->normalStyleSheet);
  }
  this->validLinks = currentParent != currentChild;
  this->UpdateValid();
}

/////////////////////////////////////////////////
void JointInspector::OnSwap()
{
  // Get current values
  std::string currentParent =
      this->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->configWidget->GetEnumWidgetValue("childCombo");

  // Choose new values
  this->configWidget->SetEnumWidgetValue("parentCombo", currentChild);
  this->configWidget->SetEnumWidgetValue("childCombo", currentParent);
}

/////////////////////////////////////////////////
void JointInspector::OnLinkInserted(const std::string &_linkName)
{
  std::string leafName = _linkName;
  size_t idx = _linkName.rfind("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+2);

  this->configWidget->AddItemEnumWidget("parentCombo", leafName);
  this->configWidget->AddItemEnumWidget("childCombo", leafName);

  this->OnLinksChanged();
}

/////////////////////////////////////////////////
void JointInspector::OnLinkRemoved(const std::string &_linkName)
{
  std::string leafName = _linkName;
  size_t idx = _linkName.rfind("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+2);

  this->configWidget->RemoveItemEnumWidget("parentCombo", leafName);
  this->configWidget->RemoveItemEnumWidget("childCombo", leafName);

  this->OnLinksChanged();
}

/////////////////////////////////////////////////
void JointInspector::Open()
{
  // Fill link combo boxes
  this->configWidget->ClearEnumWidget("parentCombo");
  this->configWidget->ClearEnumWidget("childCombo");

  for (const auto &link : this->jointMaker->LinkList())
  {
    this->configWidget->AddItemEnumWidget("parentCombo", link.second);
    this->configWidget->AddItemEnumWidget("childCombo", link.second);
  }

  // Select current parent / child
  std::string currentParent =
      this->configWidget->GetStringWidgetValue("parent");
  std::string currentChild =
      this->configWidget->GetStringWidgetValue("child");

  this->configWidget->blockSignals(true);
  this->configWidget->SetEnumWidgetValue("parentCombo", currentParent);
  this->configWidget->SetEnumWidgetValue("childCombo", currentChild);
  this->configWidget->blockSignals(false);

  // Reset states
  this->parentLinkWidget->setStyleSheet(this->normalStyleSheet);
  this->childLinkWidget->setStyleSheet(this->normalStyleSheet);
  this->okButton->setEnabled(true);

  this->move(QCursor::pos());
  this->show();
}

/////////////////////////////////////////////////
void JointInspector::SetJointId(const std::string &_id)
{
  this->jointId = _id;
}

/////////////////////////////////////////////////
void JointInspector::OnRemove()
{
  this->close();

  this->jointMaker->RemoveJoint(this->jointId);
}

/////////////////////////////////////////////////
void JointInspector::OnCancel()
{
  this->close();
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

/////////////////////////////////////////////////
void JointInspector::UpdateValid()
{
  bool valid = this->validJointName && this->validLinks;

  this->okButton->setEnabled(valid);

  if (valid)
  {
    emit Applied();
  }
}
