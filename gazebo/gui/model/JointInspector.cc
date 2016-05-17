/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Assert.hh"

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/JointInspectorPrivate.hh"
#include "gazebo/gui/model/JointMaker.hh"

#include "gazebo/rendering/Material.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointInspector::JointInspector(JointMaker *_jointMaker, QWidget *_parent)
    : QDialog(_parent), dataPtr(new JointInspectorPrivate)
{
  this->setObjectName("JointInspectorDialog");
  this->setWindowTitle(tr("Joint Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  this->dataPtr->jointMaker = _jointMaker;

  // ConfigWidget
  this->dataPtr->configWidget = new ConfigWidget;
  msgs::Joint jointMsg;
  this->dataPtr->configWidget->Load(&jointMsg);

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
    this->dataPtr->configWidget->SetVector3dWidgetValue(axisStr + "::xyz",
        axisElem->Get<ignition::math::Vector3d>("xyz"));
    this->dataPtr->configWidget->SetDoubleWidgetValue(axisStr + "::limit_lower",
        axisLimitElem->Get<double>("lower"));
    this->dataPtr->configWidget->SetDoubleWidgetValue(axisStr + "::limit_upper",
        axisLimitElem->Get<double>("upper"));
    this->dataPtr->configWidget->SetDoubleWidgetValue(axisStr +
        "::limit_effort", axisLimitElem->Get<double>("effort"));
    this->dataPtr->configWidget->SetDoubleWidgetValue(axisStr +
        "::limit_velocity", axisLimitElem->Get<double>("velocity"));
    this->dataPtr->configWidget->SetDoubleWidgetValue(axisStr + "::damping",
        axisElem->GetElement("dynamics")->Get<double>("damping"));
    this->dataPtr->configWidget->SetDoubleWidgetValue(axisStr + "::friction",
        axisElem->GetElement("dynamics")->Get<double>("friction"));
    this->dataPtr->configWidget->SetBoolWidgetValue(axisStr +
        "::parent_model_frame", axisElem->Get<bool>("use_parent_model_frame"));
  }

  this->dataPtr->configWidget->SetDoubleWidgetValue("cfm",
      odeElem->Get<double>("cfm"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("bounce",
      odeElem->Get<double>("bounce"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("velocity",
      odeElem->Get<double>("velocity"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("fudge_factor",
      odeElem->Get<double>("fudge_factor"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("limit_cfm",
      odeElem->GetElement("limit")->Get<double>("cfm"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("limit_erp",
      odeElem->GetElement("limit")->Get<double>("erp"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("suspension_cfm",
      odeElem->GetElement("suspension")->Get<double>("cfm"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("suspension_erp",
      odeElem->GetElement("suspension")->Get<double>("erp"));

  // joint type specific properties.
  this->dataPtr->configWidget->SetStringWidgetValue(
      "gearbox::gearbox_reference_body",
      jointElem->HasElement("gearbox_reference_body") ?
      jointElem->Get<std::string>("gearbox_reference_body") : "");
  this->dataPtr->configWidget->SetDoubleWidgetValue("gearbox::gearbox_ratio",
      jointElem->Get<double>("gearbox_ratio"));
  this->dataPtr->configWidget->SetDoubleWidgetValue("screw::thread_pitch",
      jointElem->Get<double>("thread_pitch"));

  // Hide fields
  this->dataPtr->configWidget->SetWidgetVisible("id", false);
  this->dataPtr->configWidget->SetWidgetVisible("parent_id", false);
  this->dataPtr->configWidget->SetWidgetVisible("child_id", false);
  this->dataPtr->configWidget->SetWidgetVisible("parent", false);
  this->dataPtr->configWidget->SetWidgetVisible("child", false);

  this->dataPtr->configWidget->SetWidgetReadOnly("id", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("parent_id", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("child_id", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("parent", true);
  this->dataPtr->configWidget->SetWidgetReadOnly("child", true);

  // Get name widget
  this->dataPtr->nameWidget =
      this->dataPtr->configWidget->ConfigChildWidgetByName("name");
  if (!this->dataPtr->nameWidget)
    gzerr << "Name widget not found" << std::endl;

  // Custom parent / child widgets
  // Parent
  std::vector<std::string> links;
  this->dataPtr->parentLinkWidget =
      this->dataPtr->configWidget->CreateEnumWidget("parent", links, 0);
  this->dataPtr->parentLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal"));
  this->dataPtr->configWidget->AddConfigChildWidget("parentCombo",
      this->dataPtr->parentLinkWidget);

  // Resize parent label
  auto parentLabel = this->dataPtr->parentLinkWidget->findChild<QLabel *>();
  parentLabel->setMaximumWidth(50);

  // Add parent icon
  this->dataPtr->parentIcon = new QLabel();
  this->dataPtr->parentIcon->setMinimumWidth(13);
  this->dataPtr->parentIcon->setMaximumHeight(13);
  auto parentLayout = qobject_cast<QHBoxLayout *>(
      this->dataPtr->parentLinkWidget->layout());
  if (parentLayout)
  {
    parentLayout->insertWidget(1, this->dataPtr->parentIcon);
    parentLayout->setAlignment(this->dataPtr->parentIcon, Qt::AlignLeft);
  }

  // Child
  this->dataPtr->childLinkWidget =
      this->dataPtr->configWidget->CreateEnumWidget("child", links, 0);
  this->dataPtr->childLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal"));
  this->dataPtr->configWidget->AddConfigChildWidget("childCombo",
      this->dataPtr->childLinkWidget);

  // Resize child label
  auto childLabel = this->dataPtr->childLinkWidget->findChild<QLabel *>();
  childLabel->setMaximumWidth(50);

  // Add child icon
  QPixmap childPix(":/images/child-link.png");
  childPix = childPix.scaled(15, 15);
  auto childIcon = new QLabel();
  childIcon->setPixmap(childPix);
  childIcon->setMaximumWidth(15);
  auto childLayout = qobject_cast<QHBoxLayout *>(
      this->dataPtr->childLinkWidget->layout());
  if (childLayout)
  {
    childLayout->insertWidget(1, childIcon);
    childLayout->setAlignment(childIcon, Qt::AlignLeft);
  }

  // Swap button
  QToolButton *swapButton = new QToolButton();
  swapButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  swapButton->setIcon(QPixmap(":/images/swap-parent-child.png"));
  swapButton->setFixedSize(QSize(45, 35));
  swapButton->setIconSize(QSize(25, 25));
  swapButton->setToolTip("Swap parent and child");
  swapButton->setStyleSheet(
      "QToolButton\
      {\
        background-color: " + ConfigWidget::bgColors[0] + ";\
        margin-left: 10px;\
      }");
  connect(swapButton, SIGNAL(clicked()), this, SLOT(OnSwap()));

  // Links layout
  QGridLayout *linksLayout = new QGridLayout();
  linksLayout->setContentsMargins(0, 0, 0, 0);
  linksLayout->addWidget(this->dataPtr->parentLinkWidget, 0, 0);
  linksLayout->addWidget(this->dataPtr->childLinkWidget, 1, 0);
  linksLayout->addWidget(swapButton, 0, 1, 2, 1);

  // Insert on the top of config widget's layout
  this->dataPtr->configWidget->InsertLayout(linksLayout, 0);

  // Connect all enum value changes, which includes type, parent and child
  QObject::connect(this->dataPtr->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)), this,
      SLOT(OnEnumChanged(const QString &, const QString &)));

  // Connect pose value changes, for joint pose
  connect(this->dataPtr->configWidget, SIGNAL(PoseValueChanged(const QString &,
      const ignition::math::Pose3d &)), this,
      SLOT(OnPoseChanged(const QString &, const ignition::math::Pose3d &)));

  // Connect vector value changes, for axes
  connect(this->dataPtr->configWidget,
      SIGNAL(Vector3dValueChanged(const QString &,
      const ignition::math::Vector3d &)), this,
      SLOT(OnVector3dChanged(const QString &,
      const ignition::math::Vector3d &)));

  // Connect string value changes, for name
  connect(this->dataPtr->configWidget,
      SIGNAL(StringValueChanged(const QString &, const std::string &)), this,
      SLOT(OnStringChanged(const QString &, const std::string &)));

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->dataPtr->configWidget);
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

  QPushButton *resetButton = new QPushButton(tr("Reset"));
  connect(resetButton, SIGNAL(clicked()), this, SLOT(RestoreOriginalData()));

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  this->dataPtr->okButton = new QPushButton(tr("OK"));
  this->dataPtr->okButton->setEnabled(true);
  connect(this->dataPtr->okButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(removeButton);
  buttonsLayout->addStretch(5);
  buttonsLayout->addWidget(resetButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(this->dataPtr->okButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);

  this->setLayout(mainLayout);

  // Qt signal / slot connections
  connect(this->dataPtr->jointMaker, SIGNAL(EmitLinkRemoved(std::string)), this,
      SLOT(OnLinkRemoved(std::string)));
  connect(this->dataPtr->jointMaker, SIGNAL(EmitLinkInserted(std::string)),
      this, SLOT(OnLinkInserted(std::string)));
  connect(this, SIGNAL(rejected()), this, SLOT(RestoreOriginalData()));

  // Initialize variables
  this->dataPtr->validJointName = true;
  this->dataPtr->validLinks = true;
}

/////////////////////////////////////////////////
JointInspector::~JointInspector()
{
}

/////////////////////////////////////////////////
void JointInspector::Update(ConstJointPtr _jointMsg)
{
  this->dataPtr->configWidget->UpdateFromMsg(_jointMsg.get());
}

/////////////////////////////////////////////////
void JointInspector::SetPose(const ignition::math::Pose3d &_pose)
{
  this->dataPtr->configWidget->SetPoseWidgetValue("pose", _pose);
}

/////////////////////////////////////////////////
msgs::Joint *JointInspector::Data() const
{
  std::string currentParent =
      this->dataPtr->configWidget->EnumWidgetValue("parentCombo");

  std::string currentChild =
      this->dataPtr->configWidget->EnumWidgetValue("childCombo");

  if (currentParent == currentChild)
  {
    return NULL;
  }

  // Get updated message from widget
  msgs::Joint *msg = dynamic_cast<msgs::Joint *>(
      this->dataPtr->configWidget->Msg());
  if (!msg)
  {
    gzerr << "It wasn't possible to get the joint message" << std::endl;
    return NULL;
  }

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

  if (this->CheckValid())
    emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnPoseChanged(const QString &/*_name*/,
    const ignition::math::Pose3d &/*_pose*/)
{
  if (this->CheckValid())
    emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnVector3dChanged(const QString &/*_name*/,
    const ignition::math::Vector3d &/*_vec*/)
{
  if (this->CheckValid())
    emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnStringChanged(const QString &_name,
    const std::string &_str)
{
  // Handle joint name
  if (_name != "name")
    return;

  /// \todo Also check if name overlaps with other joints
  this->dataPtr->validJointName = !_str.empty();

  // Warning if joint name is invalid
  if (!this->dataPtr->validJointName)
  {
    this->dataPtr->nameWidget->setStyleSheet(
        ConfigWidget::StyleSheet("warning"));
  }
  else
  {
    this->dataPtr->nameWidget->setStyleSheet(
        ConfigWidget::StyleSheet("normal"));
  }

  // Only apply if all fields are valid
  if (this->CheckValid())
    emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnJointTypeChanged(const QString &_value)
{
  std::string valueStr = _value.toLower().toStdString();
  auto type = JointMaker::ConvertJointType(valueStr);
  unsigned int axisCount = JointMaker::JointAxisCount(type);

  for (unsigned int i = 0; i < axisCount; ++i)
  {
    std::stringstream axis;
    axis << "axis" << i+1;
    std::string axisStr = axis.str();

    this->dataPtr->configWidget->SetWidgetVisible(axisStr, true);
    this->dataPtr->configWidget->SetWidgetReadOnly(axisStr, false);
    this->dataPtr->configWidget->UpdateFromMsg(
        this->dataPtr->configWidget->Msg());
  }

  for (unsigned int i = axisCount; i < 2u; ++i)
  {
    std::stringstream axis;
    axis << "axis" << i+1;
    std::string axisStr = axis.str();

    this->dataPtr->configWidget->SetWidgetVisible(axisStr, false);
    this->dataPtr->configWidget->SetWidgetReadOnly(axisStr, true);
    this->dataPtr->configWidget->UpdateFromMsg(
        this->dataPtr->configWidget->Msg());
  }

  // toggle field visibility according to joint type.
  bool isGearbox = valueStr == "gearbox";
  bool isScrew = valueStr == "screw";
  this->dataPtr->configWidget->SetWidgetVisible("gearbox", isGearbox);
  this->dataPtr->configWidget->SetWidgetReadOnly("gearbox", !isGearbox);
  this->dataPtr->configWidget->SetWidgetVisible("screw", isScrew);
  this->dataPtr->configWidget->SetWidgetReadOnly("screw", !isScrew);

  // Change child icon color according to type
  common::Color matAmbient, matDiffuse, matSpecular, matEmissive;
  rendering::Material::GetMaterialAsColor(
      this->dataPtr->jointMaker->jointMaterials[type],
      matAmbient, matDiffuse, matSpecular, matEmissive);

  std::ostringstream sheet;
  sheet << "QLabel{background-color: rgb(" <<
          (matAmbient[0] * 255) << ", " <<
          (matAmbient[1] * 255) << ", " <<
          (matAmbient[2] * 255) << "); }";

  this->dataPtr->parentIcon->setStyleSheet(QString::fromStdString(sheet.str()));
}

/////////////////////////////////////////////////
void JointInspector::OnLinksChanged(const QString &/*_linkName*/)
{
  std::string currentParent =
      this->dataPtr->configWidget->EnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->EnumWidgetValue("childCombo");

  // Warning if parent and child are equal
  if (currentParent == currentChild)
  {
    this->dataPtr->parentLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("warning"));
    this->dataPtr->childLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("warning"));
  }
  else
  {
    this->dataPtr->parentLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("normal"));
    this->dataPtr->childLinkWidget->setStyleSheet(
        ConfigWidget::StyleSheet("normal"));
  }
  this->dataPtr->validLinks = currentParent != currentChild;
}

/////////////////////////////////////////////////
void JointInspector::OnSwap()
{
  // Get current values
  std::string currentParent =
      this->dataPtr->configWidget->EnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->EnumWidgetValue("childCombo");

  // Choose new values. We only need signals to be emitted once.
  this->dataPtr->configWidget->blockSignals(true);
  this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo", currentChild);
  this->dataPtr->configWidget->blockSignals(false);
  this->dataPtr->configWidget->SetEnumWidgetValue("childCombo", currentParent);
}

/////////////////////////////////////////////////
void JointInspector::OnLinkInserted(const std::string &_linkName)
{
  std::string unscopedName = _linkName;
  size_t idx = _linkName.find("::");
  if (idx != std::string::npos)
    unscopedName = _linkName.substr(idx+2);

  this->dataPtr->configWidget->AddItemEnumWidget("parentCombo", unscopedName);
  this->dataPtr->configWidget->AddItemEnumWidget("childCombo", unscopedName);

  this->OnLinksChanged();
}

/////////////////////////////////////////////////
void JointInspector::OnLinkRemoved(const std::string &_linkName)
{
  std::string unscopedName = _linkName;
  size_t idx = _linkName.find("::");
  if (idx != std::string::npos)
    unscopedName = _linkName.substr(idx+2);

  this->dataPtr->configWidget->RemoveItemEnumWidget("parentCombo",
      unscopedName);
  this->dataPtr->configWidget->RemoveItemEnumWidget("childCombo", unscopedName);

  this->OnLinksChanged();
}

/////////////////////////////////////////////////
void JointInspector::Open()
{
  // Fill link combo boxes
  this->dataPtr->configWidget->ClearEnumWidget("parentCombo");
  this->dataPtr->configWidget->ClearEnumWidget("childCombo");

  for (const auto &link : this->dataPtr->jointMaker->LinkList())
  {
    this->dataPtr->configWidget->AddItemEnumWidget("parentCombo", link.second);
    this->dataPtr->configWidget->AddItemEnumWidget("childCombo", link.second);
  }

  // Select current parent / child
  std::string currentParent =
      this->dataPtr->configWidget->StringWidgetValue("parent");
  std::string currentChild =
      this->dataPtr->configWidget->StringWidgetValue("child");

  this->dataPtr->configWidget->blockSignals(true);
  if (!currentParent.empty())
  {
    this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo",
        currentParent);
  }
  if (!currentChild.empty())
  {
    this->dataPtr->configWidget->SetEnumWidgetValue("childCombo", currentChild);
  }
  this->dataPtr->configWidget->blockSignals(false);

  // Reset states
  this->dataPtr->parentLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal"));
  this->dataPtr->childLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal"));
  this->dataPtr->nameWidget->setStyleSheet(ConfigWidget::StyleSheet("normal"));
  this->dataPtr->okButton->setEnabled(true);

  // Keep original data in case user cancels
  auto msg = this->Data();
  if (msg)
    this->dataPtr->originalDataMsg.CopyFrom(*msg);

  // Make sure the dialog opens with the proper fields showing
  this->blockSignals(true);
  this->RestoreOriginalData();
  this->blockSignals(false);

  this->move(QCursor::pos());
  this->show();
}

/////////////////////////////////////////////////
void JointInspector::SetJointId(const std::string &_id)
{
  this->dataPtr->jointId = _id;
}

/////////////////////////////////////////////////
void JointInspector::OnRemove()
{
  this->close();

  this->dataPtr->jointMaker->RemoveJointByUser(this->dataPtr->jointId);
}

/////////////////////////////////////////////////
void JointInspector::RestoreOriginalData()
{
  msgs::JointPtr jointPtr;
  jointPtr.reset(new msgs::Joint);
  jointPtr->CopyFrom(this->dataPtr->originalDataMsg);

  // Update default widgets
  this->dataPtr->configWidget->blockSignals(true);
  this->Update(jointPtr);

  // Update joint type and parent icon
  this->OnJointTypeChanged(tr(msgs::Joint_Type_Name(jointPtr->type()).c_str()));

  // Update custom widgets
  std::string originalParent =
      this->dataPtr->configWidget->StringWidgetValue("parent");
  std::string originalChild =
      this->dataPtr->configWidget->StringWidgetValue("child");

  if (!originalParent.empty())
  {
    this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo",
        originalParent);
  }
  if (!originalChild.empty())
  {
    this->dataPtr->configWidget->SetEnumWidgetValue("childCombo",
        originalChild);
  }
  this->dataPtr->configWidget->blockSignals(false);

  // Reset variables, we assume the original data was valid
  this->dataPtr->validJointName = true;
  this->dataPtr->validLinks = true;
  this->dataPtr->nameWidget->setStyleSheet(ConfigWidget::StyleSheet("normal"));
  this->dataPtr->parentLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal"));
  this->dataPtr->childLinkWidget->setStyleSheet(
      ConfigWidget::StyleSheet("normal"));

  if (this->CheckValid())
    emit Applied();
}

/////////////////////////////////////////////////
void JointInspector::OnCancel()
{
  this->RestoreOriginalData();

  this->reject();
}

/////////////////////////////////////////////////
void JointInspector::OnOK()
{
  if (this->CheckValid())
  {
    emit Applied();
    this->accept();
  }
  else
  {
    gzerr << "It shouldn't be possible to press Ok with invalid inputs." <<
        std::endl;
  }
}

/////////////////////////////////////////////////
void JointInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
bool JointInspector::CheckValid()
{
  bool valid = this->dataPtr->validJointName && this->dataPtr->validLinks;

  if (this->dataPtr->okButton)
    this->dataPtr->okButton->setEnabled(valid);

  return valid;
}

/////////////////////////////////////////////////
void JointInspector::keyPressEvent(QKeyEvent *_event)
{
  if (_event->key() == Qt::Key_Enter)
    _event->accept();
  else
    QDialog::keyPressEvent(_event);
}

///////////////////////////////////////////////////
void JointInspector::closeEvent(QCloseEvent *_event)
{
  _event->accept();
}
