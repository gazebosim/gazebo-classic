/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/JointCreationDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointCreationDialog::JointCreationDialog(JointMaker *_jointMaker,
    QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("JointCreationDialogDialog");
  this->setWindowTitle(tr("Create Joint"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  this->setMinimumWidth(300);
  this->setMinimumHeight(550);

  this->jointMaker = _jointMaker;

  // Style sheets
  this->normalStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::level0BgColor + ";\
          color: #4c4c4c;\
        }\
        QLabel\
        {\
          color: #d0d0d0;\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::level0WidgetColor +
        "}";

  this->warningStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::level0BgColor + ";\
          color: " + ConfigWidget::redColor + ";\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::level0WidgetColor +
        "}";

  this->activeStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::level0BgColor + ";\
          color: " + ConfigWidget::greenColor + ";\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::level0WidgetColor +
        "}";

  // ConfigWidget
  this->configWidget = new ConfigWidget();

  // Joint types
  QRadioButton *fixedJointRadio = new QRadioButton(tr("Fixed"));
  QRadioButton *prismaticJointRadio = new QRadioButton(tr("Prismatic"));
  QRadioButton *revoluteJointRadio = new QRadioButton(tr("Revolute"));
  QRadioButton *revolute2JointRadio = new QRadioButton(tr("Revolute2"));
  QRadioButton *screwJointRadio = new QRadioButton(tr("Screw"));
  QRadioButton *universalJointRadio = new QRadioButton(tr("Universal"));
  QRadioButton *ballJointRadio = new QRadioButton(tr("Ball"));
  QRadioButton *gearboxJointRadio = new QRadioButton(tr("Gearbox"));

  this->typeButtons = new QButtonGroup();
  this->typeButtons->addButton(fixedJointRadio, 1);
  this->typeButtons->addButton(prismaticJointRadio, 2);
  this->typeButtons->addButton(revoluteJointRadio, 3);
  this->typeButtons->addButton(revolute2JointRadio, 4);
  this->typeButtons->addButton(screwJointRadio, 5);
  this->typeButtons->addButton(universalJointRadio, 6);
  this->typeButtons->addButton(ballJointRadio, 7);
  this->typeButtons->addButton(gearboxJointRadio, 7);
  connect(this->typeButtons, SIGNAL(buttonClicked(int)),
      this, SLOT(OnTypeFromDialog(int)));

  // Types layout
  QGridLayout *typesLayout = new QGridLayout();
  typesLayout->addWidget(fixedJointRadio, 0, 0);
  typesLayout->addWidget(revoluteJointRadio, 1, 0);
  typesLayout->addWidget(revolute2JointRadio, 2, 0);
  typesLayout->addWidget(prismaticJointRadio, 3, 0);
  typesLayout->addWidget(screwJointRadio, 0, 1);
  typesLayout->addWidget(universalJointRadio, 1, 1);
  typesLayout->addWidget(ballJointRadio, 2, 1);
  typesLayout->addWidget(gearboxJointRadio, 3, 1);

  // Types group widget
  ConfigChildWidget *typesWidget = new ConfigChildWidget();
  typesWidget->setLayout(typesLayout);

  QWidget *typesGroupWidget =
      this->configWidget->CreateGroupWidget("Joint types", typesWidget, 0);

  // Link selections
  QLabel *selectionsText = new QLabel(tr(
      "Click a link in the scene to select parent.\n"
      "Click again to select child."));

  // Parent
  std::vector<std::string> links;
  this->parentLinkWidget = configWidget->CreateEnumWidget("parent", links, 0);
  this->parentLinkWidget->setStyleSheet(this->activeStyleSheet);
  this->configWidget->AddConfigChildWidget("parentCombo",
      this->parentLinkWidget);

  // Child
  this->childLinkWidget = configWidget->CreateEnumWidget("child", links, 0);
  this->childLinkWidget->setStyleSheet(this->normalStyleSheet);
  this->configWidget->AddConfigChildWidget("childCombo", this->childLinkWidget);
  this->configWidget->SetWidgetReadOnly("childCombo", true);

  // Connect all enum value changes
  QObject::connect(this->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)), this,
      SLOT(OnEnumChanged(const QString &, const QString &)));

  // Swap button
  this->swapButton = new QToolButton();
  this->swapButton->setText("Swap");
  this->swapButton->setMinimumWidth(60);
  this->swapButton->setStyleSheet(
      "QToolButton\
      {\
        background-color: " + ConfigWidget::level0BgColor +
      "}");
  connect(this->swapButton, SIGNAL(clicked()), this, SLOT(OnSwap()));

  // Links layout
  QGridLayout *linksLayout = new QGridLayout();
  linksLayout->setContentsMargins(0, 0, 0, 0);
  linksLayout->addWidget(selectionsText, 0, 0, 1, 2);
  linksLayout->addWidget(this->parentLinkWidget, 1, 0);
  linksLayout->addWidget(this->childLinkWidget, 2, 0);
  linksLayout->addWidget(swapButton, 1, 1, 2, 1);

  // Links group widget
  ConfigChildWidget *linksWidget = new ConfigChildWidget();
  linksWidget->setLayout(linksLayout);

  QWidget *linksGroupWidget = this->configWidget->CreateGroupWidget(
      "Link Selections", linksWidget, 0);

  // Pose widget
  ConfigChildWidget *poseWidget = this->configWidget->CreatePoseWidget("pose",
      0);
  this->configWidget->AddConfigChildWidget("pose", poseWidget);
  connect(this->configWidget,
      SIGNAL(PoseValueChanged(const QString,
      const ignition::math::Pose3d)),
      this, SLOT(OnPoseFromDialog(const QString,
      const ignition::math::Pose3d)));

  QWidget *poseGroupWidget = this->configWidget->CreateGroupWidget(
      "Relative Pose", poseWidget, 0);

  // Config Widget layout
  QVBoxLayout *configLayout = new QVBoxLayout();
  configLayout->addWidget(typesGroupWidget);
  configLayout->addWidget(linksGroupWidget);
  configLayout->addWidget(poseGroupWidget);

  this->configWidget->setLayout(configLayout);

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(configWidget);
  scrollArea->setWidgetResizable(true);

  // General layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Cancel button
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  // Create button
  this->createButton = new QPushButton(tr("Create"));
  this->createButton->setEnabled(false);
  connect(this->createButton, SIGNAL(clicked()), this, SLOT(OnCreate()));

  // Buttons layout
  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(createButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  // Gazebo event connections
  this->connections.push_back(
      gui::model::Events::ConnectJointParentChosen3D(
      boost::bind(&JointCreationDialog::OnParentFrom3D, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectJointChildChosen3D(
      boost::bind(&JointCreationDialog::OnChildFrom3D, this, _1)));

  // Qt signal-slot connections
  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));
}

/////////////////////////////////////////////////
void JointCreationDialog::Open(JointMaker::JointType _type)
{
  if (!this->jointMaker)
  {
    gzerr << "Joint maker not found, can't open joint creation dialog."
        << std::endl;
    this->OnCancel();
  }

  // Check joint type
  this->typeButtons->button(static_cast<int>(_type))->setChecked(true);

  // Reset enabled states
  this->createButton->setEnabled(false);
  this->swapButton->setEnabled(false);
  this->configWidget->SetWidgetReadOnly("childCombo", true);
  this->parentLinkWidget->setStyleSheet(this->activeStyleSheet);
  this->childLinkWidget->setStyleSheet(this->normalStyleSheet);

  // Clear links
  this->configWidget->ClearEnumWidget("parentCombo");
  this->configWidget->ClearEnumWidget("childCombo");

  // Add an empty option to each
  this->configWidget->AddItemEnumWidget("parentCombo", "");
  this->configWidget->AddItemEnumWidget("childCombo", "");

  // Fill with all existing links
  for (auto link : this->jointMaker->LinkList())
  {
    this->configWidget->AddItemEnumWidget("parentCombo", link.second);
    this->configWidget->AddItemEnumWidget("childCombo", link.second);
  }

  this->move(0, 0);
  this->show();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnTypeFromDialog(int _type)
{
  JointMaker::JointType type = static_cast<JointMaker::JointType>(_type);
  gui::model::Events::jointTypeChosenDialog(type);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnLinkFromDialog()
{
  std::string currentParent =
      this->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->configWidget->GetEnumWidgetValue("childCombo");

  // Notify so 3D is updated
  if (currentParent != currentChild)
  {
    if (currentParent != "")
      gui::model::Events::jointParentChosenDialog(currentParent);
    if (currentChild != "")
      gui::model::Events::jointChildChosenDialog(currentChild);
  }

  this->OnParentImpl(QString::fromStdString(currentParent));

  if (currentChild != "")
    this->OnChildImpl(QString::fromStdString(currentChild));
}

/////////////////////////////////////////////////
void JointCreationDialog::OnPoseFromDialog(const QString &/*_name*/,
    const ignition::math::Pose3d &_pose)
{
  // Notify so 3D is updated
  gui::model::Events::jointPoseChosenDialog(_pose);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnParentFrom3D(const std::string &_linkName)
{
  if (_linkName.empty())
  {
    gzerr << "Empty link name for parent" << std::endl;
    return;
  }

  std::string leafName = _linkName;
  size_t idx = _linkName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+1);

  this->configWidget->blockSignals(true);
  if (!this->configWidget->SetEnumWidgetValue("parentCombo", leafName))
  {
    gzerr << "Requested link [" << leafName << "] not found" << std::endl;
    return;
  }
  this->configWidget->blockSignals(false);

  this->OnParentImpl(QString::fromStdString(_linkName));
}

/////////////////////////////////////////////////
void JointCreationDialog::OnChildFrom3D(const std::string &_linkName)
{
  if (_linkName.empty())
  {
    gzerr << "Empty link name for child" << std::endl;
    return;
  }

  if (this->configWidget->GetWidgetReadOnly("childCombo"))
  {
    gzerr << "It shouldn't be possible to set child before parent."
        << std::endl;
    return;
  }

  // Update child combo box
  std::string leafName = _linkName;
  size_t idx = _linkName.find_last_of("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+1);

  this->configWidget->blockSignals(true);
  if (!this->configWidget->SetEnumWidgetValue("childCombo", leafName))
  {
    gzerr << "Requested link [" << leafName << "] not found" << std::endl;
    return;
  }
  this->configWidget->blockSignals(false);

  this->OnChildImpl(QString::fromStdString(_linkName));
}

/////////////////////////////////////////////////
void JointCreationDialog::OnParentImpl(const QString &_linkName)
{
  if (_linkName.isEmpty())
  {
    gzerr << "Empty link name for parent" << std::endl;
    return;
  }

  // Remove empty option
  this->configWidget->RemoveItemEnumWidget("parentCombo", "");

  // Check if links are valid
  this->CheckLinksValid();

  // Enable child selection
  if (this->configWidget->GetWidgetReadOnly("childCombo"))
  {
    this->configWidget->SetWidgetReadOnly("childCombo", false);
    this->childLinkWidget->setStyleSheet(this->activeStyleSheet);
  }
}

/////////////////////////////////////////////////
void JointCreationDialog::OnChildImpl(const QString &_linkName)
{
  if (_linkName.isEmpty())
  {
    gzerr << "Empty link name for child" << std::endl;
    return;
  }

  // Enable create and swap
  this->createButton->setEnabled(true);
  this->swapButton->setEnabled(true);

  // Remove empty option
  this->configWidget->RemoveItemEnumWidget("childCombo", "");

  // Check if links are valid
  this->CheckLinksValid();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCancel()
{
  this->close();
  this->jointMaker->Stop();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCreate()
{
  this->accept();

  gui::model::Events::jointCreateDialog();
}

/////////////////////////////////////////////////
void JointCreationDialog::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnSwap()
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
void JointCreationDialog::UpdateRelativePose(
    const ignition::math::Pose3d &_pose)
{
  this->configWidget->SetPoseWidgetValue("pose", math::Pose(_pose));
}

/////////////////////////////////////////////////
void JointCreationDialog::OnEnumChanged(const QString &_name,
    const QString &/*_value*/)
{
  if (_name == "parentCombo" || _name == "childCombo")
    this->OnLinkFromDialog();
}

/////////////////////////////////////////////////
void JointCreationDialog::CheckLinksValid()
{
  std::string currentParent =
      this->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->configWidget->GetEnumWidgetValue("childCombo");

  // Warning if parent is the same as the child and don't allow creation
  if (currentParent == currentChild)
  {
    this->parentLinkWidget->setStyleSheet(this->warningStyleSheet);
    this->childLinkWidget->setStyleSheet(this->warningStyleSheet);
    this->createButton->setEnabled(false);
  }
  else
  {
    this->parentLinkWidget->setStyleSheet(this->normalStyleSheet);
    if (!this->configWidget->GetWidgetReadOnly("childCombo"))
    {
      this->childLinkWidget->setStyleSheet(this->normalStyleSheet);
      this->createButton->setEnabled(true);
    }
  }
}
