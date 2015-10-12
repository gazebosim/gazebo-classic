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
#include "gazebo/gui/model/JointCreationDialogPrivate.hh"
#include "gazebo/gui/model/JointCreationDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointCreationDialog::JointCreationDialog(JointMaker *_jointMaker,
    QWidget *_parent) : QDialog(_parent),
    dataPtr(new JointCreationDialogPrivate)
{
  this->setObjectName("JointCreationDialogDialog");
  this->setWindowTitle(tr("Create Joint"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  this->setMinimumWidth(300);
  this->setMinimumHeight(900);

  this->dataPtr->jointMaker = _jointMaker;

  // Style sheets
  this->dataPtr->normalStyleSheet =
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

  this->dataPtr->warningStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::bgColors[0] + ";\
          color: " + ConfigWidget::redColor + ";\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::widgetColors[0] +
        "}";

  this->dataPtr->activeStyleSheet =
        "QWidget\
        {\
          background-color: " + ConfigWidget::bgColors[0] + ";\
          color: " + ConfigWidget::greenColor + ";\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::widgetColors[0] +
        "}";

  // ConfigWidget
  this->dataPtr->configWidget = new ConfigWidget();

  // Joint types
  QRadioButton *fixedJointRadio = new QRadioButton(tr("Fixed"));
  QRadioButton *prismaticJointRadio = new QRadioButton(tr("Prismatic"));
  QRadioButton *revoluteJointRadio = new QRadioButton(tr("Revolute"));
  QRadioButton *revolute2JointRadio = new QRadioButton(tr("Revolute2"));
  QRadioButton *screwJointRadio = new QRadioButton(tr("Screw"));
  QRadioButton *universalJointRadio = new QRadioButton(tr("Universal"));
  QRadioButton *ballJointRadio = new QRadioButton(tr("Ball"));
  QRadioButton *gearboxJointRadio = new QRadioButton(tr("Gearbox"));

  this->dataPtr->typeButtons = new QButtonGroup();
  this->dataPtr->typeButtons->addButton(fixedJointRadio, 1);
  this->dataPtr->typeButtons->addButton(prismaticJointRadio, 2);
  this->dataPtr->typeButtons->addButton(revoluteJointRadio, 3);
  this->dataPtr->typeButtons->addButton(revolute2JointRadio, 4);
  this->dataPtr->typeButtons->addButton(screwJointRadio, 5);
  this->dataPtr->typeButtons->addButton(universalJointRadio, 6);
  this->dataPtr->typeButtons->addButton(ballJointRadio, 7);
  this->dataPtr->typeButtons->addButton(gearboxJointRadio, 7);
  connect(this->dataPtr->typeButtons, SIGNAL(buttonClicked(int)),
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
      this->dataPtr->configWidget->CreateGroupWidget("Joint types",
      typesWidget, 0);

  // Link selections
  QLabel *selectionsText = new QLabel(tr(
      "Click a link in the scene to select parent.\n"
      "Click again to select child."));

  // Parent
  std::vector<std::string> links;
  this->dataPtr->parentLinkWidget =
      this->dataPtr->configWidget->CreateEnumWidget("parent", links, 0);
  this->dataPtr->parentLinkWidget->setStyleSheet(
      this->dataPtr->activeStyleSheet);
  this->dataPtr->configWidget->AddConfigChildWidget("parentCombo",
      this->dataPtr->parentLinkWidget);

  // Child
  this->dataPtr->childLinkWidget =
      this->dataPtr->configWidget->CreateEnumWidget("child", links, 0);
  this->dataPtr->childLinkWidget->setStyleSheet(
      this->dataPtr->normalStyleSheet);
  this->dataPtr->configWidget->AddConfigChildWidget("childCombo",
      this->dataPtr->childLinkWidget);
  this->dataPtr->configWidget->SetWidgetReadOnly("childCombo", true);

  // Connect all enum value changes
  QObject::connect(this->dataPtr->configWidget,
      SIGNAL(EnumValueChanged(const QString &, const QString &)), this,
      SLOT(OnEnumChanged(const QString &, const QString &)));

  // Swap button
  this->dataPtr->swapButton = new QToolButton();
  this->dataPtr->swapButton->setToolButtonStyle(Qt::ToolButtonIconOnly);
  this->dataPtr->swapButton->setIcon(
      QPixmap(":/images/swap-parent-child.png"));
  this->dataPtr->swapButton->setFixedSize(QSize(50, 50));
  this->dataPtr->swapButton->setIconSize(QSize(35, 35));
  this->dataPtr->swapButton->setToolTip("Swap parent and child");
  this->dataPtr->swapButton->setStyleSheet(
      "QToolButton\
      {\
        background-color: " + ConfigWidget::bgColors[0] +
      "}");
  connect(this->dataPtr->swapButton, SIGNAL(clicked()), this, SLOT(OnSwap()));

  // Links layout
  QGridLayout *linksLayout = new QGridLayout();
  linksLayout->setContentsMargins(0, 0, 0, 0);
  linksLayout->addWidget(selectionsText, 0, 0, 1, 2);
  linksLayout->addWidget(this->dataPtr->parentLinkWidget, 1, 0);
  linksLayout->addWidget(this->dataPtr->childLinkWidget, 2, 0);
  linksLayout->addWidget(this->dataPtr->swapButton, 1, 1, 2, 1);

  // Links group widget
  ConfigChildWidget *linksWidget = new ConfigChildWidget();
  linksWidget->setLayout(linksLayout);

  QWidget *linksGroupWidget = this->dataPtr->configWidget->CreateGroupWidget(
      "Link Selections", linksWidget, 0);

  // Axis1 widget
  ConfigChildWidget *axis1Widget =
      this->dataPtr->configWidget->CreateVector3dWidget("axis1", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("axis1", axis1Widget);
  connect(this->dataPtr->configWidget,
      SIGNAL(Vector3dValueChanged(const QString,
      const ignition::math::Vector3d)),
      this, SLOT(OnVector3dFromDialog(const QString,
      const ignition::math::Vector3d)));

  // Axis1 presets widget
  this->dataPtr->axis1PresetsCombo = new QComboBox();
  this->dataPtr->axis1PresetsCombo->addItem("X");
  this->dataPtr->axis1PresetsCombo->addItem("Y");
  this->dataPtr->axis1PresetsCombo->addItem("Z");
  this->dataPtr->axis1PresetsCombo->addItem("Custom");
  connect(this->dataPtr->axis1PresetsCombo,
      SIGNAL(currentIndexChanged(const QString &)), this,
      SLOT(OnAxis1Presets(const QString &)));

  // Axis2 widget
  ConfigChildWidget *axis2Widget =
      this->dataPtr->configWidget->CreateVector3dWidget("axis2", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("axis2", axis2Widget);

  // Axis2 presets widget
  this->dataPtr->axis2PresetsCombo = new QComboBox();
  this->dataPtr->axis2PresetsCombo->addItem("X");
  this->dataPtr->axis2PresetsCombo->addItem("Y");
  this->dataPtr->axis2PresetsCombo->addItem("Z");
  this->dataPtr->axis2PresetsCombo->addItem("Custom");
  connect(this->dataPtr->axis2PresetsCombo,
      SIGNAL(currentIndexChanged(const QString &)), this,
      SLOT(OnAxis2Presets(const QString &)));

  // Axis general layout
  QGridLayout *axisGeneralLayout = new QGridLayout();
  axisGeneralLayout->setContentsMargins(0, 0, 0, 0);
  axisGeneralLayout->addWidget(axis1Widget, 0, 0);
  axisGeneralLayout->addWidget(this->dataPtr->axis1PresetsCombo, 0, 1);
  axisGeneralLayout->addWidget(axis2Widget, 1, 0);
  axisGeneralLayout->addWidget(this->dataPtr->axis2PresetsCombo, 1, 1);

  ConfigChildWidget *axisGeneralWidget = new ConfigChildWidget();
  axisGeneralWidget->setLayout(axisGeneralLayout);

  QWidget *axisGroupWidget = this->dataPtr->configWidget->CreateGroupWidget(
      "Joint axis", axisGeneralWidget, 0);

  // Align widgets
  QLabel *xAlignLabel = new QLabel(tr("X: "));
  QLabel *yAlignLabel = new QLabel(tr("Y: "));
  QLabel *zAlignLabel = new QLabel(tr("Z: "));

  auto xAlignMin = new QToolButton();
  xAlignMin->setToolButtonStyle(Qt::ToolButtonIconOnly);
  xAlignMin->setIcon(QIcon(":/images/x_min.png"));
  xAlignMin->setCheckable(true);

  auto xAlignCenter = new QToolButton();
  xAlignCenter->setToolButtonStyle(Qt::ToolButtonIconOnly);
  xAlignCenter->setIcon(QIcon(":/images/x_center.png"));
  xAlignCenter->setCheckable(true);

  auto xAlignMax = new QToolButton();
  xAlignMax->setToolButtonStyle(Qt::ToolButtonIconOnly);
  xAlignMax->setIcon(QIcon(":/images/x_max.png"));
  xAlignMax->setCheckable(true);

  auto yAlignMin = new QToolButton();
  yAlignMin->setToolButtonStyle(Qt::ToolButtonIconOnly);
  yAlignMin->setIcon(QIcon(":/images/y_min.png"));
  yAlignMin->setCheckable(true);

  auto yAlignCenter = new QToolButton();
  yAlignCenter->setToolButtonStyle(Qt::ToolButtonIconOnly);
  yAlignCenter->setIcon(QIcon(":/images/y_center.png"));
  yAlignCenter->setCheckable(true);

  auto yAlignMax = new QToolButton();
  yAlignMax->setToolButtonStyle(Qt::ToolButtonIconOnly);
  yAlignMax->setIcon(QIcon(":/images/y_max.png"));
  yAlignMax->setCheckable(true);

  auto zAlignMin = new QToolButton();
  zAlignMin->setToolButtonStyle(Qt::ToolButtonIconOnly);
  zAlignMin->setIcon(QIcon(":/images/z_min.png"));
  zAlignMin->setCheckable(true);

  auto zAlignCenter = new QToolButton();
  zAlignCenter->setToolButtonStyle(Qt::ToolButtonIconOnly);
  zAlignCenter->setIcon(QIcon(":/images/z_center.png"));
  zAlignCenter->setCheckable(true);

  auto zAlignMax = new QToolButton();
  zAlignMax->setToolButtonStyle(Qt::ToolButtonIconOnly);
  zAlignMax->setIcon(QIcon(":/images/z_max.png"));
  zAlignMax->setCheckable(true);

  auto alignXButtonGroup = new QButtonGroup();
  alignXButtonGroup->setExclusive(false);
  alignXButtonGroup->addButton(xAlignMin, 0);
  alignXButtonGroup->addButton(xAlignCenter, 1);
  alignXButtonGroup->addButton(xAlignMax, 2);
  connect(alignXButtonGroup, SIGNAL(buttonClicked(const int)),
      this, SLOT(OnAlign(const int)));
  this->dataPtr->alignGroups.push_back(alignXButtonGroup);

  auto alignYButtonGroup = new QButtonGroup();
  alignYButtonGroup->setExclusive(false);
  alignYButtonGroup->addButton(yAlignMin, 0);
  alignYButtonGroup->addButton(yAlignCenter, 1);
  alignYButtonGroup->addButton(yAlignMax, 2);
  connect(alignYButtonGroup, SIGNAL(buttonClicked(const int)),
      this, SLOT(OnAlign(const int)));
  this->dataPtr->alignGroups.push_back(alignYButtonGroup);

  auto alignZButtonGroup = new QButtonGroup();
  alignZButtonGroup->setExclusive(false);
  alignZButtonGroup->addButton(zAlignMin, 0);
  alignZButtonGroup->addButton(zAlignCenter, 1);
  alignZButtonGroup->addButton(zAlignMax, 2);
  connect(alignZButtonGroup, SIGNAL(buttonClicked(const int)),
      this, SLOT(OnAlign(const int)));
  this->dataPtr->alignGroups.push_back(alignZButtonGroup);

  // Align dropdown
  this->dataPtr->alignCombo = new QComboBox();
  this->dataPtr->alignCombo->addItem("Child to Parent", 0);
  this->dataPtr->alignCombo->addItem("Parent to Child", 1);
  connect(this->dataPtr->alignCombo, SIGNAL(currentIndexChanged(const int)),
      this, SLOT(OnAlign(const int)));

  // Align layout
  QGridLayout *alignLayout = new QGridLayout();
  alignLayout->addWidget(xAlignLabel, 0, 0);
  alignLayout->addWidget(xAlignMin, 0, 1);
  alignLayout->addWidget(xAlignCenter, 0, 2);
  alignLayout->addWidget(xAlignMax, 0, 3);
  alignLayout->addWidget(yAlignLabel, 1, 0);
  alignLayout->addWidget(yAlignMin, 1, 1);
  alignLayout->addWidget(yAlignCenter, 1, 2);
  alignLayout->addWidget(yAlignMax, 1, 3);
  alignLayout->addWidget(zAlignLabel, 2, 0);
  alignLayout->addWidget(zAlignMin, 2, 1);
  alignLayout->addWidget(zAlignCenter, 2, 2);
  alignLayout->addWidget(zAlignMax, 2, 3);
  alignLayout->addWidget(this->dataPtr->alignCombo, 0, 4, 1, 3);

  // Align group widget
  ConfigChildWidget *alignWidget = new ConfigChildWidget();
  alignWidget->setLayout(alignLayout);

  QWidget *alignGroupWidget =
      this->dataPtr->configWidget->CreateGroupWidget("Align links",
      alignWidget, 0);

  // Pose widget
  ConfigChildWidget *poseWidget =
      this->dataPtr->configWidget->CreatePoseWidget("pose", 0);
  this->dataPtr->configWidget->AddConfigChildWidget("pose", poseWidget);
  connect(this->dataPtr->configWidget,
      SIGNAL(PoseValueChanged(const QString,
      const ignition::math::Pose3d)),
      this, SLOT(OnPoseFromDialog(const QString,
      const ignition::math::Pose3d)));

  // Reset pose button
  QPushButton *resetPoseButton = new QPushButton(tr("Reset"));
  resetPoseButton->setToolTip("Reset parent and child poses");
  connect(resetPoseButton, SIGNAL(clicked()), this,
      SLOT(OnResetPoses()));

  // Pose general layout
  QVBoxLayout *poseGeneralLayout = new QVBoxLayout();
  poseGeneralLayout->setContentsMargins(0, 0, 0, 0);
  poseGeneralLayout->addWidget(poseWidget);
  poseGeneralLayout->addWidget(resetPoseButton);

  ConfigChildWidget *poseGeneralWidget = new ConfigChildWidget();
  poseGeneralWidget->setLayout(poseGeneralLayout);

  QWidget *poseGroupWidget = this->dataPtr->configWidget->CreateGroupWidget(
      "Relative Pose", poseGeneralWidget, 0);

  // Config Widget layout
  QVBoxLayout *configLayout = new QVBoxLayout();
  configLayout->addWidget(typesGroupWidget);
  configLayout->addWidget(linksGroupWidget);
  configLayout->addWidget(axisGroupWidget);
  configLayout->addWidget(alignGroupWidget);
  configLayout->addWidget(poseGroupWidget);

  this->dataPtr->configWidget->setLayout(configLayout);

  // Scroll area
  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(this->dataPtr->configWidget);
  scrollArea->setWidgetResizable(true);

  // General layout
  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);

  // Cancel button
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  // Create button
  this->dataPtr->createButton = new QPushButton(tr("Create"));
  this->dataPtr->createButton->setEnabled(false);
  connect(this->dataPtr->createButton, SIGNAL(clicked()), this,
      SLOT(OnCreate()));

  // Buttons layout
  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(this->dataPtr->createButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);

  // Gazebo event connections
  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectJointParentFrom3D(
      boost::bind(&JointCreationDialog::OnParentFrom3D, this, _1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectJointChildFrom3D(
      boost::bind(&JointCreationDialog::OnChildFrom3D, this, _1)));

  // Qt signal-slot connections
  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));
}

/////////////////////////////////////////////////
JointCreationDialog::~JointCreationDialog()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void JointCreationDialog::Open(JointMaker::JointType _type)
{
  if (!this->dataPtr->jointMaker)
  {
    gzerr << "Joint maker not found, can't open joint creation dialog."
        << std::endl;
    this->OnCancel();
  }

  // Check joint type
  this->dataPtr->typeButtons->button(static_cast<int>(_type))
      ->setChecked(true);

  // Reset enabled states
  this->dataPtr->createButton->setEnabled(false);
  this->dataPtr->swapButton->setEnabled(false);
  this->dataPtr->configWidget->SetWidgetReadOnly("childCombo", true);
  this->dataPtr->parentLinkWidget->setStyleSheet(
      this->dataPtr->activeStyleSheet);
  this->dataPtr->childLinkWidget->setStyleSheet(
      this->dataPtr->normalStyleSheet);

  // Clear links
  this->dataPtr->configWidget->ClearEnumWidget("parentCombo");
  this->dataPtr->configWidget->ClearEnumWidget("childCombo");

  // Add an empty option to each
  this->dataPtr->configWidget->AddItemEnumWidget("parentCombo", "");
  this->dataPtr->configWidget->AddItemEnumWidget("childCombo", "");

  // Fill with all existing links
  for (auto link : this->dataPtr->jointMaker->LinkList())
  {
    this->dataPtr->configWidget->AddItemEnumWidget("parentCombo", link.second);
    this->dataPtr->configWidget->AddItemEnumWidget("childCombo", link.second);
  }

  this->move(0, 0);
  this->show();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnTypeFromDialog(int _type)
{
  JointMaker::JointType type = static_cast<JointMaker::JointType>(_type);
  gui::model::Events::jointTypeFromDialog(type);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnLinkFromDialog()
{
  std::string currentParent =
      this->dataPtr->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->GetEnumWidgetValue("childCombo");

  // Notify so 3D is updated
  if (currentParent != currentChild)
  {
    if (currentParent != "")
      gui::model::Events::jointParentFromDialog(currentParent);
    if (currentChild != "")
      gui::model::Events::jointChildFromDialog(currentChild);
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
  gui::model::Events::jointPoseFromDialog(_pose, false);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnVector3dFromDialog(const QString &_name,
    const ignition::math::Vector3d &_value)
{
  int preset = 3;
  if (_value == ignition::math::Vector3d::UnitX)
    preset = 0;
  else if (_value == ignition::math::Vector3d::UnitY)
    preset = 1;
  else if (_value == ignition::math::Vector3d::UnitZ)
    preset = 2;

  if (_name == "axis1")
    this->dataPtr->axis1PresetsCombo->setCurrentIndex(preset);
  else if (_name == "axis2")
    this->dataPtr->axis2PresetsCombo->setCurrentIndex(preset);

  // Notify so 3D is updated
std::cout << _value << std::endl;
}

/////////////////////////////////////////////////
void JointCreationDialog::OnAxis1Presets(const QString &_axis)
{
  ignition::math::Vector3d vec;
  if (_axis == "X")
    vec = ignition::math::Vector3d::UnitX;
  else if (_axis == "Y")
    vec = ignition::math::Vector3d::UnitY;
  else if (_axis == "Z")
    vec = ignition::math::Vector3d::UnitZ;
  else
    return;

  // Update widget
  this->dataPtr->configWidget->SetVector3WidgetValue("axis1", vec);

  // Notify
  this->OnVector3dFromDialog("axis1", vec);
}

/////////////////////////////////////////////////
void JointCreationDialog::OnAxis2Presets(const QString &_axis)
{
  ignition::math::Vector3d vec;
  if (_axis == "X")
    vec = ignition::math::Vector3d::UnitX;
  else if (_axis == "Y")
    vec = ignition::math::Vector3d::UnitY;
  else if (_axis == "Z")
    vec = ignition::math::Vector3d::UnitZ;
  else
    return;

  // Update widget
  this->dataPtr->configWidget->SetVector3WidgetValue("axis2", vec);

  // Notify
  this->OnVector3dFromDialog("axis2", vec);
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

  this->dataPtr->configWidget->blockSignals(true);
  if (!this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo",
      leafName))
  {
    gzerr << "Requested link [" << leafName << "] not found" << std::endl;
    return;
  }
  this->dataPtr->configWidget->blockSignals(false);

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

  if (this->dataPtr->configWidget->GetWidgetReadOnly("childCombo"))
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

  this->dataPtr->configWidget->blockSignals(true);
  if (!this->dataPtr->configWidget->SetEnumWidgetValue("childCombo", leafName))
  {
    gzerr << "Requested link [" << leafName << "] not found" << std::endl;
    return;
  }
  this->dataPtr->configWidget->blockSignals(false);

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
  this->dataPtr->configWidget->RemoveItemEnumWidget("parentCombo", "");

  // Check if links are valid
  this->CheckLinksValid();

  // Enable child selection
  if (this->dataPtr->configWidget->GetWidgetReadOnly("childCombo"))
  {
    this->dataPtr->configWidget->SetWidgetReadOnly("childCombo", false);
    this->dataPtr->childLinkWidget->setStyleSheet(
        this->dataPtr->activeStyleSheet);
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
  this->dataPtr->createButton->setEnabled(true);
  this->dataPtr->swapButton->setEnabled(true);

  // Remove empty option
  this->dataPtr->configWidget->RemoveItemEnumWidget("childCombo", "");

  // Check if links are valid
  this->CheckLinksValid();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCancel()
{
  this->close();
  this->dataPtr->jointMaker->Stop();
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
      this->dataPtr->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->GetEnumWidgetValue("childCombo");

  // Choose new values
  this->dataPtr->configWidget->SetEnumWidgetValue("parentCombo", currentChild);
  this->dataPtr->configWidget->SetEnumWidgetValue("childCombo", currentParent);
}

/////////////////////////////////////////////////
void JointCreationDialog::UpdateRelativePose(
    const ignition::math::Pose3d &_pose)
{
  this->dataPtr->configWidget->SetPoseWidgetValue("pose", math::Pose(_pose));

  // TODO: Uncheck all align buttons if the pose is not satisfying alignment
  // How to know if the pose update is due to an align request coming from
  // here?
/*
  for (auto group : this->dataPtr->alignGroups)
  {
    if (group->checkedButton())
      group->checkedButton()->setChecked(false);
  }
*/
}

/////////////////////////////////////////////////
void JointCreationDialog::OnResetPoses()
{
  gui::model::Events::jointPoseFromDialog(ignition::math::Pose3d(), true);
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
      this->dataPtr->configWidget->GetEnumWidgetValue("parentCombo");
  std::string currentChild =
      this->dataPtr->configWidget->GetEnumWidgetValue("childCombo");

  // Warning if parent is the same as the child and don't allow creation
  if (currentParent == currentChild)
  {
    this->dataPtr->parentLinkWidget->setStyleSheet(
        this->dataPtr->warningStyleSheet);
    this->dataPtr->childLinkWidget->setStyleSheet(
        this->dataPtr->warningStyleSheet);
    this->dataPtr->createButton->setEnabled(false);
  }
  else
  {
    this->dataPtr->parentLinkWidget->setStyleSheet(
        this->dataPtr->normalStyleSheet);
    if (!this->dataPtr->configWidget->GetWidgetReadOnly("childCombo"))
    {
      this->dataPtr->childLinkWidget->setStyleSheet(
          this->dataPtr->normalStyleSheet);
      this->dataPtr->createButton->setEnabled(true);
    }
  }
}

/////////////////////////////////////////////////
void JointCreationDialog::OnAlign(const int _int)
{
  // Reset pose
  gui::model::Events::jointPoseFromDialog(ignition::math::Pose3d(), true);

  // Reference link
  bool childToParent = this->dataPtr->alignCombo->currentIndex() == 0;

  // Button groups
  std::vector<std::string> axes = {"x", "y", "z"};
  for (unsigned int g = 0; g < this->dataPtr->alignGroups.size(); ++g)
  {
    auto group = this->dataPtr->alignGroups[g];

    // Uncheck other buttons in the same group
    if (this->sender() == group)
    {
      for (int i = 0; i < group->buttons().size(); ++i)
      {
        if (i != _int)
        {
          group->buttons()[i]->setChecked(false);
        }
      }
    }

    // Emit event for the checked button of each group
    int checked = group->checkedId();
    if (checked == 0)
        gui::model::Events::alignJointLinks(childToParent, axes[g], "min");
    else if (checked == 1)
      gui::model::Events::alignJointLinks(childToParent, axes[g], "center");
    else if (checked == 2)
      gui::model::Events::alignJointLinks(childToParent, axes[g], "max");
  }
}
