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

//#include "gazebo/common/Console.hh"
//#include "gazebo/common/Assert.hh"

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/JointCreationDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointCreationDialog::JointCreationDialog(JointMaker *_jointMaker,
    QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("JointCreationDialogDialog");
  this->setWindowTitle(tr("Create a joint"));
  this->setWindowFlags(Qt::WindowStaysOnTopHint);

  this->jointMaker = _jointMaker;

  this->configWidget = new ConfigWidget();

  // Joint types
  QRadioButton *fixedJointRadio = new QRadioButton(tr("Fixed"));
  QRadioButton *prismaticJointRadio = new QRadioButton(tr("Prismatic"));
  QRadioButton *revoluteJointRadio = new QRadioButton(tr("Revolute"));
  QRadioButton *revolute2JointRadio = new QRadioButton(tr("Revolute2"));
  QRadioButton *screwJointRadio = new QRadioButton(tr("Screw"));
  QRadioButton *universalJointRadio = new QRadioButton(tr("Universal"));
  QRadioButton *ballJointRadio = new QRadioButton(tr("Ball"));

  this->typeButtons = new QButtonGroup();
  this->typeButtons->addButton(fixedJointRadio, 1);
  this->typeButtons->addButton(prismaticJointRadio, 2);
  this->typeButtons->addButton(revoluteJointRadio, 3);
  this->typeButtons->addButton(revolute2JointRadio, 4);
  this->typeButtons->addButton(screwJointRadio, 5);
  this->typeButtons->addButton(universalJointRadio, 6);
  this->typeButtons->addButton(ballJointRadio, 7);
  connect(this->typeButtons, SIGNAL(buttonClicked(int)),
      this, SLOT(OnTypeFromDialog(int)));

  QVBoxLayout *typesLayout = new QVBoxLayout();
  typesLayout->addWidget(fixedJointRadio);
  typesLayout->addWidget(revoluteJointRadio);
  typesLayout->addWidget(revolute2JointRadio);
  typesLayout->addWidget(prismaticJointRadio);
  typesLayout->addWidget(screwJointRadio);
  typesLayout->addWidget(universalJointRadio);
  typesLayout->addWidget(ballJointRadio);

  ConfigChildWidget *typesWidget = new ConfigChildWidget();
  typesWidget->setLayout(typesLayout);

  QWidget *typesGroupWidget =
      this->configWidget->CreateGroupWidget("Joint types", typesWidget, 0);

  // Link selections
  QLabel *selectionsText = new QLabel(tr(
      "Click a link in the scene "
       "to select parent. Click "
       "again to select child."));

  std::vector<std::string> links
  {
    "Link 1",
    "Link 2",
    "Link 3"
  };

  ConfigChildWidget *parentComboBox =
      this->configWidget->CreateEnumWidget("Parent: ", links, 0);

  ConfigChildWidget *childComboBox =
      this->configWidget->CreateEnumWidget("Child: ", links, 0);


  QVBoxLayout *linksLayout = new QVBoxLayout();
  linksLayout->addWidget(selectionsText);
  linksLayout->addWidget(parentComboBox);
  linksLayout->addWidget(childComboBox);

  ConfigChildWidget *linksWidget = new ConfigChildWidget();
  linksWidget->setLayout(linksLayout);

  QWidget *linksGroupWidget = this->configWidget->CreateGroupWidget(
      "Link Selections", linksWidget, 0);



  QVBoxLayout *configLayout = new QVBoxLayout();
  configLayout->addWidget(typesGroupWidget);
  configLayout->addWidget(linksGroupWidget);

  this->configWidget->setLayout(configLayout);






  QScrollArea *scrollArea = new QScrollArea;
  scrollArea->setWidget(configWidget);
  scrollArea->setWidgetResizable(true);


  QVBoxLayout *generalLayout = new QVBoxLayout;
  generalLayout->setContentsMargins(0, 0, 0, 0);
  generalLayout->addWidget(scrollArea);



  QHBoxLayout *buttonsLayout = new QHBoxLayout;

  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *createButton = new QPushButton(tr("Create"));
  createButton->setDefault(true);
  connect(createButton, SIGNAL(clicked()), this, SLOT(OnCreate()));

  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(createButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(generalLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setMinimumWidth(500);
  this->setMinimumHeight(300);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
void JointCreationDialog::Open(JointMaker::JointType _type)
{
  this->typeButtons->button(static_cast<int>(_type))->setChecked(true);

  this->open();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnTypeFromDialog(int _type)
{
}

/////////////////////////////////////////////////
void JointCreationDialog::OnParentFromDialog(const std::string &_linkName)
{
}

/////////////////////////////////////////////////
void JointCreationDialog::OnChildFromDialog(
    const std::string &_linkName)
{
}

/////////////////////////////////////////////////
void JointCreationDialog::OnParentFrom3D(
    const std::string &_linkName)
{
}

/////////////////////////////////////////////////
void JointCreationDialog::OnChildFrom3D(const std::string &_linkName)
{
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void JointCreationDialog::OnCreate()
{
  this->accept();
}

/////////////////////////////////////////////////
void JointCreationDialog::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}
