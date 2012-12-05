/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "WindowDoorInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowDoorInspectorDialog::WindowDoorInspectorDialog(int _type,
  QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("windowDoorInspectorDialog");


  this->modelType = _type;

  if (this->modelType == WINDOW)
    this->modelTypeStr = "Window";
  else if (this->modelType == DOOR)
    this->modelTypeStr = "Door";

  std::string titleStr = modelTypeStr + " Inspector";
  this->setWindowTitle(tr(titleStr.c_str()));

  std::string modelLabelStr = modelTypeStr + " Name:";
  QLabel *modelLabel = new QLabel(tr(modelLabelStr.c_str()));
  this->modelNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(modelLabel);
  nameLayout->addWidget(modelNameLabel);

  QLabel *widthLabel = new QLabel(tr("Width: "));
  QLabel *lengthLabel = new QLabel(tr("Length: "));
  QLabel *heightLabel = new QLabel(tr("Height: "));

  QSpinBox *widthSpinBox = new QSpinBox;
  QSpinBox *lengthSpinBox = new QSpinBox;
  QSpinBox *heightSpinBox = new QSpinBox;

  QGridLayout *sizeLayout = new QGridLayout;
  sizeLayout->addWidget(widthLabel, 0, 0);
  sizeLayout->addWidget(widthSpinBox, 0, 1);
  sizeLayout->addWidget(lengthLabel, 1, 0);
  sizeLayout->addWidget(lengthSpinBox, 1, 1);
  sizeLayout->addWidget(heightLabel, 2, 0);
  sizeLayout->addWidget(heightSpinBox, 2, 1);

  QGroupBox *sizeGroupBox = new QGroupBox(tr("Size"));
  sizeGroupBox->setLayout(sizeLayout);

  QLabel *positionXLabel = new QLabel(tr("x: "));
  QLabel *positionYLabel = new QLabel(tr("y: "));

  QSpinBox *positionXSpinBox = new QSpinBox;
  QSpinBox *positionYSpinBox = new QSpinBox;

  QHBoxLayout *positionLayout = new QHBoxLayout;
  positionLayout->addWidget(positionXLabel);
  positionLayout->addWidget(positionXSpinBox);
  positionLayout->addWidget(positionYLabel);
  positionLayout->addWidget(positionYSpinBox);

  QGroupBox *positionGroupBox = new QGroupBox(tr("Position"));
  positionGroupBox->setLayout(positionLayout);

  QLabel *typeLabel = new QLabel(tr("Type: "));
  QComboBox *typeComboBox = new QComboBox;
  typeComboBox->addItem(QString("Single"));

  QHBoxLayout *typeLayout = new QHBoxLayout;
  typeLayout->addWidget(typeLabel);
  typeLayout->addWidget(typeComboBox);

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *OKButton = new QPushButton(tr("&OK"));
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);


  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(nameLayout);
  mainLayout->addWidget(sizeGroupBox);
  mainLayout->addWidget(positionGroupBox);
  mainLayout->addLayout(typeLayout);
  mainLayout->addLayout(buttonsLayout);



  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
WindowDoorInspectorDialog::~WindowDoorInspectorDialog()
{
}


/////////////////////////////////////////////////
void WindowDoorInspectorDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void WindowDoorInspectorDialog::OnOK()
{
  /// TODO:
  this->accept();
}
