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

#include "gazebo/gui/model/PartVisualTab.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PartVisualTab::PartVisualTab()
{
  this->setObjectName("partVisualTab");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->visualsTreeWidget = new QTreeWidget();
  this->visualsTreeWidget->setColumnCount(1);
  this->visualsTreeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
  this->visualsTreeWidget->header()->hide();


  QPushButton *addVisualButton = new QPushButton(tr("+ &Add Visual"));
  connect(addVisualButton, SIGNAL(clicked()), this, SLOT(OnAddVisual()));
//  buttonsLayout->addWidget(cancelButton);

  mainLayout->addWidget(this->visualsTreeWidget);
  mainLayout->addWidget(addVisualButton);

  this->setLayout(mainLayout);

  this->OnAddVisual();
}

/////////////////////////////////////////////////
PartVisualTab::~PartVisualTab()
{
}

/////////////////////////////////////////////////
void PartVisualTab::OnAddVisual()
{
  // Create a top-level tree item for the path
  QTreeWidgetItem *visualItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0),
        QStringList(QString("Visual ")));
  this->visualsTreeWidget->addTopLevelItem(visualItem);

  QTreeWidgetItem *visualChildItem =
    new QTreeWidgetItem(static_cast<QTreeWidgetItem*>(0));
  visualItem->addChild(visualChildItem);

  QWidget *visualWidget = new QWidget;
  QVBoxLayout *visualLayout = new QVBoxLayout;

  QLabel *nameLabel = new QLabel(tr("Name:"));
  QLabel *visualNameLabel = new QLabel(tr(""));

  QLabel *geometryLabel = new QLabel(tr("Geometry:"));
  QComboBox *geometryComboBox = new QComboBox;
  geometryComboBox->addItem(tr("box"));
  geometryComboBox->addItem(tr("cylinder"));
  geometryComboBox->addItem(tr("sphere"));
  geometryComboBox->addItem(tr("custom"));

  QLabel *transparencyLabel = new QLabel(tr("Transparency:"));
  QDoubleSpinBox *transparencySpinBox = new QDoubleSpinBox;
  transparencySpinBox->setRange(-1000, 1000);
  transparencySpinBox->setSingleStep(0.01);
  transparencySpinBox->setDecimals(3);
  transparencySpinBox->setValue(0.000);

  QLabel *materialLabel = new QLabel(tr("Material:"));
  QLineEdit *materialLineEdit = new QLineEdit;

  QGridLayout *visualGeneralLayout = new QGridLayout;
  visualGeneralLayout->addWidget(nameLabel, 0, 0);
  visualGeneralLayout->addWidget(visualNameLabel, 0, 1);
  visualGeneralLayout->addWidget(geometryLabel, 1, 0);
  visualGeneralLayout->addWidget(geometryComboBox, 1, 1);
  visualGeneralLayout->addWidget(transparencyLabel, 2, 0);
  visualGeneralLayout->addWidget(transparencySpinBox, 2, 1);
  visualGeneralLayout->addWidget(materialLabel, 3, 0);
  visualGeneralLayout->addWidget(materialLineEdit, 3, 1);

  QLabel *posXLabel = new QLabel(tr("x: "));
  QLabel *posYLabel = new QLabel(tr("y: "));
  QLabel *posZLabel = new QLabel(tr("z: "));
  QLabel *rotRLabel = new QLabel(tr("roll: "));
  QLabel *rotPLabel = new QLabel(tr("pitch: "));
  QLabel *rotYLabel = new QLabel(tr("yaw: "));

  QDoubleSpinBox *posXSpinBox = new QDoubleSpinBox;
  posXSpinBox->setRange(-1000, 1000);
  posXSpinBox->setSingleStep(0.01);
  posXSpinBox->setDecimals(3);
  posXSpinBox->setValue(0.000);

  QDoubleSpinBox *posYSpinBox = new QDoubleSpinBox;
  posYSpinBox->setRange(-1000, 1000);
  posYSpinBox->setSingleStep(0.01);
  posYSpinBox->setDecimals(3);
  posYSpinBox->setValue(0.000);

  QDoubleSpinBox *posZSpinBox = new QDoubleSpinBox;
  posZSpinBox->setRange(-1000, 1000);
  posZSpinBox->setSingleStep(0.01);
  posZSpinBox->setDecimals(3);
  posZSpinBox->setValue(0.000);

  QDoubleSpinBox *rotRSpinBox = new QDoubleSpinBox;
  rotRSpinBox->setRange(-1000, 1000);
  rotRSpinBox->setSingleStep(0.01);
  rotRSpinBox->setDecimals(3);
  rotRSpinBox->setValue(0.000);

  QDoubleSpinBox *rotPSpinBox = new QDoubleSpinBox;
  rotPSpinBox->setRange(-1000, 1000);
  rotPSpinBox->setSingleStep(0.01);
  rotPSpinBox->setDecimals(3);
  rotPSpinBox->setValue(0.000);

  QDoubleSpinBox *rotYSpinBox = new QDoubleSpinBox;
  rotYSpinBox->setRange(-1000, 1000);
  rotYSpinBox->setSingleStep(0.01);
  rotYSpinBox->setDecimals(3);
  rotYSpinBox->setValue(0.000);

  QGridLayout *poseGroupLayout = new QGridLayout;
  poseGroupLayout->addWidget(posXLabel, 0, 0);
  poseGroupLayout->addWidget(posXSpinBox, 0, 1);
  poseGroupLayout->addWidget(posYLabel, 0, 2);
  poseGroupLayout->addWidget(posYSpinBox, 0, 3);
  poseGroupLayout->addWidget(posZLabel, 0, 4);
  poseGroupLayout->addWidget(posZSpinBox, 0, 5);
  poseGroupLayout->addWidget(rotRLabel, 1, 0);
  poseGroupLayout->addWidget(rotRSpinBox, 1, 1);
  poseGroupLayout->addWidget(rotPLabel, 1, 2);
  poseGroupLayout->addWidget(rotPSpinBox, 1, 3);
  poseGroupLayout->addWidget(rotYLabel, 1, 4);
  poseGroupLayout->addWidget(rotYSpinBox, 1, 5);

  poseGroupLayout->setColumnStretch(1, 1);
  poseGroupLayout->setAlignment(posXSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(posYSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(posZSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(rotRSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(rotPSpinBox, Qt::AlignLeft);
  poseGroupLayout->setAlignment(rotYSpinBox, Qt::AlignLeft);

  QGroupBox *poseGroupBox = new QGroupBox(tr("Pose"));
  poseGroupBox->setLayout(poseGroupLayout);

  visualLayout->addLayout(visualGeneralLayout);
  visualLayout->addWidget(poseGroupBox);

  visualWidget->setLayout(visualLayout);
  this->visualsTreeWidget->setItemWidget(visualChildItem, 0,
    visualWidget);
  visualItem->setExpanded(true);
  visualChildItem->setExpanded(true);
}
