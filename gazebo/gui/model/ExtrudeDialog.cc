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

#include "gazebo/gui/model/ExtrudeDialogPrivate.hh"
#include "gazebo/gui/model/ExtrudeDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ExtrudeDialog::ExtrudeDialog(std::string _filename, QWidget *_parent)
  : QDialog(_parent), dataPtr(new ExtrudeDialogPrivate)
{
  this->setObjectName("ExtrudeDialog");
  this->setWindowTitle(tr("Extrude Link"));

  // Title
  QLabel *titleLabel = new QLabel(tr(
      "<b>Extrude Link</b><br>"
      "Extrude a 2D image into a 3D mesh<br>"));

  // Unit
  QLabel *unitLabel = new QLabel(tr(
      "<b>Unit</b><br>"
      "Choose the units used in your image file."));

  this->dataPtr->unitCombo = new QComboBox();
  this->dataPtr->unitCombo->setMinimumWidth(100);
  this->dataPtr->unitCombo->addItem(tr("m"));
  this->dataPtr->unitCombo->addItem(tr("px"));
  this->dataPtr->unitCombo->addItem(tr("in"));
//  connect(this->dataPtr->unitCombo, SIGNAL(valueChanged(double)), this,
//      SLOT(OnChangeUnit(double)));

  QHBoxLayout *unitComboLayout = new QHBoxLayout;
  unitComboLayout->addWidget(new QLabel("Unit:"));
  unitComboLayout->addStretch(1);
  unitComboLayout->addWidget(this->dataPtr->unitCombo);

  QVBoxLayout *unitLayout = new QVBoxLayout;
  unitLayout->addWidget(unitLabel);
  unitLayout->addSpacing(20);
  unitLayout->addLayout(unitComboLayout);

  QWidget *unitWidget = new QWidget();
  unitWidget->setLayout(unitLayout);

  // Thickness
  QLabel *thicknessLabel = new QLabel(tr(
      "<b>Thickness</b><br>"
      "Choose the extrusion thickness."));

  this->dataPtr->thicknessSpin = new QDoubleSpinBox;
  this->dataPtr->thicknessSpin->setRange(0.001, 1000);
  this->dataPtr->thicknessSpin->setSingleStep(0.1);
  this->dataPtr->thicknessSpin->setDecimals(4);
  this->dataPtr->thicknessSpin->setValue(1);
//  connect(this->dataPtr->thicknessSpin, SIGNAL(valueChanged(double)), this,
//      SLOT(OnChangeThickness(double)));

  QHBoxLayout *thicknessSpinLayout = new QHBoxLayout;
  thicknessSpinLayout->addWidget(new QLabel("Thickness (m):"));
  thicknessSpinLayout->addStretch(1);
  thicknessSpinLayout->addWidget(this->dataPtr->thicknessSpin);

  QVBoxLayout *thicknessLayout = new QVBoxLayout;
  thicknessLayout->addWidget(thicknessLabel);
  thicknessLayout->addSpacing(20);
  thicknessLayout->addLayout(thicknessSpinLayout);

  QWidget *thicknessWidget = new QWidget();
  thicknessWidget->setLayout(thicknessLayout);

  // Buttons
  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
//  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *okButton = new QPushButton("Ok");
  okButton->setDefault(true);
//  connect(okButton, SIGNAL(clicked()), this, SLOT(OnOk()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(okButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Left column
  QWidget *leftColumn = new QWidget();
  leftColumn->setSizePolicy(QSizePolicy::Fixed,
                            QSizePolicy::Fixed);
  QVBoxLayout *leftColumnLayout = new QVBoxLayout();
  leftColumn->setLayout(leftColumnLayout);
  leftColumnLayout->addWidget(titleLabel);
  leftColumnLayout->addWidget(unitWidget);
  leftColumnLayout->addWidget(thicknessWidget);
  leftColumnLayout->addLayout(buttonsLayout);

  // Image view
  this->dataPtr->importImageView = new QGraphicsView(this);
  QGraphicsScene *scene = new QGraphicsScene();
  scene->setBackgroundBrush(Qt::white);

  this->dataPtr->imageDisplayWidth = 700;
  this->dataPtr->imageDisplayHeight = 500;
  scene->setSceneRect(0, 0, this->dataPtr->imageDisplayWidth,
                            this->dataPtr->imageDisplayHeight);

  this->dataPtr->importImageView->setSizePolicy(QSizePolicy::Expanding,
                                       QSizePolicy::Expanding);
  this->dataPtr->importImageView->setScene(scene);
  this->dataPtr->importImageView->centerOn(QPointF(0, 0));
  this->dataPtr->importImageView->setViewportUpdateMode(
      QGraphicsView::FullViewportUpdate);
  this->dataPtr->importImageView->setDragMode(QGraphicsView::ScrollHandDrag);

  QPixmap *imagePixmap = new QPixmap(QString::fromStdString(_filename));
//  this->imageWidthPx = this->imagePixmap->width();
  QGraphicsPixmapItem *imageItem = new QGraphicsPixmapItem(
      imagePixmap->scaled(scene->sceneRect().width(),
      scene->sceneRect().height(), Qt::KeepAspectRatio));

//  this->pixmapWidthPx = this->imageItem->pixmap().width();
//  this->pixmapHeightPx = this->imageItem->pixmap().height();

  if (imageItem)
  {
    scene->addItem(imageItem);
  }

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(leftColumn, 0, Qt::AlignTop);
  mainLayout->addWidget(this->dataPtr->importImageView);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
ExtrudeDialog::~ExtrudeDialog()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}
