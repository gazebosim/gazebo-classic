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

//  // Unit
//  QLabel *unitLabel = new QLabel(tr(
//      "<b>Unit</b><br>"
//      "Choose the units used in your image file."));

//  this->dataPtr->unitCombo = new QComboBox();
//  this->dataPtr->unitCombo->setMinimumWidth(100);
//  this->dataPtr->unitCombo->addItem(tr("m"));
//  this->dataPtr->unitCombo->addItem(tr("px"));
//  this->dataPtr->unitCombo->addItem(tr("in"));
////  connect(this->dataPtr->unitCombo, SIGNAL(currentIndexChanged(QString)), this,
////      SLOT(OnChangeUnit(QString)));

//  QHBoxLayout *unitComboLayout = new QHBoxLayout;
//  unitComboLayout->addWidget(new QLabel("Unit:"));
//  unitComboLayout->addStretch(1);
//  unitComboLayout->addWidget(this->dataPtr->unitCombo);

//  QVBoxLayout *unitLayout = new QVBoxLayout;
//  unitLayout->addWidget(unitLabel);
//  unitLayout->addSpacing(20);
//  unitLayout->addLayout(unitComboLayout);

//  QWidget *unitWidget = new QWidget();
//  unitWidget->setLayout(unitLayout);

  // Thickness
  this->dataPtr->thicknessSpin = new QDoubleSpinBox;
  this->dataPtr->thicknessSpin->setRange(0.001, 1000);
  this->dataPtr->thicknessSpin->setSingleStep(0.1);
  this->dataPtr->thicknessSpin->setDecimals(4);
  this->dataPtr->thicknessSpin->setValue(1);
//  connect(this->dataPtr->thicknessSpin, SIGNAL(valueChanged(double)), this,
//      SLOT(OnChangeThickness(double)));

  QHBoxLayout *thicknessLayout = new QHBoxLayout;
  thicknessLayout->addWidget(new QLabel("Thickness:"));
  thicknessLayout->addStretch(1);
  thicknessLayout->addWidget(this->dataPtr->thicknessSpin);
  thicknessLayout->addWidget(new QLabel("m"));

  // Resolution
  this->dataPtr->resolutionSpin = new QDoubleSpinBox;
  this->dataPtr->resolutionSpin->setRange(1, 1000);
  this->dataPtr->resolutionSpin->setSingleStep(10);
  this->dataPtr->resolutionSpin->setDecimals(3);
  this->dataPtr->resolutionSpin->setValue(300);
//  connect(this->dataPtr->resolutionSpin, SIGNAL(valueChanged(double)), this,
//      SLOT(OnChangeResolution(double)));

  QHBoxLayout *resolutionLayout = new QHBoxLayout;
  resolutionLayout->addWidget(new QLabel("Resolution:"));
  resolutionLayout->addStretch(1);
  resolutionLayout->addWidget(this->dataPtr->resolutionSpin);
  resolutionLayout->addWidget(new QLabel("px/m"));

  // Samples
  this->dataPtr->samplesSpin = new QDoubleSpinBox;
  this->dataPtr->samplesSpin->setRange(2, 100);
  this->dataPtr->samplesSpin->setSingleStep(1);
  this->dataPtr->samplesSpin->setDecimals(0);
  this->dataPtr->samplesSpin->setValue(5);
//  connect(this->dataPtr->samplesSpin, SIGNAL(valueChanged(double)), this,
//      SLOT(OnChangeSamples(double)));

  QHBoxLayout *samplesLayout = new QHBoxLayout;
  samplesLayout->addWidget(new QLabel("# of samples per segment:"));
  samplesLayout->addStretch(1);
  samplesLayout->addWidget(this->dataPtr->samplesSpin);

  // Buttons
  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  QPushButton *backButton = new QPushButton(tr("Back"));
  connect(backButton, SIGNAL(clicked()), this, SLOT(OnReject()));

  QPushButton *okButton = new QPushButton("Ok");
  okButton->setDefault(true);
  connect(okButton, SIGNAL(clicked()), this, SLOT(OnAccept()));
  buttonsLayout->addWidget(backButton);
  buttonsLayout->addWidget(okButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Left column
  QWidget *leftColumn = new QWidget();
  leftColumn->setSizePolicy(QSizePolicy::Fixed,
                            QSizePolicy::Fixed);
  QVBoxLayout *leftColumnLayout = new QVBoxLayout();
  leftColumn->setLayout(leftColumnLayout);
  leftColumnLayout->addWidget(titleLabel);
  leftColumnLayout->addLayout(thicknessLayout);
  leftColumnLayout->addLayout(resolutionLayout);
  leftColumnLayout->addLayout(samplesLayout);
  leftColumnLayout->addLayout(buttonsLayout);

  // Image view
  this->dataPtr->importImageView = new QGraphicsView(this);
  QGraphicsScene *scene = new QGraphicsScene();
  scene->setBackgroundBrush(Qt::white);

  this->dataPtr->imageDisplayWidth = 300;
  this->dataPtr->imageDisplayHeight = 300;
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

/////////////////////////////////////////////////
void ExtrudeDialog::OnAccept()
{
  this->accept();
}

/////////////////////////////////////////////////
void ExtrudeDialog::OnReject()
{
  this->reject();
}

/////////////////////////////////////////////////
double ExtrudeDialog::GetThickness() const
{
  return this->dataPtr->thicknessSpin->value();
}

/////////////////////////////////////////////////
unsigned int ExtrudeDialog::GetSamples() const
{
  return this->dataPtr->samplesSpin->value();
}

/////////////////////////////////////////////////
unsigned int ExtrudeDialog::GetResolution() const
{
  return this->dataPtr->resolutionSpin->value();
}
