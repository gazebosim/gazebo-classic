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
      "Extrude a 2D polyline into a 3D mesh.<br>"));

  // Thickness
  this->dataPtr->thicknessSpin = new QDoubleSpinBox();
  this->dataPtr->thicknessSpin->setRange(0.001, 1000);
  this->dataPtr->thicknessSpin->setSingleStep(0.1);
  this->dataPtr->thicknessSpin->setDecimals(4);
  this->dataPtr->thicknessSpin->setValue(1);

  // Resolution
  this->dataPtr->resolutionSpin = new QDoubleSpinBox();
  this->dataPtr->resolutionSpin->setRange(1, 100000);
  this->dataPtr->resolutionSpin->setSingleStep(10);
  this->dataPtr->resolutionSpin->setDecimals(3);
  // 3543.3 px/m == 90 dpi
  this->dataPtr->resolutionSpin->setValue(3543.3);

  // Samples
  this->dataPtr->samplesSpin = new QSpinBox();
  this->dataPtr->samplesSpin->setRange(2, 100);
  this->dataPtr->samplesSpin->setSingleStep(1);
  this->dataPtr->samplesSpin->setValue(5);
  QLabel *samplesTips = new QLabel(tr("<b><font size=4>?</font></b>"));
  samplesTips->setToolTip(
      "Number of points to divide each curve segment into.");

  QGridLayout *inputsLayout = new QGridLayout();
  inputsLayout->addWidget(new QLabel("Thickness:"), 0, 0);
  inputsLayout->addWidget(this->dataPtr->thicknessSpin, 0, 1);
  inputsLayout->addWidget(new QLabel("m"), 0, 2);
  inputsLayout->addWidget(new QLabel("Resolution:"), 1, 0);
  inputsLayout->addWidget(this->dataPtr->resolutionSpin, 1, 1);
  inputsLayout->addWidget(new QLabel("px/m"), 1, 2);
  inputsLayout->addWidget(new QLabel("Samples per segment:"), 2, 0);
  inputsLayout->addWidget(this->dataPtr->samplesSpin, 2, 1);
  inputsLayout->addWidget(samplesTips, 2, 2);

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
  leftColumnLayout->addLayout(inputsLayout);
  leftColumnLayout->addSpacing(30);
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
  QGraphicsPixmapItem *imageItem = new QGraphicsPixmapItem(
      imagePixmap->scaled(scene->sceneRect().width(),
      scene->sceneRect().height(), Qt::KeepAspectRatio));
  scene->addItem(imageItem);

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
