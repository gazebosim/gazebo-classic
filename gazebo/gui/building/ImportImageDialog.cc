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
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/ImportImageView.hh"
#include "gazebo/gui/building/ImportImageViewPrivate.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"
#include "gazebo/gui/building/ImportImageDialogPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImportImageDialog::ImportImageDialog(QWidget *_parent)
  : QDialog(_parent), dataPtr(new ImportImageDialogPrivate)
{
  this->dataPtr->view = static_cast<EditorView *>(_parent);

  this->setWindowTitle(tr("Import Image"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Title
  QLabel *titleLabel = new QLabel(tr(
      "<b>Import image</b><br>"
      "Import a building floorplan<br>"));

  // Step 1
  QLabel *step1Label = new QLabel(tr(
      "<b>Step 1: Select Image</b>"));

  QLabel *step1Supports = new QLabel(tr(
       "<font size=1 color='grey'>Supported formats: .png, .jpg</font><br>"));

  this->dataPtr->fileLineEdit = new QLineEdit();
  this->dataPtr->fileLineEdit->setPlaceholderText(tr("Image file name"));
  connect(this, SIGNAL(SetFileName(QString)),
      this->dataPtr->fileLineEdit, SLOT(setText(QString)),
      Qt::QueuedConnection);

  QPushButton *fileButton = new QPushButton(tr("..."));
  connect(fileButton, SIGNAL(clicked()), this, SLOT(OnSelectFile()));

  QHBoxLayout *fileLayout = new QHBoxLayout;
  fileLayout->addWidget(new QLabel(tr("File: ")));
  fileLayout->addWidget(this->dataPtr->fileLineEdit);
  fileLayout->addWidget(fileButton);

  QPushButton *cancelButton1 = new QPushButton(tr("Cancel"));
  this->dataPtr->nextButton = new QPushButton(tr("Next"));
  this->dataPtr->nextButton->setEnabled(false);
  connect(cancelButton1, SIGNAL(clicked()), this, SLOT(OnReject()));
  connect(this->dataPtr->nextButton, SIGNAL(clicked()), this, SLOT(OnNext()));

  QHBoxLayout *step1Buttons = new QHBoxLayout;
  step1Buttons->addWidget(cancelButton1);
  step1Buttons->addWidget(this->dataPtr->nextButton);

  QVBoxLayout *step1Layout = new QVBoxLayout;
  step1Layout->setSpacing(0);
  step1Layout->addWidget(step1Label);
  step1Layout->addLayout(fileLayout);
  step1Layout->addWidget(step1Supports);
  step1Layout->addLayout(step1Buttons);

  QWidget *step1Widget = new QWidget();
  step1Widget->setLayout(step1Layout);

  // Step 2
  QLabel *step2Label = new QLabel(tr(
      "<b>Step 2: Set Scale</b><br>"
      "Draw a line on the image to set<br>"
      "the real world distance between<br>"
      "the two end points."));

  this->dataPtr->distanceSpin = new QDoubleSpinBox;
  this->dataPtr->distanceSpin->setRange(0.001, 1000);
  this->dataPtr->distanceSpin->setSingleStep(0.1);
  this->dataPtr->distanceSpin->setDecimals(4);
  this->dataPtr->distanceSpin->setValue(1);
  this->dataPtr->distanceSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  this->dataPtr->distanceSpin->setReadOnly(true);
  connect(this->dataPtr->distanceSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnChangeDistance(double)));

  QHBoxLayout *distanceLayout = new QHBoxLayout;
  distanceLayout->addWidget(new QLabel("Distance (m):"));
  distanceLayout->addStretch(1);
  distanceLayout->addWidget(this->dataPtr->distanceSpin);

  this->dataPtr->resolutionSpin = new QDoubleSpinBox;
  this->dataPtr->resolutionSpin->setRange(0, 10000);
  this->dataPtr->resolutionSpin->setSingleStep(10);
  this->dataPtr->resolutionSpin->setDecimals(3);
  this->dataPtr->resolutionSpin->setValue(100);
  this->dataPtr->resolutionSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  this->dataPtr->resolutionSpin->setReadOnly(true);
  connect(this->dataPtr->resolutionSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnChangeResolution(double)));

  QHBoxLayout *resolutionLayout = new QHBoxLayout;
  resolutionLayout->addWidget(new QLabel("Resolution (px/m):"));
  resolutionLayout->addStretch(1);
  resolutionLayout->addWidget(this->dataPtr->resolutionSpin);

  this->dataPtr->okButton = new QPushButton(tr("Ok"));
  this->dataPtr->okButton->setEnabled(false);
  QPushButton *backButton = new QPushButton(tr("Back"));
  QPushButton *cancelButton2 = new QPushButton(tr("Cancel"));

  connect(this->dataPtr->okButton, SIGNAL(clicked()), this, SLOT(OnAccept()));
  connect(backButton, SIGNAL(clicked()), this, SLOT(OnBack()));
  connect(cancelButton2, SIGNAL(clicked()), this, SLOT(OnReject()));

  QHBoxLayout *step2Buttons = new QHBoxLayout;
  step2Buttons->addWidget(backButton);
  step2Buttons->addWidget(cancelButton2);
  step2Buttons->addWidget(this->dataPtr->okButton);

  QVBoxLayout *step2Layout = new QVBoxLayout;
  step2Layout->addWidget(step2Label);
  step2Layout->addSpacing(20);
  step2Layout->addLayout(distanceLayout);
  step2Layout->addLayout(resolutionLayout);
  step2Layout->addSpacing(40);
  step2Layout->addLayout(step2Buttons);

  QWidget *step2Widget = new QWidget();
  step2Widget->setLayout(step2Layout);

  // Left column
  this->dataPtr->stackedStepLayout = new QStackedLayout;
  this->dataPtr->stackedStepLayout->addWidget(step1Widget);
  this->dataPtr->stackedStepLayout->addWidget(step2Widget);

  QWidget *leftColumn = new QWidget();
  leftColumn->setSizePolicy(QSizePolicy::Fixed,
                            QSizePolicy::Fixed);
  QVBoxLayout *leftColumnLayout = new QVBoxLayout();
  leftColumn->setLayout(leftColumnLayout);
  leftColumnLayout->addWidget(titleLabel);
  leftColumnLayout->addLayout(this->dataPtr->stackedStepLayout);

  // Image view
  this->dataPtr->importImageView = new ImportImageView(this);
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

  // Main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(leftColumn, 0, Qt::AlignTop);
  mainLayout->addWidget(this->dataPtr->importImageView);
  this->setLayout(mainLayout);

  this->dataPtr->drawingLine = false;
}

/////////////////////////////////////////////////
ImportImageDialog::~ImportImageDialog()
{
}

/////////////////////////////////////////////////
void ImportImageDialog::OnAccept()
{
  std::string filename = this->dataPtr->fileLineEdit->text().toStdString();
  if (!filename.empty())
  {
    this->dataPtr->view->SetBackgroundImage(filename,
        this->dataPtr->resolutionSpin->value());
  }
  this->accept();
}

/////////////////////////////////////////////////
void ImportImageDialog::OnReject()
{
  gui::editor::Events::createBuildingEditorItem(std::string());
  this->reject();
}

/////////////////////////////////////////////////
void ImportImageDialog::OnNext()
{
  this->dataPtr->stackedStepLayout->setCurrentIndex(1);
  this->dataPtr->importImageView->EnableDrawDistance(true);
}

/////////////////////////////////////////////////
void ImportImageDialog::OnBack()
{
  this->dataPtr->stackedStepLayout->setCurrentIndex(0);
  this->dataPtr->importImageView->EnableDrawDistance(false);
}

/////////////////////////////////////////////////
void ImportImageDialog::OnSelectFile()
{
  QFileDialog fileDialog(this, tr("Open Image"), QDir::homePath(),
      tr("Image Files (*.png *.jpg *.jpeg)"));
  fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  if (fileDialog.exec() == QDialog::Accepted)
  {
    QStringList selected = fileDialog.selectedFiles();
    if (selected.empty())
      return;

    std::string filename = selected[0].toStdString();

    this->SetFileName(QString::fromStdString(filename));
    this->dataPtr->importImageView->SetImage(filename);

    this->dataPtr->nextButton->setEnabled(true);
  }
}

/////////////////////////////////////////////////
void ImportImageDialog::OnChangeDistance(double _distance)
{
  double distanceImage =
      this->dataPtr->importImageView->dataPtr->measureScenePx *
      this->dataPtr->importImageView->dataPtr->imageWidthPx /
      this->dataPtr->importImageView->dataPtr->pixmapWidthPx;
  this->dataPtr->resolutionSpin->setValue(distanceImage / _distance);
  this->dataPtr->importImageView->RefreshDistance(_distance);

  this->dataPtr->okButton->setEnabled(true);
}

/////////////////////////////////////////////////
void ImportImageDialog::OnChangeResolution(double /*_resolution*/)
{
  // change distance without re-changing resolution
  this->dataPtr->okButton->setEnabled(true);
}
