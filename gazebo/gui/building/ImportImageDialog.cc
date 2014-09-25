/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/ImportImageDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImportImageDialog::ImportImageDialog(QWidget *_parent)
  : QDialog(_parent)
{
  this->view = static_cast<EditorView*>(_parent);

  this->setWindowTitle(tr("Import Image"));

  QLabel *titleWidget = new QLabel(tr(
          "<b>Import image</b><br>"
          "Import a building floorplan<br>"));

  this->fileLineEdit = new QLineEdit();
  this->fileLineEdit->setPlaceholderText(tr("Image file name"));
  connect(this, SIGNAL(SetFileName(QString)),
      this->fileLineEdit, SLOT(setText(QString)), Qt::QueuedConnection);

  QPushButton *fileButton = new QPushButton(tr("..."));
  connect(fileButton, SIGNAL(clicked()), this, SLOT(OnSelectFile()));

  QHBoxLayout *fileLayout = new QHBoxLayout;
  fileLayout->addWidget(new QLabel(tr("File: ")));
  fileLayout->addWidget(this->fileLineEdit);
  fileLayout->addWidget(fileButton);

  this->distanceSpin = new QDoubleSpinBox;
  this->distanceSpin->setRange(0.001, 1000);
  this->distanceSpin->setSingleStep(0.1);
  this->distanceSpin->setDecimals(4);
  this->distanceSpin->setValue(1);
  this->distanceSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  this->distanceSpin->setReadOnly(true);
  connect(this->distanceSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnChangeDistance(double)));

  QHBoxLayout *distanceLayout = new QHBoxLayout;
  distanceLayout->addWidget(new QLabel("Distance (m):"));
  distanceLayout->addStretch(1);
  distanceLayout->addWidget(this->distanceSpin);

  this->resolutionSpin = new QDoubleSpinBox;
  this->resolutionSpin->setRange(0, 10000);
  this->resolutionSpin->setSingleStep(10);
  this->resolutionSpin->setDecimals(3);
  this->resolutionSpin->setValue(100);
  this->resolutionSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
  this->resolutionSpin->setReadOnly(true);
  connect(this->resolutionSpin, SIGNAL(valueChanged(double)), this,
      SLOT(OnChangeResolution(double)));

  QHBoxLayout *resolutionLayout = new QHBoxLayout;
  resolutionLayout->addWidget(new QLabel("Resolution (px/m):"));
  resolutionLayout->addStretch(1);
  resolutionLayout->addWidget(this->resolutionSpin);

  QDialogButtonBox *okCancelButtons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  connect(okCancelButtons, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancelButtons, SIGNAL(rejected()), this, SLOT(reject()));

  connect(this, SIGNAL(accepted()), this, SLOT(OnAccept()));

  this->importImageView = new ImportImageView(this);
  QGraphicsScene *scene = new QGraphicsScene();
  scene->setBackgroundBrush(Qt::white);

  this->imageDisplayWidth = 700;
  this->imageDisplayHeight = 500;
  scene->setSceneRect(0, 0, this->imageDisplayWidth,
                            this->imageDisplayHeight);

  this->importImageView->setSizePolicy(QSizePolicy::Expanding,
                                       QSizePolicy::Expanding);
  this->importImageView->setScene(scene);
  this->importImageView->centerOn(QPointF(0, 0));
  this->importImageView->setViewportUpdateMode(
      QGraphicsView::FullViewportUpdate);
  this->importImageView->setDragMode(QGraphicsView::ScrollHandDrag);

  QWidget *leftColumn = new QWidget();
  leftColumn->setSizePolicy(QSizePolicy::Fixed,
                            QSizePolicy::Fixed);
  QVBoxLayout *leftColumnLayout = new QVBoxLayout();
  leftColumn->setLayout(leftColumnLayout);
  leftColumnLayout->addWidget(titleWidget);
  leftColumnLayout->addWidget(new QLabel(tr(
      "<b>Step 1: Select Image</b>")));
  leftColumnLayout->addLayout(fileLayout);
  leftColumnLayout->addWidget(new QLabel(tr(
      "<font size=1 color='grey'>Supported formats: .png, .jpg</font><br>")));
  leftColumnLayout->addWidget(new QLabel(tr(
      "<b>Step 2: Set Scale</b><br>"
      "Draw a line on the image to set<br>"
      "the real world distance between<br>"
      "the two end points.<br><br>")));
  leftColumnLayout->addLayout(distanceLayout);
  leftColumnLayout->addLayout(resolutionLayout);
  leftColumnLayout->addWidget(okCancelButtons);

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(leftColumn);
  mainLayout->addWidget(this->importImageView);


  this->setLayout(mainLayout);

  this->drawingLine = false;
}

/////////////////////////////////////////////////
ImportImageDialog::~ImportImageDialog()
{
}

/////////////////////////////////////////////////
void ImportImageDialog::OnAccept()
{
  std::string filename = this->fileLineEdit->text().toStdString();
  if (!filename.empty())
  {
    this->view->SetBackgroundImage(filename, this->resolutionSpin->value());
  }
}

/////////////////////////////////////////////////
void ImportImageDialog::OnSelectFile()
{
  std::string filename = QFileDialog::getOpenFileName(this,
      tr("Open Image"), "",
      tr("Image Files (*.png *.jpg)")).toStdString();

  if (!filename.empty())
  {
    this->SetFileName(QString::fromStdString(filename));
    this->importImageView->SetImage(filename);
  }
}

/////////////////////////////////////////////////
void ImportImageDialog::OnChangeDistance(double _distance)
{
    double distanceImage = this->importImageView->measureScenePx *
                           this->importImageView->imageWidthPx /
                           this->importImageView->pixmapWidthPx;
    this->resolutionSpin->setValue(distanceImage / _distance);
    this->importImageView->RefreshDistance(_distance);
}

/////////////////////////////////////////////////
void ImportImageDialog::OnChangeResolution(double _resolution)
{
//    double distanceImage = this->importImageView->measureScenePx *
//                           this->importImageView->imageWidthPx /
//                           this->importImageView->pixmapWidthPx;
//    this->distanceSpin->setValue(_resolution / distanceImage);
}
