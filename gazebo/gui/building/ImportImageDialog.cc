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

  this->resolutionSpin = new QDoubleSpinBox;
  this->resolutionSpin->setRange(0.001, 1000);
  this->resolutionSpin->setSingleStep(0.01);
  this->resolutionSpin->setDecimals(4);
  this->resolutionSpin->setValue(0.1);

  QHBoxLayout *resolutionLayout = new QHBoxLayout;
  resolutionLayout->addWidget(new QLabel("Resolution (m/px):"));
  resolutionLayout->addStretch(1);
  resolutionLayout->addWidget(this->resolutionSpin);

  QDialogButtonBox *okCancelButtons = new QDialogButtonBox(
      QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

  connect(okCancelButtons, SIGNAL(accepted()), this, SLOT(accept()));
  connect(okCancelButtons, SIGNAL(rejected()), this, SLOT(reject()));

  connect(this, SIGNAL(accepted()), this, SLOT(OnAccept()));

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(fileLayout);
  mainLayout->addLayout(resolutionLayout);
  mainLayout->addStretch(2);
  mainLayout->addWidget(okCancelButtons);

  this->setLayout(mainLayout);
  this->resize(300, 100);
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
  }
}
