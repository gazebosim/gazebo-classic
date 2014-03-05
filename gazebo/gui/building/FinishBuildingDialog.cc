/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/gui/building/FinishBuildingDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
FinishBuildingDialog::FinishBuildingDialog(int _mode, QWidget *_parent)
  : QDialog(_parent)
{
  this->setObjectName("finishBuildingDialog");

  if (_mode == MODEL_FINISH)
    this->setWindowTitle(tr("Finish Model"));
  else if (_mode == MODEL_SAVE)
    this->setWindowTitle(tr("Save Model"));

  QLabel *messageLabel = new QLabel;
  if (_mode == MODEL_FINISH)
  {
    messageLabel->setText(
        tr("Before we finalize your model, please make sure that\n"
        "the following information is correct:\n"));
  }
  else if (_mode == MODEL_SAVE)
  {
    messageLabel->setText(
        tr("Please give your model a name:\n"));
  }

  QLabel *modelLabel = new QLabel;
  modelLabel->setText(tr("Name"));
  this->modelNameLineEdit = new QLineEdit;
  this->modelNameLineEdit->setText(tr("BuildingDefaultName"));
  QLabel *modelLocation = new QLabel;
  modelLocation->setText(tr("Location"));
  this->modelLocationLineEdit = new QLineEdit;
  this->modelLocationLineEdit->setText(QDir::homePath());
  QPushButton *browseButton = new QPushButton(tr("Browse"));
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

/*  QString contributeText(
      tr("Contribute this model to the Model Database so that\n"
         "the entire Gazebo community can benefit!\n"
         "[This will open up a new tab in your browser]\n"));
  QCheckBox *contributeCheckBox = new QCheckBox(contributeText);*/

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  std::string finishButtonText = "&Done";
  if (_mode == MODEL_SAVE)
      finishButtonText = "&Save";

  QPushButton *finishButton = new QPushButton(tr(finishButtonText.c_str()));
  finishButton->setDefault(true);
  connect(finishButton, SIGNAL(clicked()), this, SLOT(OnFinish()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(finishButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(modelLabel, 0, 0);
  gridLayout->addWidget(modelNameLineEdit, 0, 1);
  gridLayout->addWidget(modelLocation, 1, 0);
  gridLayout->addWidget(this->modelLocationLineEdit, 1, 1);
  gridLayout->addWidget(browseButton, 1, 2);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(messageLabel);
  mainLayout->addLayout(gridLayout);
//  if (_mode == MODEL_FINISH)
//    mainLayout->addWidget(contributeCheckBox);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
FinishBuildingDialog::~FinishBuildingDialog()
{
}

/////////////////////////////////////////////////
std::string FinishBuildingDialog::GetModelName() const
{
  return this->modelNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string FinishBuildingDialog::GetSaveLocation() const
{
  return this->modelLocationLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void FinishBuildingDialog::SetModelName(const std::string &_name)
{
  this->modelNameLineEdit->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void FinishBuildingDialog::SetSaveLocation(const std::string &_location)
{
  this->modelLocationLineEdit->setText(tr(_location.c_str()));
}

/////////////////////////////////////////////////
void FinishBuildingDialog::OnBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
    QDir::homePath(), QFileDialog::ShowDirsOnly
    | QFileDialog::DontResolveSymlinks);
  if (!dir.isEmpty())
    this->modelLocationLineEdit->setText(dir);
}

/////////////////////////////////////////////////
void FinishBuildingDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void FinishBuildingDialog::OnFinish()
{
  this->accept();
}
