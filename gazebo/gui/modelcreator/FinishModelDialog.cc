/*
 * Copyright 2011 Nate Koenig
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

#include "gui/modelcreator/FinishModelDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
FinishModelDialog::FinishModelDialog(QWidget *_parent)
  : QDialog(_parent)
{
  this->setObjectName("finishModelDialog");
  this->setWindowTitle(tr("Finish Model"));

  QLabel *messageLabel = new QLabel;
  messageLabel->setText(
      tr("Before we finalize your model, please make sure that\n"
      "the following information is correct:\n"));

  QLabel *modelLabel = new QLabel;
  modelLabel->setText(tr("Name"));
  QLineEdit* modelNameLineEdit = new QLineEdit;
  modelNameLineEdit->setPlaceholderText(tr("MyNamedModel.sdf"));
  QLabel *modelLocation = new QLabel;
  modelLocation->setText(tr("Location"));
  modelLocationLineEdit = new QLineEdit;
  this->modelLocationLineEdit->setPlaceholderText(tr(""));
  QPushButton *browseButton = new QPushButton(tr("Browse"));
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  QString contributeText(
      tr("Contribute this model to the Model Database so that\n"
         "the entire Gazebo community can benefit!\n"
         "[This will open up a new tab in your browser]\n"));
  QCheckBox *contributeCheckBox = new QCheckBox(contributeText);

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *finishButton = new QPushButton(tr("&Finish"));
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
  mainLayout->addWidget(contributeCheckBox);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
FinishModelDialog::~FinishModelDialog()
{
}

/////////////////////////////////////////////////
void FinishModelDialog::OnBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
    "/home", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
  this->modelLocationLineEdit->setText(dir);
}

/////////////////////////////////////////////////
void FinishModelDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void FinishModelDialog::OnFinish()
{
  /// TODO:
  this->accept();
}
