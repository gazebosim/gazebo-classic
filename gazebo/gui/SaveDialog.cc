/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include "gazebo/gui/SaveDialog.hh"

using namespace gazebo;
using namespace gui;

/// \brief Private data class
class gazebo::gui::SaveDialogPrivate
{
  /// \brief Editable line that holds the name.
  public: QLineEdit *nameLineEdit;

  /// \brief Editable line that holds the save location.
  public: QLineEdit *locationLineEdit;

  /// \brief Message displayed in the dialog.
  public: QLabel *messageLabel;
};

/////////////////////////////////////////////////
SaveDialog::SaveDialog(QWidget *_parent)
: QDialog(_parent), dataPtr(new SaveDialogPrivate)
{
  this->setObjectName("saveDialog");

  this->dataPtr->messageLabel = new QLabel;
  this->dataPtr->messageLabel->setText(
      tr("Please enter the name and location to save to"));

  QLabel *nameLabel = new QLabel;
  nameLabel->setText(tr("Name"));
  this->dataPtr->nameLineEdit = new QLineEdit;
  this->dataPtr->nameLineEdit->setText(tr("DefaultName"));
  QLabel *saveLocation = new QLabel;
  saveLocation->setText(tr("Location"));
  this->dataPtr->locationLineEdit = new QLineEdit;
  this->dataPtr->locationLineEdit->setText(QDir::homePath());
  QPushButton *browseButton = new QPushButton(tr("Browse"));
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *saveButton = new QPushButton("&Save");
  saveButton->setDefault(true);
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(saveButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(nameLabel, 0, 0);
  gridLayout->addWidget(this->dataPtr->nameLineEdit, 0, 1);
  gridLayout->addWidget(saveLocation, 1, 0);
  gridLayout->addWidget(this->dataPtr->locationLineEdit, 1, 1);
  gridLayout->addWidget(browseButton, 1, 2);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->dataPtr->messageLabel);
  mainLayout->addLayout(gridLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
SaveDialog::~SaveDialog()
{
}

/////////////////////////////////////////////////
std::string SaveDialog::SaveName() const
{
  return this->dataPtr->nameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::SaveLocation() const
{
  return this->dataPtr->locationLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void SaveDialog::SetSaveName(const std::string &_name)
{
  this->dataPtr->nameLineEdit->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::SetSaveLocation(const std::string &_location)
{
  this->dataPtr->locationLineEdit->setText(tr(_location.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::SetMessage(const std::string &_msg)
{
  this->dataPtr->messageLabel->setText(tr(_msg.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::SetTitle(const std::string &_title)
{
  this->setWindowTitle(tr(_title.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::OnBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
    QDir::homePath(), QFileDialog::ShowDirsOnly
    | QFileDialog::DontResolveSymlinks);
  if (!dir.isEmpty())
    this->dataPtr->locationLineEdit->setText(dir);
}

/////////////////////////////////////////////////
void SaveDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void SaveDialog::OnSave()
{
  this->accept();
}

/////////////////////////////////////////////////
void SaveDialog::showEvent(QShowEvent */*_event*/)
{
  this->dataPtr->nameLineEdit->selectAll();
}

/////////////////////////////////////////////////
std::string SaveDialog::Message() const
{
  return this->dataPtr->messageLabel->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::Title() const
{
  return this->windowTitle().toStdString();
}
