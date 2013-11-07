/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <boost/filesystem.hpp>
#include "gazebo/common/Console.hh"
#include "gazebo/gui/SaveDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
SaveDialog::SaveDialog(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("saveDialog");
  this->fileExtension = "";

  this->messageLabel = new QLabel;
  this->messageLabel->setText(
      tr("Please enter the name and location to save to"));

  QLabel *nameLabel = new QLabel;
  nameLabel->setText(tr("Name"));
  this->nameLineEdit = new QLineEdit;
  this->nameLineEdit->setText(tr("DefaultName"));
  QLabel *saveLocation = new QLabel;
  saveLocation->setText(tr("Location"));
  this->locationLineEdit = new QLineEdit;
  this->locationLineEdit->setText(QDir::homePath());
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
  gridLayout->addWidget(nameLineEdit, 0, 1);
  gridLayout->addWidget(saveLocation, 1, 0);
  gridLayout->addWidget(this->locationLineEdit, 1, 1);
  gridLayout->addWidget(browseButton, 1, 2);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->messageLabel);
  mainLayout->addLayout(gridLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
SaveDialog::~SaveDialog()
{
}

/////////////////////////////////////////////////
std::string SaveDialog::GetSaveName() const
{
  return this->nameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::GetSaveLocation() const
{
  return this->locationLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void SaveDialog::SetSaveName(const std::string &_name)
{
  this->nameLineEdit->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::SetSaveLocation(const std::string &_location)
{
  this->locationLineEdit->setText(tr(_location.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::SetMessage(const std::string &_msg)
{
  this->messageLabel->setText(tr(_msg.c_str()));
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
    this->locationLineEdit->setText(dir);
}

/////////////////////////////////////////////////
void SaveDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void SaveDialog::OnSave()
{
  boost::filesystem::path savePath(this->GetSaveLocation());
  savePath /= this->GetSaveName() + "." + this->fileExtension;

  try
  {
    if (boost::filesystem::exists(savePath.string()))
    {
      std::string msg = "A file named " + savePath.string() +
          " already exists.\nDo you wish to overwrite the existing file?";
      int ret = QMessageBox::warning(0, QString("File Exists"),
          QString(msg.c_str()), QMessageBox::Save | QMessageBox::Cancel,
          QMessageBox::Cancel);

      switch (ret)
      {
        case QMessageBox::Save:
          this->accept();
          break;
        case QMessageBox::Cancel:
          // Do nothing
          break;
        default:
          break;
      }
    }
    else
    {
      this->accept();
    }
  }
  catch(const boost::filesystem::filesystem_error &ex)
  {
    gzerr << ex.what() << std::endl;
  }
}


/////////////////////////////////////////////////
void SaveDialog::SetFileExtension(const std::string &_extension)
{
  this->fileExtension = _extension;
}

/////////////////////////////////////////////////
void SaveDialog::showEvent(QShowEvent */*_event*/)
{
  this->nameLineEdit->selectAll();
}
