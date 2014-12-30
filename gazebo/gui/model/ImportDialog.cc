/*
 * Copyright (C) 2013-2014 Open Source Robotics Foundation
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

#include "gazebo/gui/model/ImportDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImportDialog::ImportDialog(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("ImportDialog");
  this->setWindowTitle("Custom Part");

  this->messageLabel = new QLabel;
  this->messageLabel->setText(
      tr("You can import a 3D mesh that you have \n"
      "made with a modelling tool such as Blender \n"
      "Maya, or SolidWorks. It will apear as a \n"
      "part in the 3D View."));

  this->pathLineEdit = new QLineEdit;
  this->pathLineEdit->setText(QDir::homePath());
  QPushButton *browseButton = new QPushButton(tr("Browse"));
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  QLabel *nameLabel = new QLabel;
  nameLabel->setText(tr("Part Name:"));
  this->nameLineEdit = new QLineEdit;
  this->nameLineEdit->setText(tr("DefaultName"));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  QPushButton *importButton = new QPushButton("&Import");
  importButton->setDefault(true);
  connect(importButton, SIGNAL(clicked()), this, SLOT(OnImport()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(importButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QGridLayout *gridLayout = new QGridLayout;
  gridLayout->addWidget(this->pathLineEdit, 0, 0);
  gridLayout->addWidget(browseButton, 0, 1);
  gridLayout->addWidget(nameLabel, 1, 0);
  gridLayout->addWidget(nameLineEdit, 1, 1);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->messageLabel);
  mainLayout->addLayout(gridLayout);
  mainLayout->addLayout(buttonsLayout);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
ImportDialog::~ImportDialog()
{
}

/////////////////////////////////////////////////
std::string ImportDialog::GetPartName() const
{
  return this->nameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string ImportDialog::GetImportPath() const
{
  return this->pathLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void ImportDialog::SetPartName(const std::string &_name)
{
  this->nameLineEdit->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void ImportDialog::SetImportPath(const std::string &_path)
{
  this->pathLineEdit->setText(tr(_path.c_str()));
}

/////////////////////////////////////////////////
void ImportDialog::SetTitle(const std::string &_title)
{
  this->setWindowTitle(tr(_title.c_str()));
}

/////////////////////////////////////////////////
void ImportDialog::OnBrowse()
{
  QFileDialog fd(this, tr("Import Part"), QDir::homePath(),
      tr("Mesh files (*.dae *.stl)"));
  fd.setFilter(QDir::AllDirs | QDir::Hidden);
  fd.setFileMode(QFileDialog::ExistingFile);
  if (fd.exec())
  {
    if (!fd.selectedFiles().isEmpty())
    {
      QString file = fd.selectedFiles().at(0);
      if (!file.isEmpty())
      {
        this->pathLineEdit->setText(file);
      }
    }
  }
}

/////////////////////////////////////////////////
void ImportDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void ImportDialog::OnImport()
{
  QFileInfo info(this->pathLineEdit->text());
  if (info.isFile())
  {
    this->accept();
  }
  else
  {
    std::string msg = this->pathLineEdit->text().toStdString() +
        " is not a valid mesh file.\nPlease select another file.";
    QMessageBox::warning(this, QString("Invalid Mesh File"),
        QString(msg.c_str()), QMessageBox::Ok,
        QMessageBox::Ok);
  }
}

/////////////////////////////////////////////////
void ImportDialog::showEvent(QShowEvent */*_event*/)
{
  this->nameLineEdit->selectAll();
}
