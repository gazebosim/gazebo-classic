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

#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/SaveDialog.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
SaveDialog::SaveDialog(int _mode, QWidget *_parent)
  : QDialog(_parent)
{
  this->setObjectName("SaveDialog");
  this->setWindowTitle(tr("Save Model"));

  this->messageLabel = new QLabel;
  this->messageLabel->setText(
      tr("Pick a location for your model \"Untitled\":\n"));

  QLabel *modelLabel = new QLabel;
  modelLabel->setText(tr("Model Name: "));
  this->modelNameLineEdit = new QLineEdit;

  QLabel *modelHeader = new QLabel;
  modelHeader->setText(tr("<b>Model</b>"));

  QLabel *modelLocation = new QLabel;
  modelLocation->setText(tr("  Location:"));
  this->modelLocationLineEdit = new QLineEdit;
  // Try to get path to home folder
  if (_mode == BUILDING)
  {
    this->modelLocationLineEdit->setText(QDir::homePath()+
        "/building_editor_models/Untitled");
  }
  else if (_mode == MODEL)
  {
    this->modelLocationLineEdit->setText(QDir::homePath()+
        "/model_editor_models/Untitled");
  }

  QPushButton *browseButton = new QPushButton(tr("Browse"));
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  QLabel *authorHeader = new QLabel;
  authorHeader->setText(tr("<b>Author</b>"));
  QLabel *modelAuthorName = new QLabel;
  modelAuthorName->setText(tr("  Name:"));
  this->modelAuthorNameLineEdit = new QLineEdit;
  QLabel *modelAuthorEmail = new QLabel;
  modelAuthorEmail->setText(tr("  Email:"));
  this->modelAuthorEmailLineEdit = new QLineEdit;

  QLabel *modelVersion = new QLabel;
  modelVersion->setText(tr("  Version:"));
  this->modelVersionLineEdit = new QLineEdit;
  this->modelVersionLineEdit->setText(tr("1.0"));

  QLabel *modelDescription = new QLabel;
  modelDescription->setText(tr("  Description:"));
  this->modelDescriptionLineEdit = new QLineEdit;

/*  QString contributeText(
      tr("Contribute this model to the Model Database so that\n"
         "the entire Gazebo community can benefit!\n"
         "[This will open up a new tab in your browser]\n"));
  QCheckBox *contributeCheckBox = new QCheckBox(contributeText);*/

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));

  std::string saveButtonText = "&Save";

  QPushButton *saveButton = new QPushButton(tr(saveButtonText.c_str()));
  saveButton->setDefault(true);
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnSave()));
  buttonsLayout->addWidget(saveButton);
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QHBoxLayout *locationLayout = new QHBoxLayout;

  locationLayout->addWidget(modelLocation);
  locationLayout->addWidget(this->modelLocationLineEdit);
  locationLayout->addWidget(browseButton);

  QRadioButton *advancedOptionsCollapser = new QRadioButton();
  advancedOptionsCollapser->setChecked(false);
  advancedOptionsCollapser->setText("Advanced Options");
  advancedOptionsCollapser->setStyleSheet(
     "QRadioButton {\
        color: #d0d0d0;\
      }\
      QRadioButton::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QRadioButton::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  // initialize as "closed" (unchecked)
  // Button behavior: when "open", show advancedOptionsGrid
  connect(advancedOptionsCollapser, SIGNAL(toggled(bool)), this,
           SLOT(ToggleAdvancedOptions(bool)));

  QHBoxLayout *advancedOptions = new QHBoxLayout();
  advancedOptions->addWidget(advancedOptionsCollapser);

  // Advanced options
  QGridLayout *advancedOptionsGrid = new QGridLayout();
  advancedOptionsGrid->addWidget(modelLabel, 0, 0);
  advancedOptionsGrid->addWidget(modelNameLineEdit, 0, 1);

  advancedOptionsGrid->addWidget(modelHeader, 1, 0);
  advancedOptionsGrid->addWidget(modelVersion, 2, 0);
  advancedOptionsGrid->addWidget(this->modelVersionLineEdit, 2, 1);
  advancedOptionsGrid->addWidget(modelDescription, 3, 0);
  advancedOptionsGrid->addWidget(this->modelDescriptionLineEdit, 3, 1);

  advancedOptionsGrid->addWidget(authorHeader, 4, 0);
  advancedOptionsGrid->addWidget(modelAuthorName, 5, 0);
  advancedOptionsGrid->addWidget(this->modelAuthorNameLineEdit, 5, 1);
  advancedOptionsGrid->addWidget(modelAuthorEmail, 6, 0);
  advancedOptionsGrid->addWidget(this->modelAuthorEmailLineEdit, 6, 1);

  this->advancedOptionsWidget = new QWidget();
  this->advancedOptionsWidget->setLayout(advancedOptionsGrid);
  this->advancedOptionsWidget->hide();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->messageLabel);
  mainLayout->addLayout(locationLayout);

  mainLayout->addLayout(advancedOptions);
  mainLayout->addWidget(this->advancedOptionsWidget);
  mainLayout->addLayout(buttonsLayout);
  mainLayout->setAlignment(Qt::AlignTop);

  this->setLayout(mainLayout);
  this->setMinimumSize(400, 150);
  this->setMaximumSize(400, 380);
  this->resize(this->minimumSize());
}

/////////////////////////////////////////////////
SaveDialog::~SaveDialog()
{
}

/////////////////////////////////////////////////
std::string SaveDialog::GetModelName() const
{
  return this->modelNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::GetSaveLocation() const
{
  return this->modelLocationLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::GetAuthorName() const
{
  return this->modelAuthorNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::GetAuthorEmail() const
{
  return this->modelAuthorEmailLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::GetDescription() const
{
  return this->modelDescriptionLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveDialog::GetVersion() const
{
  return this->modelVersionLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void SaveDialog::SetModelName(const std::string &_name)
{
  this->modelNameLineEdit->setText(tr(_name.c_str()));
  std::string label = "Pick a location for your model \""
                      + _name + "\":\n";
  this->messageLabel->setText(QString(label.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::SetSaveLocation(const std::string &_location)
{
  this->modelLocationLineEdit->setText(tr(_location.c_str()));
}

/////////////////////////////////////////////////
void SaveDialog::OnBrowse()
{
  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
    QDir::homePath(), QFileDialog::ShowDirsOnly
    | QFileDialog::DontResolveSymlinks);
  if (!dir.isEmpty())
    this->modelLocationLineEdit->setText(dir);
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
bool SaveDialog::OnSaveAs(const std::string &_saveName)
{
  this->SetModelName(_saveName);

  if (this->exec() == QDialog::Accepted)
  {
    if (this->GetModelName().size() == 0)
    {
      QMessageBox msgBox(QMessageBox::Warning, QString("Empty Name"),
                       QString("Please give your model a non-empty name."));

      msgBox.exec();
      return this->OnSaveAs(_saveName);
    }
    if (this->GetSaveLocation().size() == 0)
    {
      QMessageBox msgBox(QMessageBox::Warning, QString("Empty Location"),
             QString("Please give a path to where your model will be saved."));

      msgBox.exec();
      return this->OnSaveAs(_saveName);
    }

    // Parse saveLocation and set model name
    boost::filesystem::path saveLocPath(this->GetSaveLocation());
    this->SetModelName(saveLocPath.filename().string());

    boost::filesystem::path path;
    path = path / this->GetSaveLocation();
    if (!boost::filesystem::exists(path))
    {
      if (!boost::filesystem::create_directories(path))
      {
        gzerr << "Couldn't create folder for model files." << std::endl;
        return false;
      }
      gzmsg << "Created folder " << path << " for model files." << std::endl;
    }

    boost::filesystem::path modelConfigPath = path / "model.config";

    boost::filesystem::path sdfPath = path / "model.sdf";

    // Before writing
    if (boost::filesystem::exists(sdfPath) ||
          boost::filesystem::exists(modelConfigPath))
    {
      std::string msg = "A model named " + this->GetModelName() +
                        " already exists in folder " + path.string() + ".\n\n"
                        "Do you wish to overwrite the existing model files?\n";

      QMessageBox msgBox(QMessageBox::Warning, QString("Files Exist"),
                         QString(msg.c_str()));

      QPushButton *saveButton = msgBox.addButton("Save",
                                                 QMessageBox::ApplyRole);
      msgBox.addButton(QMessageBox::Cancel);
      msgBox.exec();
      if (msgBox.clickedButton() != saveButton)
      {
        return this->OnSaveAs(this->GetModelName());
      }
    }

    this->AddDirToModelPaths(this->GetSaveLocation());

    this->GenerateConfig();
    this->SaveToConfig(this->GetSaveLocation());

    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void SaveDialog::ToggleAdvancedOptions(bool _checked)
{
  if (_checked)
  {
    this->advancedOptionsWidget->show();
    this->resize(this->maximumSize());
  }
  else
  {
    this->advancedOptionsWidget->hide();
    this->resize(this->minimumSize());
  }
}

/////////////////////////////////////////////////
// Add the parent folder of _path to the model path represented by SystemPaths,
// notify InsertModelWidget to display the model name in the "Insert Models"
// tab, and write the parent folder filename to gui.ini
void SaveDialog::AddDirToModelPaths(const std::string& _path)
{
  std::string parentDirectory = boost::filesystem::path(_path)
                                  .parent_path().string();

  std::list<std::string> modelPaths =
              gazebo::common::SystemPaths::Instance()->GetModelPaths();
  std::list<std::string>::iterator iter;
  for (iter = modelPaths.begin();
       iter != modelPaths.end(); ++iter)
  {
    if (iter->compare(parentDirectory) == 0)
    {
      break;
    }
  }

  gazebo::common::SystemPaths::Instance()->
    AddModelPathsUpdate(parentDirectory);

  std::string additionalProperties =
    gui::getINIProperty<std::string>("model_paths.filenames", "");
  if (additionalProperties.find(parentDirectory) == std::string::npos)
  {
    // Add it to gui.ini
    gui::setINIProperty("model_paths.filenames", parentDirectory);

    // Save any changes that were made to the property tree
    // TODO: check gui.ini env variable
    char *home = getenv("HOME");
    if (home)
    {
      boost::filesystem::path guiINIPath = home;
      guiINIPath  = guiINIPath / ".gazebo" / "gui.ini";
      saveINI(guiINIPath);
    }
  }
}

/////////////////////////////////////////////////
std::string SaveDialog::GetTemplateConfigString()
{
  std::ostringstream newModelStr;
  newModelStr << "<?xml version=\"1.0\"?>"
  << "<model>"
  <<   "<name>building_template_model</name>"
  <<   "<version>1.0</version>"
  <<   "<sdf version=\"1.5\">model.sdf</sdf>"
  <<   "<author>"
  <<     "<name>author_name</name>"
  <<     "<email>author_email</email>"
  <<   "</author>"
  <<   "<description>Made with the Gazebo Building Editor</description>"
  << "</model>";
  return newModelStr.str();
}

/////////////////////////////////////////////////
void SaveDialog::GenerateConfig()
{
  // Create an xml config file
  this->modelConfig.Clear();
  this->modelConfig.Parse(this->GetTemplateConfigString().c_str());

  TiXmlElement *modelXML = this->modelConfig.FirstChildElement("model");
  if (!modelXML)
  {
    gzerr << "No model name in default config file\n";
    return;
  }
  TiXmlElement *modelNameXML = modelXML->FirstChildElement("name");
  modelNameXML->FirstChild()->SetValue(this->GetModelName());

  TiXmlElement *versionXML = modelXML->FirstChildElement("version");
  if (!versionXML)
  {
    gzerr << "Couldn't find model version" << std::endl;
    versionXML->FirstChild()->SetValue("1.0");
  }
  else
  {
    versionXML->FirstChild()->SetValue(this->GetVersion());
  }

  TiXmlElement *descriptionXML = modelXML->FirstChildElement("description");
  if (!descriptionXML)
  {
    gzerr << "Couldn't find model description" << std::endl;
    descriptionXML->FirstChild()->SetValue("");
  }
  else
  {
    descriptionXML->FirstChild()->SetValue(this->GetDescription());
  }

  // TODO: Multiple authors
  TiXmlElement *authorXML = modelXML->FirstChildElement("author");
  if (!authorXML)
  {
    gzerr << "Couldn't find model author" << std::endl;
  }
  else
  {
    TiXmlElement *authorChild = authorXML->FirstChildElement("name");
    if (!authorChild)
    {
      gzerr << "Couldn't find author name" << std::endl;
      authorChild->FirstChild()->SetValue("");
    }
    else
    {
      authorChild->FirstChild()->SetValue(this->GetAuthorName());
    }
    authorChild = authorXML->FirstChildElement("email");
    if (!authorChild)
    {
      gzerr << "Couldn't find author email" << std::endl;
      authorChild->FirstChild()->SetValue("");
    }
    else
    {
      authorChild->FirstChild()->SetValue(this->GetAuthorEmail());
    }
  }
}

/////////////////////////////////////////////////
void SaveDialog::SaveToConfig(const std::string &_savePath)
{
  boost::filesystem::path path(_savePath);
  path = path / "model.config";
  const char* modelConfigString = path.string().c_str();

  this->modelConfig.SaveFile(modelConfigString);
  gzdbg << "Saved file to " << modelConfigString << std::endl;
}
