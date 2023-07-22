/*
 * Copyright (C) 2013 Open Source Robotics Foundation
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
#include <tinyxml.h>

#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonIface.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/SaveEntityDialog.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace gui;

class gazebo::gui::SaveEntityDialogPrivate
{
  /// \brief Widget container to hold advanced model saving options.
  public: QWidget *advancedOptionsWidget;

  /// \brief Label appearing at the top of the dialog box.
  public: QLabel *messageLabel;

  /// \brief Editable line that holds the model name.
  public: QLineEdit* modelNameLineEdit;

  /// \brief Editable line that holds the model's version.
  public: QLineEdit* modelVersionLineEdit;

  /// \brief Editable line that holds the model's description.
  public: QLineEdit* modelDescriptionLineEdit;

  /// \brief Editable line that holds the model's author's name.
  public: QLineEdit* modelAuthorNameLineEdit;

  /// \brief Editable line that holds the model's author's email.
  public: QLineEdit* modelAuthorEmailLineEdit;

  /// \brief Editable line that holds the model's save location.
  public: QLineEdit* modelLocationLineEdit;

  /// \brief The model's config file.
  public: TiXmlDocument modelConfig;
};

/////////////////////////////////////////////////
SaveEntityDialog::SaveEntityDialog(int _mode, QWidget *_parent)
  : QDialog(_parent), dataPtr(new SaveEntityDialogPrivate)
{
  this->setObjectName("saveDialog");
  this->setWindowTitle(tr("Save Model"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  this->dataPtr->messageLabel = new QLabel;
  this->dataPtr->messageLabel->setText(
      tr("Pick a name and a location for your model.\n"
         "All the model files will be saved in the model folder.\n"));

  QLabel *modelLabel = new QLabel;
  modelLabel->setText(tr("Model Name:"));
  this->dataPtr->modelNameLineEdit = new QLineEdit;
  connect(this->dataPtr->modelNameLineEdit, SIGNAL(textChanged(QString)), this,
           SLOT(ModelNameChangedOnDialog(QString)));

  QLabel *modelHeader = new QLabel;
  modelHeader->setText(tr("<b>Model</b>"));

  QLabel *modelLocation = new QLabel;
  modelLocation->setText(tr("Location:"));
  this->dataPtr->modelLocationLineEdit = new QLineEdit;
  this->dataPtr->modelLocationLineEdit->setMinimumWidth(300);

  // Try to get path to home folder
  if (_mode == SaveMode::BUILDING)
  {
    this->dataPtr->modelLocationLineEdit->setText(QDir::homePath()+
        "/building_editor_models/Untitled");
  }
  else if (_mode == SaveMode::MODEL)
  {
    this->dataPtr->modelLocationLineEdit->setText(QDir::homePath()+
        "/model_editor_models/Untitled");
  }

  QPushButton *browseButton = new QPushButton(tr("Browse"));
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  QLabel *authorHeader = new QLabel;
  authorHeader->setText(tr("<b>Author</b>"));
  QLabel *modelAuthorName = new QLabel;
  modelAuthorName->setText(tr("  Name:"));
  this->dataPtr->modelAuthorNameLineEdit = new QLineEdit;
  QLabel *modelAuthorEmail = new QLabel;
  modelAuthorEmail->setText(tr("  Email:"));
  this->dataPtr->modelAuthorEmailLineEdit = new QLineEdit;

  QLabel *modelVersion = new QLabel;
  modelVersion->setText(tr("  Version:"));
  this->dataPtr->modelVersionLineEdit = new QLineEdit;
  this->dataPtr->modelVersionLineEdit->setText(tr("1.0"));

  QLabel *modelDescription = new QLabel;
  modelDescription->setText(tr("  Description:"));
  this->dataPtr->modelDescriptionLineEdit = new QLineEdit;

// TODO Propshop integration
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
  connect(saveButton, SIGNAL(clicked()), this, SLOT(OnAcceptSave()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(saveButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QHBoxLayout *modelNameLayout = new QHBoxLayout;
  modelNameLayout->addWidget(modelLabel);
  modelNameLayout->addWidget(this->dataPtr->modelNameLineEdit);

  QHBoxLayout *locationLayout = new QHBoxLayout;
  locationLayout->addWidget(modelLocation);
  locationLayout->addWidget(this->dataPtr->modelLocationLineEdit);
  locationLayout->addWidget(browseButton);

  QRadioButton *advancedOptionsCollapser = new QRadioButton();
  advancedOptionsCollapser->setFocusPolicy(Qt::NoFocus);
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

  advancedOptionsGrid->addWidget(modelHeader, 1, 0);
  advancedOptionsGrid->addWidget(modelVersion, 2, 0);
  advancedOptionsGrid->addWidget(this->dataPtr->modelVersionLineEdit, 2, 1);
  advancedOptionsGrid->addWidget(modelDescription, 3, 0);
  advancedOptionsGrid->addWidget(this->dataPtr->modelDescriptionLineEdit, 3, 1);

  advancedOptionsGrid->addWidget(authorHeader, 4, 0);
  advancedOptionsGrid->addWidget(modelAuthorName, 5, 0);
  advancedOptionsGrid->addWidget(this->dataPtr->modelAuthorNameLineEdit, 5, 1);
  advancedOptionsGrid->addWidget(modelAuthorEmail, 6, 0);
  advancedOptionsGrid->addWidget(this->dataPtr->modelAuthorEmailLineEdit, 6, 1);

  this->dataPtr->advancedOptionsWidget = new QWidget();
  this->dataPtr->advancedOptionsWidget->setLayout(advancedOptionsGrid);
  this->dataPtr->advancedOptionsWidget->hide();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->dataPtr->messageLabel);
  mainLayout->addLayout(modelNameLayout);
  mainLayout->addLayout(locationLayout);
  mainLayout->addLayout(advancedOptions);
  mainLayout->addWidget(this->dataPtr->advancedOptionsWidget);
  mainLayout->addLayout(buttonsLayout);
  mainLayout->setSizeConstraint(QLayout::SetFixedSize);

  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
SaveEntityDialog::~SaveEntityDialog()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetModelName() const
{
  return this->dataPtr->modelNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetSaveLocation() const
{
  return this->dataPtr->modelLocationLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetAuthorName() const
{
  return this->dataPtr->modelAuthorNameLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetAuthorEmail() const
{
  return this->dataPtr->modelAuthorEmailLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetDescription() const
{
  return this->dataPtr->modelDescriptionLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetVersion() const
{
  return this->dataPtr->modelVersionLineEdit->text().toStdString();
}

/////////////////////////////////////////////////
void SaveEntityDialog::SetModelName(const std::string &_name)
{
  this->dataPtr->modelNameLineEdit->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
void SaveEntityDialog::SetSaveLocation(const std::string &_location)
{
  this->dataPtr->modelLocationLineEdit->setText(tr(_location.c_str()));
}

/////////////////////////////////////////////////
void SaveEntityDialog::OnBrowse()
{
  QFileDialog fileDialog(this, tr("Open Directory"), QDir::homePath());
  fileDialog.setFileMode(QFileDialog::Directory);
  fileDialog.setOptions(QFileDialog::ShowDirsOnly
      | QFileDialog::DontResolveSymlinks | QFileDialog::DontUseNativeDialog);
  fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  if (fileDialog.exec() == QDialog::Accepted)
  {
    QStringList selected = fileDialog.selectedFiles();
    if (selected.empty())
      return;

    // Substitute everything up to the model folder
    std::string folder = this->GetSaveLocation();
    folder = folder.substr(folder.rfind("/")+1);

    this->SetSaveLocation(selected[0].toStdString() + "/" + folder);
  }
}

/////////////////////////////////////////////////
void SaveEntityDialog::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void SaveEntityDialog::OnAcceptSave()
{
  this->accept();
}

/////////////////////////////////////////////////
bool SaveEntityDialog::OnSaveAs()
{
  if (this->exec() == QDialog::Accepted)
  {
    if (this->GetModelName().size() == 0)
    {
      QMessageBox msgBox(QMessageBox::Warning, QString("Empty Name"),
                       QString("Please give your model a non-empty name."));

      msgBox.exec();
      return this->OnSaveAs();
    }
    if (this->GetSaveLocation().size() == 0)
    {
      QMessageBox msgBox(QMessageBox::Warning, QString("Empty Location"),
             QString("Please give a path to where your model will be saved."));

      msgBox.exec();
      return this->OnSaveAs();
    }

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
      std::string msg = "A model already exists in folder \n" +
                        path.string() + ".\n\n" +
                        "Do you wish to overwrite the existing model files?\n";

      QMessageBox msgBox(QMessageBox::Warning, QString("Files Exist"),
                         QString(msg.c_str()));

      QPushButton *cancelButton =
          msgBox.addButton("Cancel", QMessageBox::RejectRole);
      QPushButton *saveButton = msgBox.addButton("Save",
          QMessageBox::AcceptRole);
      msgBox.setDefaultButton(saveButton);
      msgBox.setEscapeButton(cancelButton);
      msgBox.exec();
      if (msgBox.clickedButton() != saveButton)
      {
        return this->OnSaveAs();
      }
    }

    this->AddDirToModelPaths(this->GetSaveLocation());
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void SaveEntityDialog::ToggleAdvancedOptions(bool _checked)
{
  if (_checked)
  {
    this->dataPtr->advancedOptionsWidget->show();
  }
  else
  {
    this->dataPtr->advancedOptionsWidget->hide();
  }
}

/////////////////////////////////////////////////
void SaveEntityDialog::AddDirToModelPaths(const std::string &_path)
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
    // Append it to gui.ini
    if (additionalProperties.empty())
      additionalProperties = parentDirectory;
    else
      additionalProperties = additionalProperties + ":" + parentDirectory;

    gui::setINIProperty("model_paths.filenames", additionalProperties);

    // Save any changes that were made to the property tree
    // TODO: check gui.ini env variable
    char *home = getenv(HOMEDIR);
    if (home)
    {
      boost::filesystem::path guiINIPath = home;
      guiINIPath  = guiINIPath / ".gazebo" / "gui.ini";
      saveINI(guiINIPath);
    }
  }
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetTemplateConfigString()
{
  std::ostringstream newModelStr;
  newModelStr << "<?xml version=\"1.0\"?>"
  << "<model>"
  <<   "<name>template_model</name>"
  <<   "<version>1.0</version>"
  <<   "<sdf version=\"" << SDF_VERSION << "\">model.sdf</sdf>"
  <<   "<author>"
  <<     "<name>author_name</name>"
  <<     "<email>author_email</email>"
  <<   "</author>"
  <<   "<description>Made with Gazebo</description>"
  << "</model>";
  return newModelStr.str();
}

/////////////////////////////////////////////////
void SaveEntityDialog::GenerateConfig()
{
  // Create an xml config file
  this->dataPtr->modelConfig.Clear();
  this->dataPtr->modelConfig.Parse(this->GetTemplateConfigString().c_str());

  TiXmlElement *modelXML = this->dataPtr
      ->modelConfig.FirstChildElement("model");
  if (!modelXML)
  {
    gzerr << "No model name in default config file" << std::endl;
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
void SaveEntityDialog::SaveToConfig()
{
  boost::filesystem::path path(this->GetSaveLocation());
  path = path / "model.config";
  const char* modelConfigString = path.string().c_str();

  this->dataPtr->modelConfig.SaveFile(modelConfigString);
  gzdbg << "Saved file to " << modelConfigString << std::endl;
}

/////////////////////////////////////////////////
void SaveEntityDialog::SaveToSDF(sdf::SDFPtr _modelSDF)
{
  std::ofstream savefile;
  boost::filesystem::path path(this->GetSaveLocation());
  path = path / "model.sdf";

  // FIXME
  savefile.open(path.string().c_str());
  if (!savefile.is_open())
  {
    gzerr << "Couldn't open file for writing: " << path.string() << std::endl;
    return;
  }
  savefile << _modelSDF->ToString();
  savefile.close();
  gzdbg << "Saved file to " << path.string() << std::endl;
  this->AddDirToModelPaths(this->GetSaveLocation());
}

/////////////////////////////////////////////////
std::string SaveEntityDialog::GetFolderNameFromModelName(const std::string
    &_modelName)
{
  // Auto-generate folder name based on model name
  std::string foldername = _modelName;

  std::vector<std::pair<std::string, std::string> > replacePairs;
  replacePairs.push_back(std::pair<std::string, std::string>(" ", "_"));

  for (unsigned int i = 0; i < replacePairs.size(); ++i)
  {
    std::string forbiddenChar = replacePairs[i].first;
    std::string replaceChar = replacePairs[i].second;
    size_t index = foldername.find(forbiddenChar);
    while (index != std::string::npos)
    {
      foldername.replace(index, forbiddenChar.size(), replaceChar);
      index = foldername.find(forbiddenChar);
    }
  }

  return foldername;
}

/////////////////////////////////////////////////
void SaveEntityDialog::ModelNameChangedOnDialog(QString _modelName)
{
  std::string folderName = this->GetFolderNameFromModelName(
      _modelName.toStdString());

  // Use current path and change only last folder name
  std::string path = this->GetSaveLocation();
  path = path.substr(0, path.rfind("/")+1);
  path += folderName;

  this->SetSaveLocation(path);
}
