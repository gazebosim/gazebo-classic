/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
/*
#include "gazebo/common/Console.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/gui/ConfigWidget.hh"

#include "gazebo/gui/model/LinkConfig.hh"
#include "gazebo/gui/model/VisualConfig.hh"
#include "gazebo/gui/model/CollisionConfig.hh"
*/
#include "gazebo/gui/model/ModelPluginInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelPluginInspector::ModelPluginInspector(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("ModelPluginInspector");
  this->setWindowTitle(tr("Model Plugin Inspector"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // Plugin name
  QLabel *nameKeyLabel = new QLabel(tr("Name:"));
  this->nameValueLabel = new QLabel(tr(""));
  this->nameValueLabel->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);

  // Plugin filename
  QLabel *filenameKeyLabel = new QLabel(tr("Filename:"));
  this->filenameValueLabel = new QLabel(tr(""));

  // Plugin parameters
  this->paramsText = new QTextEdit();
  this->paramsText->setReadOnly(true);
  this->paramsText->setMinimumWidth(600);
  this->paramsText->setStyleSheet("\
      QTextEdit\
      {\
        background-color: rgba(200, 200, 200, 255);\
      }");

  // Contents layout
  QGridLayout *contentsLayout = new QGridLayout;
  contentsLayout->addWidget(nameKeyLabel, 0, 0);
  contentsLayout->addWidget(this->nameValueLabel, 0, 1);
  contentsLayout->addWidget(filenameKeyLabel, 1, 0);
  contentsLayout->addWidget(this->filenameValueLabel, 1, 1);
  contentsLayout->addWidget(this->paramsText, 2, 0, 1, 2);
  contentsLayout->setAlignment(this->nameValueLabel, Qt::AlignLeft);
  contentsLayout->setAlignment(this->filenameValueLabel, Qt::AlignLeft);

  // Buttons
  QPushButton *cancelButton = new QPushButton(tr("Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *OKButton = new QPushButton(tr("OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(contentsLayout);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
ModelPluginInspector::~ModelPluginInspector()
{
}

/////////////////////////////////////////////////
void ModelPluginInspector::SetName(const std::string &_name)
{
  this->nameValueLabel->setText(tr(_name.c_str()));
}

/////////////////////////////////////////////////
std::string ModelPluginInspector::Name() const
{
  return this->nameValueLabel->text().toStdString();
}

/////////////////////////////////////////////////
void ModelPluginInspector::SetFilename(const std::string &_filename)
{
  this->filenameValueLabel->setText(tr(_filename.c_str()));
}

/////////////////////////////////////////////////
std::string ModelPluginInspector::Filename() const
{
  return this->filenameValueLabel->text().toStdString();
}

/////////////////////////////////////////////////
void ModelPluginInspector::SetParams(const std::string &_params)
{
  this->paramsText->setPlainText(tr(_params.c_str()));
}

/////////////////////////////////////////////////
std::string ModelPluginInspector::Params() const
{
  return this->paramsText->toPlainText().toStdString();
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void ModelPluginInspector::OnOK()
{
  emit Accepted();
}

/////////////////////////////////////////////////
void ModelPluginInspector::enterEvent(QEvent */*_event*/)
{
  QApplication::setOverrideCursor(Qt::ArrowCursor);
}
