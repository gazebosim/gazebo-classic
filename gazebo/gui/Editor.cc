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

#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/Editor.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
Editor::Editor(MainWindow *_mainWindow)
  : mainWindow(_mainWindow)
{
  this->tabWidget = NULL;
}

/////////////////////////////////////////////////
Editor::~Editor()
{
  this->tabWidget->hide();
  delete this->tabWidget;
}

/////////////////////////////////////////////////
void Editor::Init(const std::string &_objName,
                  const std::string &_tabLabel, QWidget *_widget)
{
  if (this->tabWidget)
    delete this->tabWidget;

  this->tabWidget = new QTabWidget;
  this->tabWidget->setObjectName(QString::fromStdString(_objName));
  this->tabWidget->addTab(_widget, QString::fromStdString(_tabLabel));
  this->tabWidget->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  this->tabWidget->setMinimumWidth(250);
  this->tabWidget->hide();

  this->mainWindow->AddToLeftColumn(_objName, this->tabWidget);
}

