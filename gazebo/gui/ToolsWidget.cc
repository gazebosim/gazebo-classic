/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <boost/bind.hpp>

#include "gazebo/common/Events.hh"
#include "gazebo/gui/JointControlWidget.hh"
#include "gazebo/gui/ToolsWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ToolsWidget::ToolsWidget(QWidget *_parent)
  : QWidget(_parent)
{
  // This name is used in the qt style sheet
  this->setObjectName("toolsWidget");

  // Create the joint control tool
  this->jointControlWidget = new JointControlWidget(this);

  // Create the main tab widget for all the tools
  this->tabWidget = new QTabWidget();

  // Use the embedded tab style from the stylesheet
  this->tabWidget->setObjectName("embeddedTab");

  // Add the joint control tool
  this->tabWidget->addTab(this->jointControlWidget, "Joints");

  // Make the widget fill the full space
  this->tabWidget->setSizePolicy(QSizePolicy::Expanding,
                                 QSizePolicy::Expanding);
  this->tabWidget->setMinimumWidth(250);

  // Create the main layout for this widget
  QVBoxLayout *mainLayout = new QVBoxLayout;

  // Add the tab widget to the main layout
  mainLayout->addWidget(this->tabWidget);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(0, 0, 0, 0);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);

  // Listen to entity selection events
  this->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
        boost::bind(&ToolsWidget::OnSetSelectedEntity, this, _1, _2)));
}

/////////////////////////////////////////////////
ToolsWidget::~ToolsWidget()
{
}

/////////////////////////////////////////////////
void ToolsWidget::OnSetSelectedEntity(const std::string &_name,
                                      const std::string &/*_mode*/)
{
  this->jointControlWidget->SetModelName(_name);
}
