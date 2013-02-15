/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/gui/LogPlayWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LogPlayWidget::LogPlayWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("logPlayWidget");
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QPushButton *fileButton = new QPushButton("Browse");
  connect(fileButton, SIGNAL(clicked()), this, SLOT(OnFileButton()));

  QSlider *slider = new QSlider(Qt::Horizontal);
  slider->setTickInterval(10);
  slider->setSingleStep(1);

  mainLayout->addWidget(fileButton);
  mainLayout->addWidget(slider);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
LogPlayWidget::~LogPlayWidget()
{
}

/////////////////////////////////////////////////
void LogPlayWidget::OnFileButton()
{
  std::string filename = QFileDialog::getOpenFileName(this,
      tr("Open Log File"), "~/.gazebo/log",
      tr("Log Files (*.log)")).toStdString();

  if (!filename.empty())
  {
    gzmsg << "File[" << filename << "]\n";
  }
}
