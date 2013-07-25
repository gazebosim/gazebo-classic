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

#include "gazebo/gui/model/PartGeneralTab.hh"
#include "gazebo/gui/model/PartVisualTab.hh"
#include "gazebo/gui/model/PartInspector.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
PartInspector::PartInspector(QWidget *_parent) : QDialog(_parent)
{
  this->setObjectName("PartInspector");
  this->setWindowTitle(tr("Part Inspector"));

  QLabel *partLabel = new QLabel(tr("Name:"));
  this->partNameLabel = new QLabel(tr(""));

  QHBoxLayout *nameLayout = new QHBoxLayout;
  nameLayout->addWidget(partLabel);
  nameLayout->addWidget(partNameLabel);

  this->generalTab = new PartGeneralTab;
  this->visualTab = new PartVisualTab;

  // Create the main tab widget for all components in a part
  this->tabWidget = new QTabWidget();
  this->tabWidget->setObjectName("partInspectorTab");
  this->tabWidget->addTab(this->generalTab, "General");
  this->tabWidget->addTab(this->visualTab, "Visual");

  QHBoxLayout *buttonsLayout = new QHBoxLayout;
  QPushButton *cancelButton = new QPushButton(tr("&Cancel"));
  connect(cancelButton, SIGNAL(clicked()), this, SLOT(OnCancel()));
  QPushButton *applyButton = new QPushButton(tr("&Apply"));
  connect(applyButton, SIGNAL(clicked()), this, SLOT(OnApply()));
  QPushButton *OKButton = new QPushButton(tr("&OK"));
  OKButton->setDefault(true);
  connect(OKButton, SIGNAL(clicked()), this, SLOT(OnOK()));
  buttonsLayout->addWidget(cancelButton);
  buttonsLayout->addWidget(applyButton);
  buttonsLayout->addWidget(OKButton);
  buttonsLayout->setAlignment(Qt::AlignRight);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(nameLayout);
  mainLayout->addWidget(tabWidget);
  mainLayout->addLayout(buttonsLayout);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
PartInspector::~PartInspector()
{
}

/////////////////////////////////////////////////
PartGeneralTab *PartInspector::GetGeneral() const
{
  return this->generalTab;
}

/////////////////////////////////////////////////
PartVisualTab *PartInspector::GetVisual() const
{
  return this->visualTab;
}


/////////////////////////////////////////////////
void PartInspector::OnCancel()
{
  this->close();
}

/////////////////////////////////////////////////
void PartInspector::OnApply()
{
  emit Applied();
}

/////////////////////////////////////////////////
void PartInspector::OnOK()
{
  emit Applied();
  this->accept();
}
