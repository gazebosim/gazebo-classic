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

#include "gazebo/gui/TerrainEditorPalette.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TerrainEditorPalette::TerrainEditorPalette(QWidget *_parent)
    : QWidget(_parent)
{
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QPushButton *raiseButton = new QPushButton;
  raiseButton->setIcon(QIcon(":/images/wall.png"));
  raiseButton->setIconSize(QSize(30, 60));
  raiseButton->setFlat(true);
  connect(raiseButton, SIGNAL(clicked()), this, SLOT(OnRaise()));

  QPushButton *lowerButton = new QPushButton;
  lowerButton->setIcon(QIcon(":/images/wall.png"));
  lowerButton->setIconSize(QSize(30, 60));
  lowerButton->setFlat(true);
  connect(lowerButton, SIGNAL(clicked()), this, SLOT(OnLower()));

  mainLayout->addWidget(raiseButton);
  mainLayout->setAlignment(Qt::AlignTop | Qt::AlignHCenter);

  this->setObjectName("terrainEditorPalette");
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
TerrainEditorPalette::~TerrainEditorPalette()
{
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnRaise()
{
  printf("Raise\n");
  gui::events::manipMode("raise_terrain");
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnLower()
{
  printf("Lower\n");
  gui::events::manipMode("lower_terrain");
}

/////////////////////////////////////////////////
void TerrainEditorPalette::OnSave()
{
}
