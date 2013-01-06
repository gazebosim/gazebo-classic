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

#include "gazebo/gui/model_editor/EditorWidget.hh"
#include "gazebo/gui/model_editor/BuildingEditorWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorWidget::EditorWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("editorWidget");

  this->buildingEditorWidget = new BuildingEditorWidget(this);

  QHBoxLayout *editorLayout = new QHBoxLayout;
  editorLayout->setContentsMargins(0, 0, 0, 0);
  editorLayout->addWidget(this->buildingEditorWidget);
  editorLayout->setContentsMargins(0, 0, 0, 0);
  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->buildingEditorWidget->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  mainLayout->addWidget(this->buildingEditorWidget);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);
}

/////////////////////////////////////////////////
EditorWidget::~EditorWidget()
{
}

/////////////////////////////////////////////////
QWidget *EditorWidget::GetBuildingEditor() const
{
  return this->buildingEditorWidget;
}

/////////////////////////////////////////////////
void EditorWidget::SetMode(int mode)
{
  /// TODO: 2 modes, building and model,
  ///       create enums for these
  switch (mode)
  {
    case 0:
      this->buildingEditorWidget->hide();
      break;
    case 1:
      this->buildingEditorWidget->show();
      break;
    default:
      break;
  }

}
