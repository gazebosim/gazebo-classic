/*
 * Copyright 2011 Nate Koenig
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

#include "gui/EditorWidget.hh"
#include "gui/BuildingCreatorWidget.hh"

using namespace gazebo;
using namespace gui;

EditorWidget::EditorWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("editorWidget");

  this->editorFrame = new QFrame;
  this->editorFrame->setFrameShape(QFrame::NoFrame);
  this->editorFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->editorFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  this->buildingCreatorWidget = new BuildingCreatorWidget(this);

  QHBoxLayout *editorLayout = new QHBoxLayout;
  editorLayout->setContentsMargins(0, 0, 0, 0);

  this->editorFrame->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  editorLayout->addWidget(this->buildingCreatorWidget);
  editorLayout->setContentsMargins(0, 0, 0, 0);
  this->editorFrame->setLayout(editorLayout);

  this->editorFrame->setMouseTracking(true);
  this->setMouseTracking(true);
}

EditorWidget::~EditorWidget()
{

}

void EditorWidget::SetMode(int mode)
{
  /// TODO: 2 modes, building and model,
  ///       create enums for these
  switch (mode)
  {
    case 0:
      this->buildingCreatorWidget->hide();
      break;
    case 1:
      this->buildingCreatorWidget->show();
      break;
    default:
      break;
  }

}
/*
QSize EditorWidget::sizeHint() const
{
  return QSize(100, 100);
}*/
