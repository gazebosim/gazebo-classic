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

#include "gui/model_editor/BuildingEditorWidget.hh"
#include "gui/model_editor/GridLines.hh"
#include "gui/model_editor/EditorView.hh"
#include "gui/model_editor/EditorItem.hh"
#include "gui/model_editor/RectItem.hh"
#include "gui/model_editor/WindowItem.hh"
#include "gui/model_editor/DoorItem.hh"
#include "gui/model_editor/PolylineItem.hh"
#include "gui/model_editor/WallItem.hh"
#include "gui/model_editor/LevelWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingEditorWidget::BuildingEditorWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("buildingEditorWidget");

  EditorView *view = new EditorView();
  QGraphicsScene *scene = new QGraphicsScene();

  QColor c (250,250,250);
  QBrush brush (c, Qt::SolidPattern);
  scene->setBackgroundBrush(brush);

  int boundingWidth = 800;
  int boundingHeight = 600;
  scene->setSceneRect(-boundingWidth/2, -boundingHeight/2,
                      boundingWidth, boundingHeight);
  QHBoxLayout *canvasLayout = new QHBoxLayout(this);
  canvasLayout->addWidget(view);

  GridLines *gridLines = new GridLines (boundingWidth, boundingHeight);
  scene->addItem(gridLines);

  view->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  view->setScene(scene);
  view->centerOn(QPointF(0, 0));
  view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

/*  WallItem *lineItem = new WallItem(QPointF(0, 0), QPointF(-100, 100));
  scene->addItem(lineItem);

  WindowItem *windowItem = new WindowItem();
  windowItem->setPos(-100,100);
  scene->addItem(windowItem);

  DoorItem *doorItem = new DoorItem();
  doorItem->setPos(0,0);
  scene->addItem(doorItem);*/

  LevelWidget *levelWidget = new LevelWidget(this);
  levelWidget->resize(140,50);
//  QGraphicsProxyWidget* proxyWidget = scene->addWidget(levelWidget);
//  proxyWidget->setFlag(QGraphicsItem::ItemIgnoresTransformations);
//  proxyWidget->setPos(QPointF(-levelWidget->width() / 2, 0));

  canvasLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(canvasLayout);
}

/////////////////////////////////////////////////
BuildingEditorWidget::~BuildingEditorWidget()
{
}
