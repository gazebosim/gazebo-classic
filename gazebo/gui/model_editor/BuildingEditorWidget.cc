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

#include "BuildingEditorWidget.hh"
#include "GridLines.hh"
#include "EditorView.hh"
#include "RectItem.hh"
#include "WindowItem.hh"
#include "DoorItem.hh"
#include "PolylineItem.hh"
#include "WallItem.hh"

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

  int boundingWidth = 600;
  int boundingHeight = 300;
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

  WallItem *lineItem = new WallItem(QPointF(0, 0), QPointF(-100, 100));
  scene->addItem(lineItem);

  WindowItem *windowItem = new WindowItem();
  windowItem->setPos(-100,100);
  scene->addItem(windowItem);

  DoorItem *doorItem = new DoorItem();
  doorItem->setPos(0,0);
  scene->addItem(doorItem);

  canvasLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(canvasLayout);

}

/////////////////////////////////////////////////
BuildingEditorWidget::~BuildingEditorWidget()
{
}
