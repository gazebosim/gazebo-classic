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

#include "BuildingCreatorWidget.hh"
#include "SelectableLineSegment.hh"
#include "GridLines.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingCreatorWidget::BuildingCreatorWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("buildingCreatorWidget");
//  setBackgroundRole(QPalette::Base);
//  setAutoFillBackground(true);

  view = new QGraphicsView();
  scene = new QGraphicsScene();

  QColor c (250,250,250);
  QBrush brush (c, Qt::SolidPattern);
  scene->setBackgroundBrush(brush);

  int initialHeight = 400;
  scene->setSceneRect(-300, -initialHeight/2, 300, initialHeight);
  QHBoxLayout *canvasLayout = new QHBoxLayout(this);
  canvasLayout->addWidget(view);

  GridLines *gridLines = new GridLines (this->width(), initialHeight);
  scene->addItem(gridLines);

  SelectableLineSegment* sampleLine = new SelectableLineSegment(
    QPointF(100,100));
  sampleLine->setPos(0,0);

  scene->addItem(sampleLine);
  view->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  view->setScene(scene);
//  view->centerOn(QPointF(0, 0));
  view->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);

  qDebug() << " width height "<< _parent->width() << " " << _parent->height() << scene->width() << scene->height() << view->width() << view->height();
  canvasLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(canvasLayout);

}

/////////////////////////////////////////////////
BuildingCreatorWidget::~BuildingCreatorWidget()
{

}

/////////////////////////////////////////////////
void BuildingCreatorWidget::paintEvent(QPaintEvent * /* event */)
{
}
