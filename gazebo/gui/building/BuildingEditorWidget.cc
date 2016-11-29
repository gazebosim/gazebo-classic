/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/building/BuildingEditorWidget.hh"
#include "gazebo/gui/building/BuildingEditorWidgetPrivate.hh"
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/LevelWidget.hh"
#include "gazebo/gui/building/ScaleWidget.hh"

using namespace gazebo;
using namespace gui;

QCursor BuildingEditorWidget::rotateCursor;

/////////////////////////////////////////////////
BuildingEditorWidget::BuildingEditorWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new BuildingEditorWidgetPrivate)
{
  this->setObjectName("buildingEditorWidget");

  QPixmap rotatePixmap(":/images/rotate_object.png");
  rotateCursor = QCursor(rotatePixmap);

  EditorView *view = new EditorView();
  this->dataPtr->scene = new QGraphicsScene();

  QColor c(250, 250, 250);
  QBrush brush(c, Qt::SolidPattern);
  this->dataPtr->scene->setBackgroundBrush(brush);

  this->dataPtr->minimumWidth = 1240*2;
  this->dataPtr->minimumHeight = 1024*2;
  this->dataPtr->scene->setSceneRect(-this->dataPtr->minimumWidth/2,
                                     -this->dataPtr->minimumHeight/2,
                                      this->dataPtr->minimumWidth,
                                      this->dataPtr->minimumHeight);
  QHBoxLayout *canvasLayout = new QHBoxLayout(this);
  canvasLayout->addWidget(view);
  canvasLayout->setAlignment(Qt::AlignHCenter);

  view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  view->setScene(this->dataPtr->scene);
  view->centerOn(QPointF(0, 0));
  view->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
  view->setDragMode(QGraphicsView::ScrollHandDrag);

  this->dataPtr->levelWidget = new LevelWidget(this);
  this->dataPtr->levelWidget->resize(250, 50);
//  QGraphicsProxyWidget* proxyWidget = scene->addWidget(levelWidget);
//  proxyWidget->setFlag(QGraphicsItem::ItemIgnoresTransformations);
//  proxyWidget->setPos(QPointF(-levelWidget->width() / 2, 0));

  this->dataPtr->scaleWidget = new ScaleWidget(this);
  this->dataPtr->scaleWidget->resize(150, 50);

  canvasLayout->setContentsMargins(0, 0, 0, 0);
  canvasLayout->setSpacing(0);
  this->setLayout(canvasLayout);
}

/////////////////////////////////////////////////
BuildingEditorWidget::~BuildingEditorWidget()
{
}

/////////////////////////////////////////////////
void BuildingEditorWidget::resizeEvent(QResizeEvent *_event)
{
  qreal boundingWidth =
      std::max(this->dataPtr->minimumWidth, _event->size().width());
  boundingWidth =
      std::max(boundingWidth, this->dataPtr->scene->sceneRect().width());
  qreal boundingHeight = std::max(this->dataPtr->minimumHeight,
      _event->size().height());
  boundingHeight =
      std::max(boundingHeight, this->dataPtr->scene->sceneRect().height());

  this->dataPtr->scene->setSceneRect(-boundingWidth/2, -boundingHeight/2,
      boundingWidth, boundingHeight);

  this->dataPtr->levelWidget->move(_event->size().width()/2
      - this->dataPtr->levelWidget->size().width()/2, 0);

  this->dataPtr->scaleWidget->move(20, _event->size().height()
      - this->dataPtr->scaleWidget->size().height() - 20);
}
