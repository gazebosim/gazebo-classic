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
#include "CreatorView.hh"
#include "RectItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
BuildingCreatorWidget::BuildingCreatorWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("buildingCreatorWidget");

  CreatorView *view = new CreatorView();
  QGraphicsScene *scene = new QGraphicsScene();

  QColor c (250,250,250);
  QBrush brush (c, Qt::SolidPattern);
  //this->scene->setBackgroundBrush(brush);
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

  RectItem *item = new RectItem();
  item->setPos(200,200);
  scene->addItem(item);

//  qDebug() << " width height "<< _parent->width() << " " << _parent->height() << scene->width() << scene->height() << view->width() << view->height();
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
/*
/////////////////////////////////////////////////
void BuildingCreatorWidget::mousePressEvent(QMouseEvent *_event)
{
  this->drawMode = Wall;
  qDebug() << " mouse press event " << _event->pos();
}

/////////////////////////////////////////////////
void BuildingCreatorWidget::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  switch (drawMode)
  {
    case None:
      break;
    case Wall:
      if (!drawInProgress)
      {
        drawInProgress = true;
        lastLinePosX = _event->pos().x();
        lastLinePosY = _event->pos().y();
      }
      this->DrawLines(_event->pos());
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
void BuildingCreatorWidget::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  switch (drawMode)
  {
    case None:
      break;
    case Wall:
      if (drawInProgress)
      {
        SelectableLineSegment* line = lineList.back();
        if (line)
        {
          line->SetCornerPosition(_event->pos(), 1);
        }
      }
      this->DrawLines(_event->pos());
      break;
    default:
      break;
  }
}


/////////////////////////////////////////////////
void BuildingCreatorWidget::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  drawMode =  None;
  drawInProgress = false;
}

/////////////////////////////////////////////////
void BuildingCreatorWidget::DrawLines(QPointF _pos)
{
  SelectableLineSegment* line = new SelectableLineSegment(
    QPointF(lastLinePosX, lastLinePosY), _pos);
  lastLinePosX = _pos.x();
  lastLinePosY = _pos.y();
  lineList.push_back(line);
  this->scene->addItem(line);
}*/
