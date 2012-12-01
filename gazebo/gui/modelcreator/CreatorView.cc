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

#include "CreatorView.hh"
#include "SelectableLineSegment.hh"
#include "GridLines.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CreatorView::CreatorView(QWidget *_parent)
  : QGraphicsView(_parent)
{
  this->setObjectName("creatorScene");

  this->drawMode = None;
  this->drawInProgress = false;

}

/////////////////////////////////////////////////
CreatorView::~CreatorView()
{

}

/////////////////////////////////////////////////
void CreatorView::mousePressEvent(QMouseEvent *_event)
{
  this->drawMode = Wall;
  qDebug() << " mouse press event " << mapToScene(_event->pos());
}

/////////////////////////////////////////////////
void CreatorView::mouseReleaseEvent(QMouseEvent *_event)
{
  switch (drawMode)
  {
    case None:
      break;
    case Wall:
      if (!drawInProgress)
      {
        drawInProgress = true;
        lastLinePos = _event->pos();
      }
      this->DrawLines(_event->pos());
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
void CreatorView::mouseMoveEvent(QMouseEvent *_event)
{

  QPointF point = mapToScene(lastLinePos) - mapToScene(_event->pos());
  QPoint p(static_cast<int>(point.x()), static_cast<int>(point.y()));

  qDebug() << " mouse move event " << mapToScene(_event->pos()) << mapToScene(lastLinePos) << p;
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
          line->SetCornerPosition(p, 1);
        }
      }
      break;
    default:
      break;
  }
}


/////////////////////////////////////////////////
void CreatorView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  drawMode =  None;
  drawInProgress = false;
}

/////////////////////////////////////////////////
void CreatorView::DrawLines(QPoint _pos)
{
  QPointF pointStart = mapToScene(lastLinePos);
  QPointF pointEnd = mapToScene(_pos);
  SelectableLineSegment* line = new SelectableLineSegment(
    pointStart, pointEnd);
  lastLinePos = _pos;
  lineList.push_back(line);
  scene()->addItem(line);
}
