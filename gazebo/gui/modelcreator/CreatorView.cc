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
  : QGraphicsView(_parent), currentLine(0)
{
  this->setObjectName("creatorView");

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

  QGraphicsView::mousePressEvent(_event);
}

/////////////////////////////////////////////////
void CreatorView::mouseReleaseEvent(QMouseEvent *_event)
{
  switch (drawMode)
  {
    case None:
      break;
    case Wall:
      this->DrawLines(_event->pos());
      break;
    default:
      break;
  }

  if (drawMode != None && !drawInProgress)
    drawInProgress = true;

  QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void CreatorView::mouseMoveEvent(QMouseEvent *_event)
{
  QPointF delta = _event->pos() - lastMousePos;
  switch (drawMode)
  {
    case None:
      break;
    case Wall:
      if (drawInProgress)
      {
        if (currentLine)
          currentLine->TranslateCorner(delta, 1);
      }
      break;
    default:
      break;
  }
  lastMousePos = _event->pos();
  QGraphicsView::mouseMoveEvent(_event);
}

/////////////////////////////////////////////////
void CreatorView::mouseDoubleClickEvent(QMouseEvent */*_event*/)
{
  drawMode =  None;
  drawInProgress = false;

  if (currentLine)
  {
    lineList.pop_back();
    scene()->removeItem(currentLine);
    delete currentLine;
    currentLine = NULL;
  }
}

/////////////////////////////////////////////////
void CreatorView::DrawLines(QPoint _pos)
{
  QPointF pointStart = mapToScene(_pos);
  QPointF pointEnd = pointStart;
  SelectableLineSegment* newLine = new SelectableLineSegment(
    pointStart, pointEnd);

  if (currentLine)
    currentLine->ConnectLine(newLine);
  currentLine = newLine;

  lastLineCornerPos = _pos;
  lastMousePos = _pos;
  lineList.push_back(newLine);
  scene()->addItem(newLine);
}
