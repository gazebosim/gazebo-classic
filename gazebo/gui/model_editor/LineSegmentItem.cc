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

#include "LineSegmentItem.hh"
#include "WallInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LineSegmentItem::LineSegmentItem(QGraphicsItem *_parent, int _index)
    : QGraphicsLineItem(_parent), index(_index), start(0, 0), end(0, 0)
{
  if (_parent)
    this->setParentItem(_parent);
  this->setAcceptHoverEvents(true);
}

/////////////////////////////////////////////////
LineSegmentItem::~LineSegmentItem()
{
}

/////////////////////////////////////////////////
void LineSegmentItem::SetLine(QPointF _start, QPointF _end)
{
  this->start = _start;
  this->end = _end;
  this->setLine(this->start.x(), this->start.y(), this->end.x(), this->end.y());
}

/////////////////////////////////////////////////
void LineSegmentItem::SetStartPoint(QPointF _start)
{
  this->start = _start;
  this->setLine(this->start.x(), this->start.y(), this->end.x(), this->end.y());
}

/////////////////////////////////////////////////
void LineSegmentItem::SetEndPoint(QPointF _end)
{
  this->end = _end;
  this->setLine(this->start.x(), this->start.y(), this->end.x(), this->end.y());
}

/////////////////////////////////////////////////
int LineSegmentItem::GetIndex()
{
  return index;
}

/////////////////////////////////////////////////
void LineSegmentItem::SetMouseState(int _state)
{
  this->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int LineSegmentItem::GetMouseState()
{
  return this->mouseButtonState;
}

/////////////////////////////////////////////////
void LineSegmentItem::SetMouseDownX(double _x)
{
  this->mouseDownX = _x;
}

/////////////////////////////////////////////////
void LineSegmentItem::SetMouseDownY(double _y)
{
  this->mouseDownY = _y;
}

/////////////////////////////////////////////////
qreal LineSegmentItem::GetMouseDownX()
{
  return this->mouseDownX;
}

/////////////////////////////////////////////////
qreal LineSegmentItem::GetMouseDownY()
{
  return this->mouseDownY;
}

/////////////////////////////////////////////////
void LineSegmentItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void LineSegmentItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void LineSegmentItem::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void LineSegmentItem::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
  QColor lineColor = Qt::black;
  QPen linePen = this->pen();
  linePen.setColor(lineColor);
  this->setPen(linePen);
}

/////////////////////////////////////////////////
void LineSegmentItem::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  QColor lineColor = Qt::red;
  QPen linePen = this->pen();
  linePen.setColor(lineColor);
  this->setPen(linePen);
}
