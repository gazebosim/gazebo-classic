/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/building/RotateHandle.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RotateHandle::RotateHandle(QGraphicsItem *_parent) : QGraphicsItem(_parent)
{
  this->setParentItem(_parent);
  this->setAcceptHoverEvents(true);
  this->handleSize = 6;
  this->handleOffsetHeight = 10;
  this->origin = QPointF(0, 0);
  this->handleOffset = this->origin - QPointF(0, handleOffsetHeight);
}

/////////////////////////////////////////////////
RotateHandle::~RotateHandle()
{
}

/////////////////////////////////////////////////
void RotateHandle::SetMouseState(int _state)
{
  this->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int RotateHandle::GetMouseState() const
{
  return this->mouseButtonState;
}

/////////////////////////////////////////////////
void RotateHandle::SetMouseDownX(double _x)
{
  this->mouseDownX = _x;
}

/////////////////////////////////////////////////
void RotateHandle::SetMouseDownY(double _y)
{
  this->mouseDownY = _y;
}

/////////////////////////////////////////////////
double RotateHandle::GetMouseDownX() const
{
  return this->mouseDownX;
}

/////////////////////////////////////////////////
double RotateHandle::GetMouseDownY() const
{
  return this->mouseDownY;
}

/////////////////////////////////////////////////
void RotateHandle::mouseReleaseEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void RotateHandle::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void RotateHandle::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void RotateHandle::hoverLeaveEvent(QGraphicsSceneHoverEvent *)
{
  this->borderColor = Qt::black;
  this->update();
}

/////////////////////////////////////////////////
void RotateHandle::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  this->borderColor = Qt::red;
  this->update();
}

/////////////////////////////////////////////////
QRectF RotateHandle::boundingRect() const
{
  return QRectF(-this->handleSize/2.0,
      -(this->handleOffsetHeight + this->handleSize),
      this->handleSize, this->handleOffsetHeight + this->handleSize);
}

/////////////////////////////////////////////////
void RotateHandle::paint(QPainter *_painter, const QStyleOptionGraphicsItem *,
  QWidget *)
{
  QPen borderPen;
  borderPen.setColor(this->borderColor);

  borderPen.setStyle(Qt::SolidLine);
  _painter->setPen(borderPen);

  QRectF rotateRect(handleOffset.x() - handleSize/2.0,
      handleOffset.y() - handleSize, handleSize, handleSize);
  _painter->drawLine(origin, handleOffset);
  _painter->drawArc(rotateRect, 0, 16*360);
}
