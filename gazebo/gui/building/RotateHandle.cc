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

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/building/RotateHandle.hh"
#include "gazebo/gui/building/RotateHandlePrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RotateHandle::RotateHandle(QGraphicsItem *_parent)
  : QGraphicsItem(_parent), dataPtr(new RotateHandlePrivate)
{
  this->setParentItem(_parent);
  this->setAcceptHoverEvents(true);
  this->dataPtr->handleSize = 6;
  this->dataPtr->handleOffsetHeight = 10;
  this->dataPtr->origin = ignition::math::Vector2d(0, 0);
  this->dataPtr->borderColor = common::Color::Black;
  this->dataPtr->handleOffset = this->dataPtr->origin -
      ignition::math::Vector2d(0, this->dataPtr->handleOffsetHeight);
}

/////////////////////////////////////////////////
RotateHandle::~RotateHandle()
{
}

/////////////////////////////////////////////////
void RotateHandle::SetMouseState(const int _state)
{
  this->dataPtr->mouseButtonState = _state;
}

/////////////////////////////////////////////////
int RotateHandle::MouseState() const
{
  return this->dataPtr->mouseButtonState;
}

/////////////////////////////////////////////////
void RotateHandle::SetMouseDownX(const double _x)
{
  this->dataPtr->mouseDownX = _x;
}

/////////////////////////////////////////////////
void RotateHandle::SetMouseDownY(const double _y)
{
  this->dataPtr->mouseDownY = _y;
}

/////////////////////////////////////////////////
double RotateHandle::MouseDownX() const
{
  return this->dataPtr->mouseDownX;
}

/////////////////////////////////////////////////
double RotateHandle::MouseDownY() const
{
  return this->dataPtr->mouseDownY;
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
  this->dataPtr->borderColor = common::Color::Black;
  this->update();
}

/////////////////////////////////////////////////
void RotateHandle::hoverEnterEvent(QGraphicsSceneHoverEvent *)
{
  this->dataPtr->borderColor = common::Color::Red;
  this->update();
}

/////////////////////////////////////////////////
QRectF RotateHandle::boundingRect() const
{
  return QRectF(-this->dataPtr->handleSize/2.0,
      -(this->dataPtr->handleOffsetHeight + this->dataPtr->handleSize),
      this->dataPtr->handleSize,
      this->dataPtr->handleOffsetHeight + this->dataPtr->handleSize);
}

/////////////////////////////////////////////////
void RotateHandle::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_options*/, QWidget */*_widget*/)
{
  QPen borderPen;
  borderPen.setColor(Conversions::Convert(this->dataPtr->borderColor));

  borderPen.setStyle(Qt::SolidLine);
  _painter->setPen(borderPen);

  QRectF rotateRect(this->dataPtr->handleOffset.X() -
      this->dataPtr->handleSize/2.0,
      this->dataPtr->handleOffset.Y() - this->dataPtr->handleSize,
      this->dataPtr->handleSize, this->dataPtr->handleSize);
  _painter->drawLine(this->dataPtr->origin.X(), this->dataPtr->origin.Y(),
      this->dataPtr->handleOffset.X(), this->dataPtr->handleOffset.Y());
  _painter->drawArc(rotateRect, 0, 16*360);
}
