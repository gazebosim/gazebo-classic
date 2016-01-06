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

#include "gazebo/gui/building/GridLines.hh"
#include "gazebo/gui/building/GridLinesPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GridLines::GridLines(int _w, int _h) : QGraphicsItem(),
    dataPtr(new GridLinesPrivate())
{
  this->setFlag(QGraphicsItem::ItemIsSelectable, false);
  this->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);

  this->dataPtr->width = _w;
  this->dataPtr->height = _h;
  this->dataPtr->space = 10;
}

/////////////////////////////////////////////////
void GridLines::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  // re-draw the grid lines from 0,0 to this->dataPtr->width,
  // this->dataPtr->height
  // do horizontal first
  QColor c(200, 200, 255, 125);

  _painter->setPen(c);

  for (int yPos = -this->dataPtr->height; yPos < this->dataPtr->height;
      yPos+=this->dataPtr->space)
  {
    _painter->drawLine(-this->dataPtr->width, yPos, this->dataPtr->width, yPos);
  }

  for (int xPos = -this->dataPtr->width; xPos < this->dataPtr->width;
      xPos+=this->dataPtr->space)
  {
    _painter->drawLine(xPos, -this->dataPtr->height, xPos,
        this->dataPtr->height);
  }
}

/////////////////////////////////////////////////
QRectF GridLines::boundingRect() const
{
  return QRectF (0, 0, 0, 0);
}

/////////////////////////////////////////////////
void GridLines::SetSize(int _width, int _height)
{
  this->dataPtr->width = _width;
  this->dataPtr->height = _height;
}

/////////////////////////////////////////////////
void GridLines::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->ignore();
}

/////////////////////////////////////////////////
void GridLines::mouseMoveEvent(QGraphicsSceneMouseEvent *_event)
{
  _event->ignore();
}
