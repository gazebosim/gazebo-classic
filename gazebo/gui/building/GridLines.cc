/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GridLines::GridLines(int _w, int _h) : QGraphicsItem(), width(_w), height(_h),
  space(10)
{
  this->setFlag(QGraphicsItem::ItemIsSelectable, false);
  this->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
}

/////////////////////////////////////////////////
GridLines::~GridLines()
{
}

/////////////////////////////////////////////////
void GridLines::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  // re-draw the grid lines from 0,0 to this->width, this->height
  // do horizontal first
  QColor c(200, 200, 255, 125);

  _painter->setPen(c);

  for (int yPos = -this->height; yPos < this->height; yPos+=this->space)
  {
    _painter->drawLine(-this->width, yPos, this->width, yPos);
  }

  for (int xPos = -this->width; xPos < this->width; xPos+=this->space)
  {
    _painter->drawLine(xPos, -this->height, xPos, this->height);
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
  this->width = _width;
  this->height = _height;
}

/////////////////////////////////////////////////
void GridLines::mouseMoveEvent(QGraphicsSceneDragDropEvent *)
{
}

/////////////////////////////////////////////////
void GridLines::mousePressEvent(QGraphicsSceneDragDropEvent *)
{
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
