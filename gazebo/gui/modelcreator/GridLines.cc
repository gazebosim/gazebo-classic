 /*
 * Copyright 2012 Nate Koenig
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

#include "GridLines.hh"

/////////////////////////////////////////////////
GridLines::GridLines( int _w, int _h ) : QGraphicsItem(), width(_w), height(_h),
  space(10)
{
}

/////////////////////////////////////////////////
void GridLines::paint (QPainter *_painter,
                       const QStyleOptionGraphicsItem */*_option*/,
                       QWidget */*_widget*/)
{
  // re-draw the grid lines from 0,0 to this->width, this->height
  // do horizontal first
  QColor c(200,200,255,125);

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
  return QRectF ( static_cast<qreal>(0),
                  static_cast<qreal>(0),
                  static_cast<qreal>(this->width),
                  static_cast<qreal>(this->height));
}

/////////////////////////////////////////////////
void GridLines::HandleWindowSizeChanged(int _w, int _h)
{
  this->width = _w;
  this->height = _h;
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
void GridLines::mousePressEvent ( QGraphicsSceneMouseEvent *_event)
{
}

/////////////////////////////////////////////////
void GridLines::mouseMoveEvent ( QGraphicsSceneMouseEvent *_event)
{
}
