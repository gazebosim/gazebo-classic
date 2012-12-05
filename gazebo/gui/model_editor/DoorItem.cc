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

#include "RectItem.hh"
#include "DoorItem.hh"
#include "WindowDoorInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DoorItem::DoorItem(): RectItem()
{
}

DoorItem::~DoorItem()
{
}

/////////////////////////////////////////////////
void DoorItem::paint (QPainter *_painter, const QStyleOptionGraphicsItem *,
    QWidget *)
{
  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

  QPen pen;
  pen.setStyle(Qt::SolidLine);
  pen.setColor(outterBorderColor);
  _painter->setPen(pen);

  _painter->drawLine(topLeft, topRight);
  _painter->drawLine(topLeft, bottomLeft);
  QRect arcRect(this->drawingOriginX - this->drawingWidth,
      this->drawingOriginY - this->drawingHeight,
      this->drawingWidth*2, this->drawingHeight*2);
  _painter->drawArc(arcRect, 0, -90 * 16);
}

/////////////////////////////////////////////////
void DoorItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  qDebug() << " double click door";
  WindowDoorInspectorDialog dialog(0);
  if (dialog.exec() == QDialog::Accepted)
  {
  }
  _event->setAccepted(true);
}
