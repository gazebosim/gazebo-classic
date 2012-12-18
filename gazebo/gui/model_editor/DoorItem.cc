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
  this->doorDepth = 10;
  this->doorHeight = 0;
  this->doorWidth = 50;

  this->width = this->doorWidth;
  this->height = this->doorDepth + this->doorWidth;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();

  this->doorPos = this->pos();

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);
}

/////////////////////////////////////////////////
DoorItem::~DoorItem()
{
}

/////////////////////////////////////////////////
void DoorItem::paint (QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->showCorners(this->isSelected());

  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

  QPen doorPen;
  doorPen.setStyle(Qt::SolidLine);
  doorPen.setColor(this->borderColor);
  _painter->setPen(doorPen);

  _painter->drawLine(topLeft, bottomLeft);
  QRect arcRect(this->drawingOriginX - this->drawingWidth,
      this->drawingOriginY - this->drawingHeight,
      this->drawingWidth*2, this->drawingHeight*2);
  _painter->drawArc(arcRect, 0, -90 * 16);

  doorPen.setWidth(this->doorDepth);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->doorDepth/2.0, 0),
      topRight - QPointF(this->doorDepth/2.0, 0));

  double borderSize = 1.0;
  doorPen.setColor(Qt::white);
  doorPen.setWidth(this->doorDepth - borderSize*2);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->doorDepth/2.0, 0),
      topRight - QPointF(this->doorDepth/2.0, 0));

  this->doorWidth = this->drawingWidth;
  this->doorPos = this->pos();

//  QGraphicsPolygonItem::paint(_painter, _option, _widget);
}

/////////////////////////////////////////////////
void DoorItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  WindowDoorInspectorDialog dialog(1);
  dialog.SetWidth(this->doorWidth);
  dialog.SetDepth(this->doorDepth);
  dialog.SetHeight(this->doorHeight);
  dialog.SetPosition(this->pos());
  if (dialog.exec() == QDialog::Accepted)
  {
    this->SetSize(QSize(dialog.GetWidth(),
        dialog.GetDepth() + this->doorWidth));
    this->setPos(dialog.GetPosition());
    this->doorWidth = dialog.GetWidth();
    this->doorHeight = dialog.GetHeight();
    this->doorDepth = dialog.GetDepth();
    this->doorPos = dialog.GetPosition();
  }
  _event->setAccepted(true);
}
