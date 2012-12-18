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
#include "WindowItem.hh"
#include "WindowDoorInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowItem::WindowItem(): RectItem()
{
  this->windowDepth = 10;
  this->windowHeight = 0;
  this->windowWidth = 50;
  this->windowSideBar = 10;
  this->windowPos = this->pos();

  this->width = this->windowWidth;
  this->height = this->windowDepth + this->windowSideBar;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);
}

/////////////////////////////////////////////////
WindowItem::~WindowItem()
{
}

/////////////////////////////////////////////////
void WindowItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem *_option, QWidget *_widget)
{
  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

  QPointF midLeft(this->drawingOriginX, this->drawingHeight/2.0);
  QPointF midRight(this->drawingWidth, this->drawingHeight/2.0);

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->showCorners(this->isSelected());

  QPen windowPen;
  windowPen.setStyle(Qt::SolidLine);
  windowPen.setColor(borderColor);
  _painter->setPen(windowPen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);

  windowPen.setWidth(this->windowDepth);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->windowDepth/2, 0),
      midRight - QPointF(this->windowDepth/2, 0));

  double borderSize = 1.0;
  windowPen.setColor(Qt::white);
  windowPen.setWidth(this->windowDepth - borderSize*2);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->windowDepth/2.0, 0),
      midRight - QPointF(this->windowDepth/2.0 - 0.5, 0) );

  this->windowWidth = this->drawingWidth;
  this->windowPos = this->pos();
  _painter->restore();

//  QGraphicsPolygonItem::paint(_painter, _option, _widget);
}

/////////////////////////////////////////////////
void WindowItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  WindowDoorInspectorDialog dialog(0);
  dialog.SetWidth(this->windowWidth);
  dialog.SetHeight(this->windowHeight);
  dialog.SetDepth(this->windowDepth);
  dialog.SetPosition(this->windowPos);
  if (dialog.exec() == QDialog::Accepted)
  {
    this->SetSize(QSize(dialog.GetWidth(),
        dialog.GetDepth() + this->windowSideBar));
    this->setPos(dialog.GetPosition());
    this->windowWidth = dialog.GetWidth();
    this->windowHeight = dialog.GetHeight();
    this->windowDepth = dialog.GetDepth();
    this->windowPos = dialog.GetPosition();
  }
  _event->setAccepted(true);
}
