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
  this->width = 100;
  this->height = 10;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();
}

WindowItem::~WindowItem()
{
}

/////////////////////////////////////////////////
void WindowItem::paint (QPainter *_painter, const QStyleOptionGraphicsItem *,
    QWidget *)
{
  QPointF topLeft(this->drawingOriginX, this->drawingOriginY);
  QPointF topRight(this->drawingWidth, this->drawingOriginY);
  QPointF bottomLeft(this->drawingOriginX, this->drawingHeight);
  QPointF bottomRight(this->drawingWidth, this->drawingHeight);

  QPointF midLeft(this->drawingOriginX, this->drawingHeight/2.0);
  QPointF midRight(this->drawingWidth, this->drawingHeight/2.0);



  if (this->isSelected())
  {
    this->showBoundingBox(_painter);
  }
  this->showCorners(this->isSelected());



  QPen pen;
  pen.setStyle(Qt::SolidLine);
  pen.setColor(borderColor);
  _painter->setPen(pen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);
  _painter->drawLine(midLeft, midRight);




}

/////////////////////////////////////////////////
void WindowItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  WindowDoorInspectorDialog dialog(0);
  dialog.SetWidth(this->GetWidth());
  dialog.SetHeight(this->GetHeight());
  if (dialog.exec() == QDialog::Accepted)
  {
    this->SetSize(QSize(dialog.GetWidth(), dialog.GetHeight()));
  }
  _event->setAccepted(true);
}
