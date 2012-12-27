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

#include "gui/model_editor/BuildingItem.hh"
#include "gui/model_editor/RectItem.hh"
#include "gui/model_editor/WindowItem.hh"
#include "gui/model_editor/WindowDoorInspectorDialog.hh"
#include "gui/model_editor/BuildingMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowItem::WindowItem(): RectItem(), BuildingItem()
{
  this->editorType = "Window";
  this->scale = BuildingMaker::conversionScale;

  this->level = 0;
  this->levelBaseHeight = 0;

  this->windowDepth = 20;
  this->windowHeight = 80;
  this->windowWidth = 80;
  this->windowSideBar = 10;
  this->windowPos = this->pos();
  this->windowElevation = 50;

  this->width = this->windowWidth;
  this->height = this->windowDepth;
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
QVector3D WindowItem::GetSize()
{
  return QVector3D(this->windowWidth, this->windowDepth, this->windowHeight);
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetScenePosition()
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->windowElevation);
}

/////////////////////////////////////////////////
double WindowItem::GetSceneRotation()
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void WindowItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  QPointF topLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF topRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF bottomLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);
  QPointF bottomRight(this->drawingOriginX  + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPointF midLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY);
  QPointF midRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY);

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

  windowPen.setWidth(this->windowDepth/2.0);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->windowDepth/4, 0),
      midRight - QPointF(this->windowDepth/4, 0));

  double borderSize = 1.0;
  windowPen.setColor(Qt::white);
  windowPen.setWidth(this->windowDepth/2.0 - borderSize*2);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->windowDepth/4.0, 0),
      midRight - QPointF(this->windowDepth/4.0 - 0.5, 0) );

  this->windowWidth = this->drawingWidth;
  this->windowDepth = this->drawingHeight;
  this->windowPos = this->pos();
  _painter->restore();

//  QGraphicsPolygonItem::paint(_painter, _option, _widget);
}

/////////////////////////////////////////////////
void WindowItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  WindowDoorInspectorDialog dialog(WindowDoorInspectorDialog::WINDOW);
  dialog.SetWidth(this->windowWidth * this->scale);
  dialog.SetHeight(this->windowHeight * this->scale);
  dialog.SetElevation(this->windowElevation * this->scale);
  dialog.SetDepth(this->windowDepth * this->scale);

  QPointF itemPos = this->windowPos * this->scale;
  itemPos.setY(-itemPos.y());
  dialog.SetPosition(itemPos);
  if (dialog.exec() == QDialog::Accepted)
  {
    this->SetSize(QSize(dialog.GetWidth() / this->scale,
        dialog.GetDepth() / this->scale));
    this->windowWidth = dialog.GetWidth() / this->scale;
    this->windowHeight = dialog.GetHeight() / this->scale;
    this->windowDepth = dialog.GetDepth() / this->scale;
    this->windowElevation = dialog.GetElevation() / this->scale;
    itemPos = dialog.GetPosition() / this->scale;
    itemPos.setY(-itemPos.y());
    this->windowPos = itemPos;
    this->setPos(this->windowPos);
    this->WindowChanged();
  }
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void WindowItem::WindowChanged()
{
  emit widthChanged(this->windowWidth);
  emit depthChanged(this->windowDepth);
  emit heightChanged(this->windowHeight);
  emit positionChanged(this->windowPos.x(), this->windowPos.y(),
      this->levelBaseHeight + this->windowElevation);
}
