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

#include "gazebo/gui/model_editor/EditorItem.hh"
#include "gazebo/gui/model_editor/RectItem.hh"
#include "gazebo/gui/model_editor/BuildingItem.hh"
#include "gazebo/gui/model_editor/BuildingMaker.hh"
#include "gazebo/gui/model_editor/FloorItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
FloorItem::FloorItem(): RectItem(), BuildingItem()
{
  this->editorType = "Floor";
  this->scale = BuildingMaker::conversionScale;

  this->level = 0;
  this->levelBaseHeight = 0;

  this->floorWidth = 100;
  this->floorDepth = 100;
  this->floorHeight = 10;

  this->floorPos = this->scenePos();
//  this->setVisible(false);
}

/////////////////////////////////////////////////
FloorItem::~FloorItem()
{
}

/////////////////////////////////////////////////
/*void FloorItem::SetSize(QVector3D _size)
{
  this->floorWidth = _size.x();
  this->floorDepth = _size.y();
  this->floorHeight = _size.z();
  this->SetSize(QSize(this->floorWidth, this->floorDepth));
  this->FloorChanged();
}*/

/////////////////////////////////////////////////
QVector3D FloorItem::GetSize() const
{
  return QVector3D(this->floorWidth, this->floorDepth, this->floorHeight);
}

/////////////////////////////////////////////////
QVector3D FloorItem::GetScenePosition() const
{
  return QVector3D(this->floorPos.x(), this->floorPos.y(), 0);
}

/////////////////////////////////////////////////
double FloorItem::GetSceneRotation() const
{
  return 0;
}

/////////////////////////////////////////////////
void FloorItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
  if (!this->isSelected())
    this->scene()->clearSelection();

  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void FloorItem::paint (QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowCorners(this->isSelected());

  QPointF topLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF topRight(this->drawingOriginX + this->drawingWidth/2,
      this->drawingOriginY - this->drawingHeight/2);
  QPointF bottomLeft(this->drawingOriginX - this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);
  QPointF bottomRight(this->drawingOriginX  + this->drawingWidth/2,
      this->drawingOriginY + this->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(borderColor);
  _painter->setPen(rectPen);

  _painter->drawLine(topLeft, topRight);
  _painter->drawLine(topRight, bottomRight);
  _painter->drawLine(bottomRight, bottomLeft);
  _painter->drawLine(bottomLeft, topLeft);

  this->floorWidth = this->drawingWidth;
  this->floorDepth = this->drawingHeight;
  this->floorPos = this->scenePos();

//  QGraphicsPolygonItem::paint(_painter, _option, _widget);
}

/////////////////////////////////////////////////
void FloorItem::FloorChanged()
{
  emit widthChanged(this->floorWidth);
  emit depthChanged(this->floorDepth);
  emit heightChanged(this->floorHeight);
  emit positionChanged(this->floorPos.x(), this->floorPos.y(),
      this->levelBaseHeight/* + this->floorElevation*/);
}

/////////////////////////////////////////////////
void FloorItem::SizeChanged()
{
  emit widthChanged(this->floorWidth);
  emit depthChanged(this->floorDepth);
}
