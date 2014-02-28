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

#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/WallItem.hh"
#include "gazebo/gui/building/LineSegmentItem.hh"
#include "gazebo/gui/building/FloorItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
FloorItem::FloorItem(): RectItem(), BuildingItem()
{
  this->editorType = "Floor";

  this->level = 0;
  this->levelBaseHeight = 0;

  this->floorWidth = 100;
  this->floorDepth = 100;
  this->floorHeight = 10;

  this->floorPos = this->scenePos();

  this->setFlag(QGraphicsItem::ItemIsSelectable, false);
  this->dirty = false;

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(RecalculateBoundingBox()));
  timer->start(300);
}

/////////////////////////////////////////////////
FloorItem::~FloorItem()
{
}


/////////////////////////////////////////////////
QVector3D FloorItem::GetSize() const
{
  return QVector3D(this->floorWidth, this->floorDepth, this->floorHeight);
}

/////////////////////////////////////////////////
QVector3D FloorItem::GetScenePosition() const
{
  return QVector3D(this->floorPos.x(), this->floorPos.y(),
      this->levelBaseHeight);
}

/////////////////////////////////////////////////
double FloorItem::GetSceneRotation() const
{
  return 0;
}

/////////////////////////////////////////////////
void FloorItem::AttachWall(WallItem *_wallItem)
{
  if (this->floorBoundingRect.isEmpty())
  {
    this->floorBoundingRect = _wallItem->mapToScene(_wallItem->boundingRect());
  }
  else
  {
    this->floorBoundingRect = this->floorBoundingRect.united(
      _wallItem->mapToScene(_wallItem->boundingRect()));
  }
  this->walls.push_back(_wallItem);

  for (unsigned int i = 0; i < _wallItem->GetSegmentCount(); ++i)
  {
    LineSegmentItem *segment = _wallItem->GetSegment(i);
    connect(segment, SIGNAL(WidthChanged(double)), this,
        SLOT(NotifyChange()));
    connect(segment, SIGNAL(DepthChanged(double)), this,
        SLOT(NotifyChange()));
    connect(segment, SIGNAL(PosXChanged(double)), this,
        SLOT(NotifyChange()));
    connect(segment, SIGNAL(PosYChanged(double)), this,
        SLOT(NotifyChange()));
  }
  connect(_wallItem, SIGNAL(ItemDeleted()), this,
      SLOT(WallDeleted()));
  this->Update();
}

/////////////////////////////////////////////////
void FloorItem:: WallDeleted()
{
  EditorItem *wallItem = dynamic_cast<EditorItem *>(QObject::sender());
  if (wallItem)
  {
    this->walls.erase(std::remove(this->walls.begin(), this->walls.end(),
      wallItem), this->walls.end());
  }
  this->dirty = true;
}

/////////////////////////////////////////////////
void FloorItem:: NotifyChange()
{
  this->dirty = true;
}

/////////////////////////////////////////////////
void FloorItem::RecalculateBoundingBox()
{
  if ((this->walls.empty()) || !this->dirty)
    return;

  WallItem *wallItem = this->walls[0];
  this->floorBoundingRect = wallItem->mapToScene(
      wallItem->boundingRect());
  for (unsigned int i = 1; i < this->walls.size(); ++i)
  {
    this->floorBoundingRect = this->floorBoundingRect.united(
        this->walls[i]->mapToScene(this->walls[i]->boundingRect()));
  }
  this->Update();
  this->dirty = false;
}

/////////////////////////////////////////////////
void FloorItem::Update()
{
  QRectF allWallBound = this->floorBoundingRect.boundingRect();
  this->floorWidth = allWallBound.width();
  this->floorDepth = allWallBound.height();

  this->floorPos = QPointF(allWallBound.x()  + allWallBound.width()/2,
      allWallBound.y()+allWallBound.height()/2);

  this->drawingWidth = this->floorWidth;
  this->drawingHeight = this->floorDepth;
  this->setPos(this->floorPos);

  this->FloorChanged();
}


/////////////////////////////////////////////////
void FloorItem::mousePressEvent(QGraphicsSceneMouseEvent *_event)
{
//  if (!this->isSelected())
//    this->scene()->clearSelection();

  _event->setAccepted(false);
}

/////////////////////////////////////////////////
void FloorItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

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
void FloorItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *_event)
{
  _event->ignore();
}

/////////////////////////////////////////////////
void FloorItem::FloorChanged()
{
  emit WidthChanged(this->floorWidth);
  emit DepthChanged(this->floorDepth);
  emit HeightChanged(this->floorHeight);
  emit PositionChanged(this->floorPos.x(), this->floorPos.y(),
      this->levelBaseHeight/* + this->floorElevation*/);
}

/////////////////////////////////////////////////
void FloorItem::SizeChanged()
{
  emit WidthChanged(this->floorWidth);
  emit DepthChanged(this->floorDepth);
}
