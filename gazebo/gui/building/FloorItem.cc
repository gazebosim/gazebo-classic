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

#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/WallSegmentItem.hh"
#include "gazebo/gui/building/FloorItem.hh"
#include "gazebo/gui/building/FloorItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
FloorItem::FloorItem(): RectItem(*new FloorItemPrivate), BuildingItem(),
    dataPtr(std::static_pointer_cast<FloorItemPrivate>(this->rectDPtr))
{
  this->dataPtr->editorType = "Floor";

  this->dataPtr->level = 0;
  this->dataPtr->levelBaseHeight = 0;

  this->dataPtr->floorWidth = 100;
  this->dataPtr->floorDepth = 100;
  this->dataPtr->floorHeight = 10;

  this->dataPtr->floorPos = this->scenePos();

  this->setFlag(QGraphicsItem::ItemIsSelectable, false);
  this->dataPtr->dirty = false;

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
  return QVector3D(this->dataPtr->floorWidth, this->dataPtr->floorDepth, this->dataPtr->floorHeight);
}

/////////////////////////////////////////////////
QVector3D FloorItem::GetScenePosition() const
{
  return QVector3D(this->dataPtr->floorPos.x(), this->dataPtr->floorPos.y(),
      this->dataPtr->levelBaseHeight);
}

/////////////////////////////////////////////////
double FloorItem::GetSceneRotation() const
{
  return 0;
}

/////////////////////////////////////////////////
void FloorItem::AttachWallSegment(WallSegmentItem *_wallSegmentItem)
{
  this->dataPtr->floorBoundingRect << _wallSegmentItem->boundingRect().topLeft();
  this->dataPtr->floorBoundingRect << _wallSegmentItem->boundingRect().bottomRight();
  this->dataPtr->wallSegments.push_back(_wallSegmentItem);

  connect(_wallSegmentItem, SIGNAL(WidthChanged(double)), this,
      SLOT(NotifyChange()));
  connect(_wallSegmentItem, SIGNAL(DepthChanged(double)), this,
      SLOT(NotifyChange()));
  connect(_wallSegmentItem, SIGNAL(PosXChanged(double)), this,
      SLOT(NotifyChange()));
  connect(_wallSegmentItem, SIGNAL(PosYChanged(double)), this,
      SLOT(NotifyChange()));
  connect(_wallSegmentItem, SIGNAL(ItemDeleted()), this,
      SLOT(WallSegmentDeleted()));
  this->Update();
}

/////////////////////////////////////////////////
void FloorItem::WallSegmentDeleted()
{
  EditorItem *wallSegmentItem = dynamic_cast<EditorItem *>(QObject::sender());
  if (wallSegmentItem)
  {
    this->dataPtr->wallSegments.erase(std::remove(this->dataPtr->wallSegments.begin(),
        this->dataPtr->wallSegments.end(), wallSegmentItem), this->dataPtr->wallSegments.end());
  }
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void FloorItem::NotifyChange()
{
  this->dataPtr->dirty = true;
}

/////////////////////////////////////////////////
void FloorItem::RecalculateBoundingBox()
{
  if ((this->dataPtr->wallSegments.empty()) || !this->dataPtr->dirty)
    return;

  this->dataPtr->floorBoundingRect.clear();
  for (unsigned int i = 0; i < this->dataPtr->wallSegments.size(); ++i)
  {
    this->dataPtr->floorBoundingRect <<
        this->dataPtr->wallSegments[i]->boundingRect().topLeft();
    this->dataPtr->floorBoundingRect <<
        this->dataPtr->wallSegments[i]->boundingRect().bottomRight();
  }
  this->Update();
  this->dataPtr->dirty = false;
}

/////////////////////////////////////////////////
void FloorItem::Update()
{
  QRectF allWallBound = this->dataPtr->floorBoundingRect.boundingRect();
  this->dataPtr->floorWidth = allWallBound.width();
  this->dataPtr->floorDepth = allWallBound.height();

  this->dataPtr->floorPos = QPointF(allWallBound.x()  + allWallBound.width()/2,
      allWallBound.y()+allWallBound.height()/2);

  this->dataPtr->drawingWidth = this->dataPtr->floorWidth;
  this->dataPtr->drawingHeight = this->dataPtr->floorDepth;
  this->setPos(this->dataPtr->floorPos);

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

  QPointF topLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF topRight(this->dataPtr->drawingOriginX + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF bottomLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);
  QPointF bottomRight(this->dataPtr->drawingOriginX  + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(this->dataPtr->borderColor);
  _painter->setPen(rectPen);

  _painter->drawLine(topLeft, topRight);
  _painter->drawLine(topRight, bottomRight);
  _painter->drawLine(bottomRight, bottomLeft);
  _painter->drawLine(bottomLeft, topLeft);

  this->dataPtr->floorWidth = this->dataPtr->drawingWidth;
  this->dataPtr->floorDepth = this->dataPtr->drawingHeight;
  this->dataPtr->floorPos = this->scenePos();

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
  emit WidthChanged(this->dataPtr->floorWidth);
  emit DepthChanged(this->dataPtr->floorDepth);
  emit HeightChanged(this->dataPtr->floorHeight);
  emit PositionChanged(this->dataPtr->floorPos.x(), this->dataPtr->floorPos.y(),
      this->dataPtr->levelBaseHeight/* + this->dataPtr->floorElevation*/);
}

/////////////////////////////////////////////////
void FloorItem::SizeChanged()
{
  emit WidthChanged(this->dataPtr->floorWidth);
  emit DepthChanged(this->dataPtr->floorDepth);
}
