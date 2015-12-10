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
FloorItem::FloorItem(): RectItem(*new FloorItemPrivate), BuildingItem()
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  dPtr->editorType = "Floor";

  dPtr->level = 0;
  dPtr->levelBaseHeight = 0;

  dPtr->floorWidth = 100;
  dPtr->floorDepth = 100;
  dPtr->floorHeight = 10;

  dPtr->floorPos = this->scenePos();

  this->setFlag(QGraphicsItem::ItemIsSelectable, false);
  dPtr->dirty = false;

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
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  return QVector3D(dPtr->floorWidth, dPtr->floorDepth, dPtr->floorHeight);
}

/////////////////////////////////////////////////
QVector3D FloorItem::GetScenePosition() const
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  return QVector3D(dPtr->floorPos.x(), dPtr->floorPos.y(),
      dPtr->levelBaseHeight);
}

/////////////////////////////////////////////////
double FloorItem::GetSceneRotation() const
{
  return 0;
}

/////////////////////////////////////////////////
void FloorItem::AttachWallSegment(WallSegmentItem *_wallSegmentItem)
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  dPtr->floorBoundingRect << _wallSegmentItem->boundingRect().topLeft();
  dPtr->floorBoundingRect << _wallSegmentItem->boundingRect().bottomRight();
  dPtr->wallSegments.push_back(_wallSegmentItem);

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
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  EditorItem *wallSegmentItem = dynamic_cast<EditorItem *>(QObject::sender());
  if (wallSegmentItem)
  {
    dPtr->wallSegments.erase(std::remove(dPtr->wallSegments.begin(),
        dPtr->wallSegments.end(), wallSegmentItem), dPtr->wallSegments.end());
  }
  dPtr->dirty = true;
}

/////////////////////////////////////////////////
void FloorItem::NotifyChange()
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  dPtr->dirty = true;
}

/////////////////////////////////////////////////
void FloorItem::RecalculateBoundingBox()
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  if ((dPtr->wallSegments.empty()) || !dPtr->dirty)
    return;

  dPtr->floorBoundingRect.clear();
  for (unsigned int i = 0; i < dPtr->wallSegments.size(); ++i)
  {
    dPtr->floorBoundingRect <<
        dPtr->wallSegments[i]->boundingRect().topLeft();
    dPtr->floorBoundingRect <<
        dPtr->wallSegments[i]->boundingRect().bottomRight();
  }
  this->Update();
  dPtr->dirty = false;
}

/////////////////////////////////////////////////
void FloorItem::Update()
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  QRectF allWallBound = dPtr->floorBoundingRect.boundingRect();
  dPtr->floorWidth = allWallBound.width();
  dPtr->floorDepth = allWallBound.height();

  dPtr->floorPos = QPointF(allWallBound.x()  + allWallBound.width()/2,
      allWallBound.y()+allWallBound.height()/2);

  dPtr->drawingWidth = dPtr->floorWidth;
  dPtr->drawingHeight = dPtr->floorDepth;
  this->setPos(dPtr->floorPos);

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
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPointF topLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF topRight(dPtr->drawingOriginX + dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF bottomLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);
  QPointF bottomRight(dPtr->drawingOriginX  + dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);

  QPen rectPen;
  rectPen.setStyle(Qt::SolidLine);
  rectPen.setColor(dPtr->borderColor);
  _painter->setPen(rectPen);

  _painter->drawLine(topLeft, topRight);
  _painter->drawLine(topRight, bottomRight);
  _painter->drawLine(bottomRight, bottomLeft);
  _painter->drawLine(bottomLeft, topLeft);

  dPtr->floorWidth = dPtr->drawingWidth;
  dPtr->floorDepth = dPtr->drawingHeight;
  dPtr->floorPos = this->scenePos();

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
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  emit WidthChanged(dPtr->floorWidth);
  emit DepthChanged(dPtr->floorDepth);
  emit HeightChanged(dPtr->floorHeight);
  emit PositionChanged(dPtr->floorPos.x(), dPtr->floorPos.y(),
      dPtr->levelBaseHeight/* + dPtr->floorElevation*/);
}

/////////////////////////////////////////////////
void FloorItem::SizeChanged()
{
  auto dPtr = static_cast<FloorItemPrivate *>(this->dataPtr);

  emit WidthChanged(dPtr->floorWidth);
  emit DepthChanged(dPtr->floorDepth);
}
