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

#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/BuildingItem.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/RectItem.hh"
#include "gazebo/gui/building/WindowDoorInspectorDialog.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/DoorItem.hh"
#include "gazebo/gui/building/DoorItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DoorItem::DoorItem(): RectItem(*new DoorItemPrivate), BuildingItem()
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  dPtr->editorType = "Door";
  dPtr->itemScale = BuildingMaker::conversionScale;

  dPtr->level = 0;
  dPtr->levelBaseHeight = 0;

  dPtr->doorDepth = 17;
  dPtr->doorHeight = 200;
  dPtr->doorWidth = 90;
  dPtr->doorElevation = 0;

  dPtr->width = dPtr->doorWidth;
  dPtr->height = dPtr->doorDepth;
  dPtr->drawingWidth = dPtr->width;
  dPtr->drawingHeight = dPtr->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  dPtr->doorPos = this->scenePos();

  dPtr->zValueIdle = 3;
  this->setZValue(dPtr->zValueIdle);

  dPtr->inspector = new WindowDoorInspectorDialog(
    WindowDoorInspectorDialog::DOOR);
  dPtr->inspector->setModal(false);
  connect(dPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  dPtr->openInspectorAct = new QAction(tr("&Open Door Inspector"), this);
  dPtr->openInspectorAct->setStatusTip(tr("Open Door Inspector"));
  connect(dPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  dPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  dPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(dPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
DoorItem::~DoorItem()
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  delete dPtr->inspector;
}

/////////////////////////////////////////////////
QVector3D DoorItem::GetSize() const
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  return QVector3D(dPtr->doorWidth, dPtr->doorDepth, dPtr->doorHeight);
}

/////////////////////////////////////////////////
QVector3D DoorItem::GetScenePosition() const
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      dPtr->doorElevation);
}

/////////////////////////////////////////////////
double DoorItem::GetSceneRotation() const
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  return dPtr->rotationAngle;
}

/////////////////////////////////////////////////
void DoorItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

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

  QPen doorPen;
  doorPen.setStyle(Qt::SolidLine);
  doorPen.setColor(dPtr->borderColor);
  _painter->setPen(doorPen);

  _painter->drawLine(topLeft, bottomLeft + QPointF(0, dPtr->drawingWidth));
  QRect arcRect(topLeft.x() - dPtr->drawingWidth,
      topLeft.y() + dPtr->drawingHeight - dPtr->drawingWidth,
      dPtr->drawingWidth*2, dPtr->drawingWidth*2);
  _painter->drawArc(arcRect, 0, -90 * 16);

  doorPen.setWidth(dPtr->doorDepth);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(dPtr->doorDepth/2.0,
      dPtr->doorDepth/2.0), topRight - QPointF(dPtr->doorDepth/2.0,
      -dPtr->doorDepth/2.0));

  double borderSize = 1.0;
  doorPen.setColor(Qt::white);
  doorPen.setWidth(dPtr->doorDepth - borderSize*2);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(dPtr->doorDepth/2.0,
      dPtr->doorDepth/2.0), topRight - QPointF(dPtr->doorDepth/2.0,
      -dPtr->doorDepth/2.0));

  dPtr->doorWidth = dPtr->drawingWidth;
  dPtr->doorDepth = dPtr->drawingHeight;
  dPtr->doorPos = this->scenePos();
}

/////////////////////////////////////////////////
void DoorItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  this->OnOpenInspector();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void DoorItem::OnApply()
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  WindowDoorInspectorDialog *dialog =
     qobject_cast<WindowDoorInspectorDialog *>(QObject::sender());

  QPointF itemPos = dPtr->doorPos * dPtr->itemScale;
  itemPos.setY(-itemPos.y());
  this->SetSize(QSize(dialog->GetWidth() / dPtr->itemScale,
      (dialog->GetDepth() / dPtr->itemScale)));
  dPtr->doorWidth = dialog->GetWidth() / dPtr->itemScale;
  dPtr->doorHeight = dialog->GetHeight() / dPtr->itemScale;
  dPtr->doorDepth = dialog->GetDepth() / dPtr->itemScale;
  dPtr->doorElevation = dialog->GetElevation() / dPtr->itemScale;
  if ((fabs(dialog->GetPosition().x() - itemPos.x()) >= 0.01)
      || (fabs(dialog->GetPosition().y() - itemPos.y()) >= 0.01))
  {
    itemPos = dialog->GetPosition() / dPtr->itemScale;
    itemPos.setY(-itemPos.y());
    dPtr->doorPos = itemPos;
    this->setPos(dPtr->doorPos);
//    this->setParentItem(NULL);
  }
  this->DoorChanged();
}

/////////////////////////////////////////////////
void DoorItem::OnOpenInspector()
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  dPtr->inspector->SetName(this->GetName());
  dPtr->inspector->SetWidth(dPtr->doorWidth * dPtr->itemScale);
  dPtr->inspector->SetDepth(dPtr->doorDepth * dPtr->itemScale);
  dPtr->inspector->SetHeight(dPtr->doorHeight * dPtr->itemScale);
  dPtr->inspector->SetElevation(dPtr->doorElevation * dPtr->itemScale);
  QPointF itemPos = dPtr->doorPos * dPtr->itemScale;
  itemPos.setY(-itemPos.y());
  dPtr->inspector->SetPosition(itemPos);
  dPtr->inspector->move(QCursor::pos());
  dPtr->inspector->show();
}

/////////////////////////////////////////////////
void DoorItem::DoorChanged()
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  emit WidthChanged(dPtr->doorWidth);
  emit DepthChanged(dPtr->doorDepth);
  emit HeightChanged(dPtr->doorHeight);
  emit PositionChanged(dPtr->doorPos.x(), dPtr->doorPos.y(),
      dPtr->levelBaseHeight + dPtr->doorElevation);
}

/////////////////////////////////////////////////
void DoorItem::SizeChanged()
{
  auto dPtr = static_cast<DoorItemPrivate *>(this->dataPtr);

  emit WidthChanged(dPtr->doorWidth);
  emit DepthChanged(dPtr->doorDepth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void DoorItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
