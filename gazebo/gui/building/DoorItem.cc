/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/DoorItem.hh"
#include "gazebo/gui/building/DoorItemPrivate.hh"
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/WindowDoorInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DoorItem::DoorItem() : RectItem(), dataPtr(new DoorItemPrivate)
{
  this->editorType = "Door";
  this->itemScale = BuildingMaker::conversionScale;

  this->level = 0;
  this->levelBaseHeight = 0;

  this->dataPtr->doorDepth = 17;
  this->dataPtr->doorHeight = 200;
  this->dataPtr->doorWidth = 90;
  this->dataPtr->doorElevation = 0;

  this->width = this->dataPtr->doorWidth;
  this->height = this->dataPtr->doorDepth;
  this->drawingWidth = this->width;
  this->drawingHeight = this->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  this->dataPtr->doorPos = Conversions::Convert(this->scenePos());

  this->zValueIdle = 3;
  this->setZValue(this->zValueIdle);

  this->dataPtr->inspector = new WindowDoorInspectorDialog(
    WindowDoorInspectorDialog::DOOR);
  this->dataPtr->inspector->setModal(false);
  connect(this->dataPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->openInspectorAct =
      new QAction(tr("&Open Door Inspector"), this);
  this->openInspectorAct->setStatusTip(tr("Open Door Inspector"));
  connect(this->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->deleteItemAct = new QAction(tr("&Delete"), this);
  this->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
DoorItem::~DoorItem()
{
  delete this->dataPtr->inspector;
}

/////////////////////////////////////////////////
ignition::math::Vector3d DoorItem::Size() const
{
  return ignition::math::Vector3d(this->dataPtr->doorWidth,
      this->dataPtr->doorDepth, this->dataPtr->doorHeight);
}

/////////////////////////////////////////////////
ignition::math::Vector3d DoorItem::ScenePosition() const
{
  return ignition::math::Vector3d(this->scenePos().x(), this->scenePos().y(),
      this->dataPtr->doorElevation);
}

/////////////////////////////////////////////////
double DoorItem::SceneRotation() const
{
  return this->rotationAngle;
}

/////////////////////////////////////////////////
void DoorItem::paint(QPainter *_painter,
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

  QPen doorPen;
  doorPen.setStyle(Qt::SolidLine);
  doorPen.setColor(Conversions::Convert(this->borderColor));
  _painter->setPen(doorPen);

  _painter->drawLine(topLeft, bottomLeft + QPointF(0, this->drawingWidth));
  QRect arcRect(topLeft.x() - this->drawingWidth,
      topLeft.y() + this->drawingHeight - this->drawingWidth,
      this->drawingWidth*2, this->drawingWidth*2);
  _painter->drawArc(arcRect, 0, -90 * 16);

  doorPen.setWidth(this->dataPtr->doorDepth);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->dataPtr->doorDepth/2.0,
      this->dataPtr->doorDepth/2.0), topRight -
      QPointF(this->dataPtr->doorDepth/2.0, - this->dataPtr->doorDepth/2.0));

  double borderSize = 1.0;
  doorPen.setColor(Qt::white);
  doorPen.setWidth(this->dataPtr->doorDepth - borderSize*2);
  _painter->setPen(doorPen);
  _painter->drawLine(topLeft + QPointF(this->dataPtr->doorDepth/2.0,
      this->dataPtr->doorDepth/2.0), topRight -
      QPointF(this->dataPtr->doorDepth/2.0, - this->dataPtr->doorDepth/2.0));

  this->dataPtr->doorWidth = this->drawingWidth;
  this->dataPtr->doorDepth = this->drawingHeight;
  this->dataPtr->doorPos = Conversions::Convert(this->scenePos());
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
  WindowDoorInspectorDialog *dialog =
     qobject_cast<WindowDoorInspectorDialog *>(QObject::sender());

  QPointF itemPos = Conversions::Convert(this->dataPtr->doorPos) *
      this->itemScale;
  itemPos.setY(-itemPos.y());
  this->SetSize(ignition::math::Vector2i(dialog->Width() / this->itemScale,
      (dialog->Depth() / this->itemScale)));
  this->dataPtr->doorWidth = dialog->Width() / this->itemScale;
  this->dataPtr->doorHeight = dialog->Height() / this->itemScale;
  this->dataPtr->doorDepth = dialog->Depth() / this->itemScale;
  this->dataPtr->doorElevation = dialog->Elevation() / this->itemScale;
  if ((fabs(dialog->Position().X() - itemPos.x()) >= 0.01)
      || (fabs(dialog->Position().Y() - itemPos.y()) >= 0.01))
  {
    itemPos = Conversions::Convert(dialog->Position()) / this->itemScale;
    itemPos.setY(-itemPos.y());
    this->dataPtr->doorPos = Conversions::Convert(itemPos);
    this->setPos(itemPos);
//    this->setParentItem(NULL);
  }
  this->DoorChanged();
}

/////////////////////////////////////////////////
void DoorItem::OnOpenInspector()
{
  this->dataPtr->inspector->SetName(this->Name());
  this->dataPtr->inspector->SetWidth(
      this->dataPtr->doorWidth * this->itemScale);
  this->dataPtr->inspector->SetDepth(
      this->dataPtr->doorDepth * this->itemScale);
  this->dataPtr->inspector->SetHeight(
      this->dataPtr->doorHeight * this->itemScale);
  this->dataPtr->inspector->SetElevation(
      this->dataPtr->doorElevation * this->itemScale);
  auto itemPos = this->dataPtr->doorPos * this->itemScale;
  itemPos.Y(-itemPos.Y());
  this->dataPtr->inspector->SetPosition(itemPos);
  this->dataPtr->inspector->move(QCursor::pos());
  this->dataPtr->inspector->show();
}

/////////////////////////////////////////////////
void DoorItem::DoorChanged()
{
  emit WidthChanged(this->dataPtr->doorWidth);
  emit DepthChanged(this->dataPtr->doorDepth);
  emit HeightChanged(this->dataPtr->doorHeight);
  emit PositionChanged(this->dataPtr->doorPos.X(), this->dataPtr->doorPos.Y(),
      this->levelBaseHeight + this->dataPtr->doorElevation);
}

/////////////////////////////////////////////////
void DoorItem::SizeChanged()
{
  emit WidthChanged(this->dataPtr->doorWidth);
  emit DepthChanged(this->dataPtr->doorDepth);
  this->UpdateMeasures();
}

/////////////////////////////////////////////////
void DoorItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
