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
#include "gazebo/gui/building/WindowItem.hh"
#include "gazebo/gui/building/WindowItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WindowItem::WindowItem(): RectItem(*new WindowItemPrivate), BuildingItem(),
    dataPtr(std::static_pointer_cast<WindowItemPrivate>(this->rectDPtr))
{
  this->dataPtr->editorType = "Window";
  this->dataPtr->itemScale = BuildingMaker::conversionScale;

  this->dataPtr->level = 0;
  this->dataPtr->levelBaseHeight = 0;

  this->dataPtr->windowDepth = 17;
  this->dataPtr->windowHeight = 80;
  this->dataPtr->windowWidth = 80;
  this->dataPtr->windowSideBar = 10;
  this->dataPtr->windowPos = this->scenePos();
  this->dataPtr->windowElevation = 50;

  this->dataPtr->width = this->dataPtr->windowWidth;
  this->dataPtr->height = this->dataPtr->windowDepth;
  this->dataPtr->drawingWidth = this->dataPtr->width;
  this->dataPtr->drawingHeight = this->dataPtr->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  this->dataPtr->zValueIdle = 3;
  this->setZValue(this->dataPtr->zValueIdle);

  this->dataPtr->inspector = new WindowDoorInspectorDialog(
      WindowDoorInspectorDialog::WINDOW);
  this->dataPtr->inspector->setModal(false);
  connect(this->dataPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  this->dataPtr->openInspectorAct = new QAction(tr("&Open Window Inspector"), this);
  this->dataPtr->openInspectorAct->setStatusTip(tr("Open Window Inspector"));
  connect(this->dataPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  this->dataPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  this->dataPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(this->dataPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
WindowItem::~WindowItem()
{
  delete this->dataPtr->inspector;
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetSize() const
{
  return QVector3D(this->dataPtr->windowWidth, this->dataPtr->windowDepth, this->dataPtr->windowHeight);
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetScenePosition() const
{
  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      this->dataPtr->windowElevation);
}

/////////////////////////////////////////////////
double WindowItem::GetSceneRotation() const
{
  return this->dataPtr->rotationAngle;
}

/////////////////////////////////////////////////
void WindowItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  QPointF topLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF topRight(this->dataPtr->drawingOriginX + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY - this->dataPtr->drawingHeight/2);
  QPointF bottomLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);
  QPointF bottomRight(this->dataPtr->drawingOriginX  + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY + this->dataPtr->drawingHeight/2);

  QPointF midLeft(this->dataPtr->drawingOriginX - this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY);
  QPointF midRight(this->dataPtr->drawingOriginX + this->dataPtr->drawingWidth/2,
      this->dataPtr->drawingOriginY);

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen windowPen;
  windowPen.setStyle(Qt::SolidLine);
  windowPen.setColor(this->dataPtr->borderColor);
  _painter->setPen(windowPen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);

  windowPen.setWidth(this->dataPtr->windowDepth/2.0);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->dataPtr->windowDepth/4, 0),
      midRight - QPointF(this->dataPtr->windowDepth/4, 0));

  double borderSize = 1.0;
  windowPen.setColor(Qt::white);
  windowPen.setWidth(this->dataPtr->windowDepth/2.0 - borderSize*2);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(this->dataPtr->windowDepth/4.0, 0),
      midRight - QPointF(this->dataPtr->windowDepth/4.0 - 0.5, 0) );

  this->dataPtr->windowWidth = this->dataPtr->drawingWidth;
  this->dataPtr->windowDepth = this->dataPtr->drawingHeight;
  this->dataPtr->windowPos = this->scenePos();
  _painter->restore();

  //  QGraphicsPolygonItem::paint(_painter, _option, _widget);
}

/////////////////////////////////////////////////
void WindowItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *_event)
{
  this->OnOpenInspector();
  _event->setAccepted(true);
}

/////////////////////////////////////////////////
void WindowItem::OnApply()
{
  WindowDoorInspectorDialog *dialog =
     qobject_cast<WindowDoorInspectorDialog *>(QObject::sender());

  QPointF itemPos = this->dataPtr->windowPos * this->dataPtr->itemScale;
  itemPos.setY(-itemPos.y());
  this->SetSize(QSize(dialog->GetWidth() / this->dataPtr->itemScale,
      dialog->GetDepth() / this->dataPtr->itemScale));
  this->dataPtr->windowWidth = dialog->GetWidth() / this->dataPtr->itemScale;
  this->dataPtr->windowHeight = dialog->GetHeight() / this->dataPtr->itemScale;
  this->dataPtr->windowDepth = dialog->GetDepth() / this->dataPtr->itemScale;
  this->dataPtr->windowElevation = dialog->GetElevation() / this->dataPtr->itemScale;
  if ((fabs(dialog->GetPosition().x() - itemPos.x()) >= 0.01)
      || (fabs(dialog->GetPosition().y() - itemPos.y()) >= 0.01))
  {
    itemPos = dialog->GetPosition() / this->dataPtr->itemScale;
    itemPos.setY(-itemPos.y());
    this->dataPtr->windowPos = itemPos;
    this->setPos(this->dataPtr->windowPos);
    // this->dataPtr->setParentItem(NULL);
  }
  this->WindowChanged();
}

/////////////////////////////////////////////////
void WindowItem::WindowChanged()
{
  emit WidthChanged(this->dataPtr->windowWidth);
  emit DepthChanged(this->dataPtr->windowDepth);
  emit HeightChanged(this->dataPtr->windowHeight);
  emit PositionChanged(this->dataPtr->windowPos.x(), this->dataPtr->windowPos.y(),
      this->dataPtr->levelBaseHeight + this->dataPtr->windowElevation);
}

/*
/////////////////////////////////////////////////
void WindowItem::contextMenuEvent(QGraphicsSceneContextMenuEvent *_event)
{
  QMenu menu;
  menu.addAction(this->openInspectorAct);
  menu.exec(_event->screenPos());
  _event->accept();
}
*/

/////////////////////////////////////////////////
void WindowItem::OnOpenInspector()
{
  this->dataPtr->inspector->SetName(this->GetName());
  this->dataPtr->inspector->SetWidth(this->dataPtr->windowWidth * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetHeight(this->dataPtr->windowHeight * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetElevation(this->dataPtr->windowElevation * this->dataPtr->itemScale);
  this->dataPtr->inspector->SetDepth(this->dataPtr->windowDepth * this->dataPtr->itemScale);

  QPointF itemPos = this->dataPtr->windowPos * this->dataPtr->itemScale;
  itemPos.setY(-itemPos.y());
  this->dataPtr->inspector->SetPosition(itemPos);
  this->dataPtr->inspector->move(QCursor::pos());
  this->dataPtr->inspector->show();
}

/////////////////////////////////////////////////
void WindowItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
