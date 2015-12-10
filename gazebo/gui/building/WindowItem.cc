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
WindowItem::WindowItem(): RectItem(*new WindowItemPrivate), BuildingItem()
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  dPtr->editorType = "Window";
  dPtr->itemScale = BuildingMaker::conversionScale;

  dPtr->level = 0;
  dPtr->levelBaseHeight = 0;

  dPtr->windowDepth = 17;
  dPtr->windowHeight = 80;
  dPtr->windowWidth = 80;
  dPtr->windowSideBar = 10;
  dPtr->windowPos = this->scenePos();
  dPtr->windowElevation = 50;

  dPtr->width = dPtr->windowWidth;
  dPtr->height = dPtr->windowDepth;
  dPtr->drawingWidth = dPtr->width;
  dPtr->drawingHeight = dPtr->height;

  this->UpdateCornerPositions();
  this->UpdateMeasures();

  dPtr->zValueIdle = 3;
  this->setZValue(dPtr->zValueIdle);

  dPtr->inspector = new WindowDoorInspectorDialog(
      WindowDoorInspectorDialog::WINDOW);
  dPtr->inspector->setModal(false);
  connect(dPtr->inspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  dPtr->openInspectorAct = new QAction(tr("&Open Window Inspector"), this);
  dPtr->openInspectorAct->setStatusTip(tr("Open Window Inspector"));
  connect(dPtr->openInspectorAct, SIGNAL(triggered()),
    this, SLOT(OnOpenInspector()));
  dPtr->deleteItemAct = new QAction(tr("&Delete"), this);
  dPtr->deleteItemAct->setStatusTip(tr("Delete"));
  connect(dPtr->deleteItemAct, SIGNAL(triggered()),
    this, SLOT(OnDeleteItem()));

  this->SetResizeFlag(ITEM_WIDTH);
}

/////////////////////////////////////////////////
WindowItem::~WindowItem()
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  delete dPtr->inspector;
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetSize() const
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  return QVector3D(dPtr->windowWidth, dPtr->windowDepth, dPtr->windowHeight);
}

/////////////////////////////////////////////////
QVector3D WindowItem::GetScenePosition() const
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  return QVector3D(this->scenePos().x(), this->scenePos().y(),
      dPtr->windowElevation);
}

/////////////////////////////////////////////////
double WindowItem::GetSceneRotation() const
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  return dPtr->rotationAngle;
}

/////////////////////////////////////////////////
void WindowItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem * /*_option*/, QWidget * /*_widget*/)
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  QPointF topLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF topRight(dPtr->drawingOriginX + dPtr->drawingWidth/2,
      dPtr->drawingOriginY - dPtr->drawingHeight/2);
  QPointF bottomLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);
  QPointF bottomRight(dPtr->drawingOriginX  + dPtr->drawingWidth/2,
      dPtr->drawingOriginY + dPtr->drawingHeight/2);

  QPointF midLeft(dPtr->drawingOriginX - dPtr->drawingWidth/2,
      dPtr->drawingOriginY);
  QPointF midRight(dPtr->drawingOriginX + dPtr->drawingWidth/2,
      dPtr->drawingOriginY);

  _painter->save();

  if (this->isSelected())
    this->DrawBoundingBox(_painter);
  this->ShowHandles(this->isSelected());

  QPen windowPen;
  windowPen.setStyle(Qt::SolidLine);
  windowPen.setColor(dPtr->borderColor);
  _painter->setPen(windowPen);

  _painter->drawLine(topLeft, bottomLeft);
  _painter->drawLine(topRight, bottomRight);

  windowPen.setWidth(dPtr->windowDepth/2.0);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(dPtr->windowDepth/4, 0),
      midRight - QPointF(dPtr->windowDepth/4, 0));

  double borderSize = 1.0;
  windowPen.setColor(Qt::white);
  windowPen.setWidth(dPtr->windowDepth/2.0 - borderSize*2);
  _painter->setPen(windowPen);
  _painter->drawLine(midLeft + QPointF(dPtr->windowDepth/4.0, 0),
      midRight - QPointF(dPtr->windowDepth/4.0 - 0.5, 0) );

  dPtr->windowWidth = dPtr->drawingWidth;
  dPtr->windowDepth = dPtr->drawingHeight;
  dPtr->windowPos = this->scenePos();
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
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  WindowDoorInspectorDialog *dialog =
     qobject_cast<WindowDoorInspectorDialog *>(QObject::sender());

  QPointF itemPos = dPtr->windowPos * dPtr->itemScale;
  itemPos.setY(-itemPos.y());
  this->SetSize(QSize(dialog->GetWidth() / dPtr->itemScale,
      dialog->GetDepth() / dPtr->itemScale));
  dPtr->windowWidth = dialog->GetWidth() / dPtr->itemScale;
  dPtr->windowHeight = dialog->GetHeight() / dPtr->itemScale;
  dPtr->windowDepth = dialog->GetDepth() / dPtr->itemScale;
  dPtr->windowElevation = dialog->GetElevation() / dPtr->itemScale;
  if ((fabs(dialog->GetPosition().x() - itemPos.x()) >= 0.01)
      || (fabs(dialog->GetPosition().y() - itemPos.y()) >= 0.01))
  {
    itemPos = dialog->GetPosition() / dPtr->itemScale;
    itemPos.setY(-itemPos.y());
    dPtr->windowPos = itemPos;
    this->setPos(dPtr->windowPos);
    // dPtr->setParentItem(NULL);
  }
  this->WindowChanged();
}

/////////////////////////////////////////////////
void WindowItem::WindowChanged()
{
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  emit WidthChanged(dPtr->windowWidth);
  emit DepthChanged(dPtr->windowDepth);
  emit HeightChanged(dPtr->windowHeight);
  emit PositionChanged(dPtr->windowPos.x(), dPtr->windowPos.y(),
      dPtr->levelBaseHeight + dPtr->windowElevation);
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
  auto dPtr = static_cast<WindowItemPrivate *>(this->dataPtr);

  dPtr->inspector->SetName(this->GetName());
  dPtr->inspector->SetWidth(dPtr->windowWidth * dPtr->itemScale);
  dPtr->inspector->SetHeight(dPtr->windowHeight * dPtr->itemScale);
  dPtr->inspector->SetElevation(dPtr->windowElevation * dPtr->itemScale);
  dPtr->inspector->SetDepth(dPtr->windowDepth * dPtr->itemScale);

  QPointF itemPos = dPtr->windowPos * dPtr->itemScale;
  itemPos.setY(-itemPos.y());
  dPtr->inspector->SetPosition(itemPos);
  dPtr->inspector->move(QCursor::pos());
  dPtr->inspector->show();
}

/////////////////////////////////////////////////
void WindowItem::OnDeleteItem()
{
  dynamic_cast<EditorView *>(this->scene()->views()[0])->DeleteItem(this);
}
