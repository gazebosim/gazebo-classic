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

#include "EditorView.hh"
#include "GridLines.hh"
#include "RectItem.hh"
#include "WindowItem.hh"
#include "DoorItem.hh"
#include "PolylineItem.hh"
#include "WallItem.hh"
#include "gui/GuiEvents.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorView::EditorView(QWidget *_parent)
  : QGraphicsView(_parent), currentMouseItem(0)
{
  this->setObjectName("editorView");

  this->drawMode = None;
  this->drawInProgress = false;
  this->mouseMode = Select;

  this->connections.push_back(
  gui::Events::ConnectCreateEditorItem(
    boost::bind(&EditorView::OnCreateEditorItem, this, _1)));
}

/////////////////////////////////////////////////
EditorView::~EditorView()
{
}

/////////////////////////////////////////////////
void EditorView::mousePressEvent(QMouseEvent *_event)
{
  QGraphicsView::mousePressEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::mouseReleaseEvent(QMouseEvent *_event)
{
  /// TODO for testing, delete me later
/*  if (_event->button() == Qt::RightButton)
  {
    if (this->drawMode == Wall)
    {
      qDebug() << "set draw mode to none";
      this->drawMode = None;
    }
    else {
      qDebug() << "set draw mode to wall";
      this->drawMode = Wall;
    }
    return;
  }*/

  switch (drawMode)
  {
    case None:
      break;
    case Wall:
      this->DrawLine(_event->pos());
      break;
    case Window:
      if (drawInProgress)
      {
        this->windowList.push_back(dynamic_cast<WindowItem*>(
            this->currentMouseItem));
        this->drawMode = None;
        this->drawInProgress = false;
      }

      break;
    case Door:
      if (drawInProgress)
      {
        this->doorList.push_back(dynamic_cast<DoorItem*>(
            this->currentMouseItem));
        this->drawMode = None;
        this->drawInProgress = false;
      }
      break;
    default:
      break;
  }

  if (!drawInProgress)
    QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::mouseMoveEvent(QMouseEvent *_event)
{
  QPointF trans = _event->pos() - lastMousePos;
  switch (drawMode)
  {
    case None:
      break;
    case Wall:
    {
      WallItem *wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
      if (this->drawInProgress && wallItem)
      {
        if (wallItem)
          wallItem->TranslateVertex(wallItem->GetCount()-1,trans);
      }
      break;
    }
    case Window:
      this->DrawWindow(_event->pos());
      break;
    case Door:
      this->DrawDoor(_event->pos());
      break;
    default:
      break;
  }

  lastMousePos = _event->pos();
  QGraphicsView::mouseMoveEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (this->drawMode == Wall)
  {
    WallItem* wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
    wallItem->PopEndPoint();
    wallList.push_back(wallItem);
    this->drawMode = None;
    this->drawInProgress = false;
  }
  QGraphicsView::mouseDoubleClickEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::DrawLine(QPoint _pos)
{
  if (!drawInProgress)
  {
    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart;

    WallItem* lineItem = new WallItem(pointStart, pointEnd);
    this->scene()->addItem(lineItem);
    this->currentMouseItem = lineItem;
    this->drawInProgress = true;

    lastLineCornerPos = _pos;
    lastMousePos = _pos;
  }
  else
  {
    WallItem* wallItem = dynamic_cast<WallItem*>(this->currentMouseItem);
    if (wallItem)
      wallItem->AddPoint(this->mapToScene(_pos));
  }
}

/////////////////////////////////////////////////
void EditorView::DrawWindow(QPoint _pos)
{
  WindowItem *windowItem = NULL;
  if (!drawInProgress)
  {
    windowItem = new WindowItem();
    this->scene()->addItem(windowItem);
    this->currentMouseItem = windowItem;
    this->drawInProgress = true;
  }
  windowItem = dynamic_cast<WindowItem*>(this->currentMouseItem);
  if (windowItem)
  {
    QPointF scenePos = this->mapToScene(_pos);
    windowItem->setPos(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::DrawDoor(QPoint _pos)
{
  DoorItem *doorItem = NULL;
  if (!drawInProgress)
  {
    doorItem = new DoorItem();
    this->scene()->addItem(doorItem);
    this->currentMouseItem = doorItem;
    this->drawInProgress = true;
  }
  doorItem = dynamic_cast<DoorItem*>(currentMouseItem);
  if (doorItem)
  {
    QPointF scenePos = mapToScene(_pos);
    doorItem->setPos(scenePos.x(), scenePos.y());
  }
}

/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{
  if (_type == "Wall")
    this->drawMode = Wall;
  else if (_type == "Window")
    this->drawMode = Window;
  else if (_type == "Door")
    this->drawMode = Door;
  else if (_type == "Stairs")
    this->drawMode = Stairs;

  if (this->drawInProgress && this->currentMouseItem)
  {
    this->scene()->removeItem(this->currentMouseItem);
    this->currentMouseItem = NULL;
    this->drawInProgress = false;
  }

  qDebug() << " on create editor item" << _type.c_str();
}
