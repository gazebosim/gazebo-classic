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
  : QGraphicsView(_parent)
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
      DrawLine(_event->pos());
      break;
    case Window:
      break;
    case Door:
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
      if (drawInProgress && !lineList.empty())
      {
        lineList.back()->TranslateVertex(lineList.back()->GetCount() - 1,
            trans);
      }
      break;
    case Window:
      DrawWindow(_event->pos());
      break;
    case Door:
      DrawDoor(_event->pos());
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
  drawMode = None;
  drawInProgress = false;

  QGraphicsView::mouseDoubleClickEvent(_event);
}

/////////////////////////////////////////////////
void EditorView::DrawLine(QPoint _pos)
{
  if (!drawInProgress)
  {
    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart;

    WallItem* line = new WallItem(pointStart, pointEnd);
    scene()->addItem(line);
    lineList.push_back(line);
    drawInProgress = true;

    lastLineCornerPos = _pos;
    lastMousePos = _pos;
  }
  else
  {
    lineList.back()->AddPoint(this->mapToScene(_pos));
  }
}

/////////////////////////////////////////////////
void EditorView::DrawWindow(QPoint _pos)
{
  if (!drawInProgress)
  {
    WindowItem *windowItem = new WindowItem();
    scene()->addItem(windowItem);
    windowList.push_back(windowItem);
    drawInProgress = true;
  }
  if (!windowList.empty())
  {
    QPointF scenePos = mapToScene(_pos);
    windowList.back()->setPos(scenePos.x(), scenePos.y());
//qDebug() << " setting window to scenePos";
  }
}

/////////////////////////////////////////////////
void EditorView::DrawDoor(QPoint _pos)
{
  if (!drawInProgress)
  {
    DoorItem *doorItem = new DoorItem();
    scene()->addItem(doorItem);
    doorList.push_back(doorItem);
    drawInProgress = true;
  }
  if (!doorList.empty())
  {
    QPointF scenePos = mapToScene(_pos);
    doorList.back()->setPos(scenePos.x(), scenePos.y());
  }
}


/////////////////////////////////////////////////
void EditorView::OnCreateEditorItem(const std::string &_type)
{
  if (_type == "Wall")
    drawMode = Wall;
  else if (_type == "Window")
    drawMode = Window;
  else if (_type == "Door")
    drawMode = Door;
  else if (_type == "Stairs")
    drawMode = Stairs;

  qDebug() << " on create editor item" << _type.c_str();
}
