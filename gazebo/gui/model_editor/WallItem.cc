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

#include "PolylineItem.hh"
#include "WallItem.hh"
#include "LineSegmentItem.hh"
#include "WallInspectorDialog.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallItem::WallItem(QPointF _start, QPointF _end)
    : PolylineItem(_start, _end)
{
}

/////////////////////////////////////////////////
WallItem::~WallItem()
{
}

/////////////////////////////////////////////////
bool WallItem::segmentEventFilter(LineSegmentItem *_segment,
    QGraphicsSceneMouseEvent *_event)
{
  QPointF scenePosition =  _segment->mapToScene(_event->pos());
  switch (_event->type())
  {
    case QEvent::GraphicsSceneMousePress:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMousePress);
      _segment->SetMouseDownX(scenePosition.x());
      _segment->SetMouseDownY(scenePosition.y());
      this->segmentMouseMove = scenePosition;
      break;
    }
    case QEvent::GraphicsSceneMouseRelease:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMouseRelease);
      break;
    }
    case QEvent::GraphicsSceneMouseMove:
    {
      _segment->SetMouseState(QEvent::GraphicsSceneMouseMove);
      break;
    }
    case QEvent::GraphicsSceneMouseDoubleClick:
    {
      WallInspectorDialog dialog;
      if (dialog.exec() == QDialog::Accepted)
      {
      }
      _segment->SetMouseState(QEvent::GraphicsSceneMouseDoubleClick);
      break;
    }
    default:
    {
      break;
    }
  }

  if (_segment->GetMouseState() == QEvent::GraphicsSceneMouseMove)
  {
    QPointF trans = scenePosition - segmentMouseMove;

    this->TranslateVertex(_segment->GetIndex(), trans);
    this->TranslateVertex(_segment->GetIndex() + 1, trans);

    this->segmentMouseMove = scenePosition;

    this->update();
  }
  return true;
}
