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

#include "gui/model_editor/BuildingItem.hh"
#include "gui/model_editor/PolylineItem.hh"
#include "gui/model_editor/WallItem.hh"
#include "gui/model_editor/LineSegmentItem.hh"
#include "gui/model_editor/WallInspectorDialog.hh"
#include "gui/model_editor/BuildingMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
WallItem::WallItem(QPointF _start, QPointF _end)
    : PolylineItem(_start, _end), BuildingItem()
{
  this->editorType = "Wall";
  this->scale = BuildingMaker::conversionScale;

  this->level = 0;

  this->wallThickness = 20;
  this->wallHeight = 0;

  this->SetThickness(this->wallThickness);
}

/////////////////////////////////////////////////
WallItem::~WallItem()
{
}

/////////////////////////////////////////////////
double WallItem::GetHeight()
{
  return this->wallHeight;
}

/////////////////////////////////////////////////
void WallItem::SetHeight(double _height)
{
  this->wallHeight = _height;
}

/////////////////////////////////////////////////
WallItem *WallItem::Clone()
{
  WallItem *wallItem = new WallItem(this->scenePos(), this->scenePos());

  LineSegmentItem *segment = this->segments[0];
  wallItem->SetVertexPosition(1, this->mapToScene(segment->line().p2()));


  // TODO: Code below just simiulates the way wall are created through user
  // interactions. There should be a better way of doing this
  for (unsigned int i = 1; i < this->segments.size(); ++i)
  {
    segment = this->segments[i];
    wallItem->AddPoint(this->mapToScene(segment->line().p1()));
    wallItem->SetVertexPosition(wallItem->GetVertexCount()-1,
        this->mapToScene(segment->line().p2()));  }
  wallItem->AddPoint(this->mapToScene(
      this->segments[wallItem->GetSegmentCount()-1]->line().p2()));
  wallItem->PopEndPoint();

  wallItem->SetLevel(this->level);
  wallItem->SetHeight(this->wallHeight);
  wallItem->SetThickness(this->wallThickness);

  return wallItem;
}

/////////////////////////////////////////////////
bool WallItem::segmentEventFilter(LineSegmentItem *_segment,
    QEvent *_event)
{

  QGraphicsSceneMouseEvent *mouseEvent =
    dynamic_cast<QGraphicsSceneMouseEvent*>(_event);

  if (!mouseEvent)
    return false;

  QPointF scenePosition =  mouseEvent->scenePos();
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
      QLineF line = _segment->line();
      double segmentLength = line.length();
      QPointF segmentStartPoint = this->mapToScene(line.p1());
      QPointF segmentEndPoint = this->mapToScene(line.p2());

      WallInspectorDialog dialog;
      dialog.SetThickness(this->wallThickness * this->scale);
      dialog.SetHeight(this->wallHeight * this->scale);
      dialog.SetLength(segmentLength * this->scale);
      QPointF startPos = segmentStartPoint * this->scale;
      startPos.setY(-startPos.y());
      dialog.SetStartPosition(startPos);
      QPointF endPos = segmentEndPoint * this->scale;
      endPos.setY(-endPos.y());
      dialog.SetEndPosition(endPos);
      if (dialog.exec() == QDialog::Accepted)
      {
        this->wallThickness = dialog.GetThickness() / this->scale;
        this->SetThickness(this->wallThickness);
        this->wallHeight = dialog.GetHeight() / this->scale;
        this->WallChanged();

        double newLength = dialog.GetLength() / this->scale;
        // The if statement below limits the change to either the length of
        // the wall segment or its start/end pos.
        // Comparison between doubles up to 1 decimal place
        if (fabs(newLength - (segmentLength * this->scale)) < 0.1)
        {
          line.setLength(newLength);
          this->SetVertexPosition(_segment->GetIndex()+1,
              this->mapToScene(line.p2()));
        }
        else
        {
          QPointF newStartPoint = dialog.GetStartPosition() / this->scale;
          newStartPoint.setY(-newStartPoint.y());
          QPointF newEndPoint = dialog.GetEndPosition() / this->scale;
          newEndPoint.setY(-newEndPoint.y());

          this->SetVertexPosition(_segment->GetIndex(),
            newStartPoint);
          this->SetVertexPosition(_segment->GetIndex() + 1,
            newEndPoint);
        }
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

/////////////////////////////////////////////////
void WallItem::WallChanged()
{
  emit depthChanged(this->wallThickness);
  emit heightChanged(this->wallHeight);
}

/////////////////////////////////////////////////
void WallItem::Update()
{
  this->WallChanged();
  PolylineItem::Update();
}
