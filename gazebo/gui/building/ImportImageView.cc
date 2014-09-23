/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/gui/building/ImportImageDialog.hh"
#include "gazebo/gui/building/GridLines.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/LineSegmentItem.hh"
#include "gazebo/gui/building/MeasureItem.hh"
//#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/ImportImageView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImportImageView::ImportImageView(QWidget *_parent)
  : QGraphicsView(_parent), currentMouseItem(0)
{
  this->setObjectName("importImageView");

  this->drawInProgress = false;

  this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  this->gridLines = NULL;
  this->imageItem = NULL;
  this->imagePixmap = NULL;

  this->viewScale = 1.0;

  this->noImageText = new QGraphicsTextItem;
  noImageText->setPlainText("No image selected");
  noImageText->setDefaultTextColor(Qt::gray);

  this->measureItem = NULL;
}

/////////////////////////////////////////////////
ImportImageView::~ImportImageView()
{
}


/////////////////////////////////////////////////
void ImportImageView::SetImage(const std::string &_filename, int _width,
    int _height)
{
  if (this->imageItem)
    this->scene()->removeItem(this->imageItem);
  if (this->gridLines)
    this->scene()->removeItem(this->gridLines);
  this->scene()->removeItem(this->noImageText);
  if (this->measureItem)
    this->scene()->removeItem(this->measureItem);

  this->imagePixmap = new QPixmap(QString(_filename.c_str()));
  this->imageItem = new QGraphicsPixmapItem(this->imagePixmap->scaled(
      this->scene()->sceneRect().width(),
      this->scene()->sceneRect().height(), Qt::KeepAspectRatio));

  if (this->imageItem)
  {
    this->scene()->addItem(this->imageItem);
  }
  else
  {
    this->scene()->addItem(this->gridLines);
  }
}

/////////////////////////////////////////////////
void ImportImageView::resizeEvent(QResizeEvent *_event)
{
  if (this->scene())
  {
    this->scene()->setSceneRect(0, 0, _event->size().width(),
                                      _event->size().height());

    if (!this->gridLines)
    {
      this->gridLines = new GridLines(_event->size().width(),
          _event->size().height());
      this->scene()->addItem(this->gridLines);
      this->scene()->addItem(this->noImageText);
    }
    else
    {
      this->gridLines->SetSize(_event->size().width(),
            _event->size().height());
    }

    if (this->imageItem)
    {
      this->scene()->removeItem(this->imageItem);
      this->imageItem = new QGraphicsPixmapItem(this->imagePixmap->scaled(
          this->scene()->sceneRect().width(),
          this->scene()->sceneRect().height(), Qt::KeepAspectRatio));
      this->scene()->addItem(this->imageItem);
    }
  }
}

/////////////////////////////////////////////////
void ImportImageView::mouseMoveEvent(QMouseEvent *_event)
{
  if (this->drawInProgress && this->measureItem)
  {
    // snap to 0/90/180 degrees
    LineSegmentItem *segment = this->measureItem->GetSegment(
        this->measureItem->GetSegmentCount()-1);
    QPointF p1 = segment->mapToScene(segment->line().p1());
    QPointF p2 = this->mapToScene(_event->pos());
    QLineF line(p1, p2);
    double angle = line.angle();
    double range = 10;
    if ((angle < range) || (angle > (360 - range)) ||
        ((angle > (180 - range)) && (angle < (180 + range))))
    {
      this->measureItem->SetVertexPosition(this->measureItem->GetVertexCount()-1,
          QPointF(p2.x(), p1.y()));
    }
    else if (((angle > (90 - range)) && (angle < (90 + range))) ||
        ((angle > (270 - range)) && (angle < (270 + range))))
    {
      this->measureItem->SetVertexPosition(this->measureItem->GetVertexCount()-1,
          QPointF(p1.x(), p2.y()));
    }
    else
    {
      this->measureItem->SetVertexPosition(this->measureItem->GetVertexCount()-1, p2);
    }
  }

  if (!drawInProgress)
  {
    QGraphicsView::mouseMoveEvent(_event);
  }
}

/////////////////////////////////////////////////
void ImportImageView::mouseReleaseEvent(QMouseEvent *_event)
{
  if (this->imageItem)
  {
    this->DrawMeasure(_event->pos());
  }

  if (!this->drawInProgress)
  {
    this->currentMouseItem = NULL;
  }

  QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void ImportImageView::DrawMeasure(const QPoint &_pos)
{
  if (!this->drawInProgress)
  {
    if (this->measureItem)
    {
      this->scene()->removeItem(this->measureItem);
    }

    QApplication::setOverrideCursor(QCursor(Qt::CrossCursor));

    QPointF pointStart = mapToScene(_pos);
    QPointF pointEnd = pointStart + QPointF(1, 0);

    this->measureItem = new MeasureItem(pointStart, pointEnd);
    this->scene()->addItem(this->measureItem);
    this->currentMouseItem = this->measureItem;
    this->drawInProgress = true;
  }
  else
  {
    this->measureItem = dynamic_cast<MeasureItem *>(this->currentMouseItem);
    if (this->measureItem)
    {
      LineSegmentItem *segment = this->measureItem->GetSegment(
          this->measureItem->GetSegmentCount()-1);
      this->measureItem->AddPoint(segment->mapToScene(segment->line().p2())
          + QPointF(1, 0));

      this->measureItem->PopEndPoint();
    }

    this->currentMouseItem = NULL;
    this->drawInProgress = false;
    this->releaseKeyboard();
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
  }
}
