/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <cstdlib>
#include "gazebo/gui/model/GraphView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GraphView::GraphView(QWidget *_parent)
  : QGraphicsView(_parent)
{
  this->setObjectName("GraphView");
  this->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);

  this->viewScale = 1.0;
}

/////////////////////////////////////////////////
void GraphView::scrollContentsBy(int _dx, int _dy)
{
  QGraphicsView::scrollContentsBy(_dx, _dy);
}

/////////////////////////////////////////////////
void GraphView::resizeEvent(QResizeEvent */*_event*/)
{
}

/////////////////////////////////////////////////
void GraphView::contextMenuEvent(QContextMenuEvent *_event)
{
  QGraphicsItem *item = this->scene()->itemAt(this->mapToScene(_event->pos()));
  if (item)
  {
    _event->ignore();
    QGraphicsView::contextMenuEvent(_event);
    return;
  }
}

/////////////////////////////////////////////////
void GraphView::wheelEvent(QWheelEvent *_event)
{
  int wheelIncr = 120;
  int sign = (_event->delta() > 0) ? 1 : -1;
  int delta = std::max(std::abs(_event->delta()), wheelIncr) * sign;
  int numSteps = delta / wheelIncr;

  QMatrix mat = matrix();
  QPointF mousePosition = _event->pos();

  mat.translate((width()/2) - mousePosition.x(), (height()/2) -
    mousePosition.y());

  double scaleFactor = 1.15;

  if (numSteps > 0)
  {
    mat.scale(numSteps*scaleFactor, numSteps*scaleFactor);
    this->viewScale *= numSteps*scaleFactor;
  }
  else
  {
    mat.scale(-1/(numSteps*scaleFactor), -1/(numSteps*scaleFactor));
    this->viewScale *= -1/(numSteps*scaleFactor);
  }
  mat.translate(mousePosition.x() - (this->width()/2),
      mousePosition.y() -(this->height()/2));
  this->setMatrix(mat);

  _event->accept();
}

/////////////////////////////////////////////////
void GraphView::mousePressEvent(QMouseEvent *_event)
{
  if (_event->button() != Qt::RightButton)
  {
    QGraphicsItem *mouseItem =
        this->scene()->itemAt(this->mapToScene(_event->pos()));
    if (mouseItem && !mouseItem->isSelected())
    {
      this->scene()->clearSelection();
      mouseItem->setSelected(true);
    }
    QGraphicsView::mousePressEvent(_event);
  }
}

/////////////////////////////////////////////////
void GraphView::mouseReleaseEvent(QMouseEvent *_event)
{
  QGraphicsView::mouseReleaseEvent(_event);
}

/////////////////////////////////////////////////
void GraphView::mouseMoveEvent(QMouseEvent *_event)
{
  QGraphicsView::mouseMoveEvent(_event);
}

/////////////////////////////////////////////////
void GraphView::keyPressEvent(QKeyEvent *_event)
{
  QGraphicsView::keyPressEvent(_event);
}

/////////////////////////////////////////////////
void GraphView::mouseDoubleClickEvent(QMouseEvent *_event)
{
  QGraphicsView::mouseDoubleClickEvent(_event);
}
