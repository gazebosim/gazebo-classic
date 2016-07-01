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

#include <sstream>
#include <string>
#include <vector>

#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/ScaleWidget.hh"

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief Private data for the ScaleWidget class
    class ScaleWidgetPrivate
    {
      /// \brief Text displaying the scale.
      public: std::string scaleText;

      /// \brief A list of gui editor events connected to this widget.
      public: std::vector<event::ConnectionPtr> connections;
    };
  }
}

using namespace gazebo;
using namespace gui;

//////////////////////////////////////////////////
ScaleWidget::ScaleWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new ScaleWidgetPrivate)
{
  this->setObjectName("scaleWidget");
  this->dataPtr->scaleText = "1.00 m";

  this->setAttribute(Qt::WA_TransparentForMouseEvents);
  this->dataPtr->connections.push_back(
      gui::editor::Events::ConnectChangeBuildingEditorZoom(
      std::bind(&ScaleWidget::OnChangeZoom, this, std::placeholders::_1)));
}

//////////////////////////////////////////////////
ScaleWidget::~ScaleWidget()
{
  this->dataPtr->connections.clear();
}

//////////////////////////////////////////////////
void ScaleWidget::paintEvent(QPaintEvent *)
{
  QPoint topLeft(0, 20);
  QPoint bottomRight(100, 40);
  QPointF midPoint = (topLeft + bottomRight)/2;

  QPainter painter(this);
  QPen rulerPen;
  rulerPen.setColor(Qt::gray);
  rulerPen.setWidth(3);
  painter.setPen(rulerPen);
  painter.drawLine(topLeft.x(), midPoint.y(), bottomRight.x(), midPoint.y());
  painter.drawLine(topLeft.x(), topLeft.y(), topLeft.x(), bottomRight.y());
  painter.drawLine(bottomRight.x(), topLeft.y(), bottomRight.x(),
    bottomRight.y());

  QPoint textTopLeft(topLeft.x(), 2*topLeft.y() - bottomRight.y());
  QPoint textBottomRight(bottomRight.x(), textTopLeft.y() +
    (bottomRight.y() - topLeft.y()));
  QRect rulerRect(textTopLeft, textBottomRight);
  painter.drawText(rulerRect, Qt::AlignHCenter,
    QString(this->dataPtr->scaleText.c_str()));
}

//////////////////////////////////////////////////
void ScaleWidget::OnChangeZoom(const double _zoomFactor)
{
  std::stringstream str;
  double places = pow(10.0, 2);
  str << round((1.0/_zoomFactor) * places) / places << " m";
  this->dataPtr->scaleText = str.str();
}
