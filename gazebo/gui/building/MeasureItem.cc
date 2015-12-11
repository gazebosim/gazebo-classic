/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <ignition/math/Angle.hh>

#include "gazebo/common/Exception.hh"
#include "gazebo/gui/building/EditorView.hh"
#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/MeasureItem.hh"
#include "gazebo/gui/building/MeasureItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
MeasureItem::MeasureItem(const QPointF &_start, const QPointF &_end)
    : SegmentItem(*new MeasureItemPrivate),
      dataPtr(std::static_pointer_cast<MeasureItemPrivate>(this->editorDPtr))
{
  //auto dPtr = static_cast<MeasureItemPrivate *>(this->dataPtr);

  this->dataPtr->editorType = "Measure";

  this->setFlag(QGraphicsItem::ItemSendsGeometryChanges);
  this->setAcceptHoverEvents(true);

  this->SetLine(_start, _end);
  this->dataPtr->zValueSelected = 8;
  this->setZValue(this->dataPtr->zValueSelected);
  this->ShowHandles(false);

  this->dataPtr->value = 0;
}

/////////////////////////////////////////////////
MeasureItem::~MeasureItem()
{
}

/////////////////////////////////////////////////
void MeasureItem::paint(QPainter *_painter,
    const QStyleOptionGraphicsItem */*_option*/, QWidget */*_widget*/)
{
  //auto this->dataPtr = static_cast<MeasureItemPrivate *>(this->dataPtr);

  QPointF p1 = this->line().p1();
  QPointF p2 = this->line().p2();
  double angle = IGN_DTOR(this->line().angle());

  QPen measurePen;
  measurePen.setStyle(Qt::SolidLine);
  measurePen.setColor(QColor(247, 142, 30, 120));
  double tipLength = 10;
  measurePen.setWidth(3);
  _painter->setPen(measurePen);

  // Line
  _painter->drawLine(this->line());

  // End tips
  _painter->drawLine(QPointF(p1.x()+tipLength*qCos(angle+M_PI/2.0),
                             p1.y()-tipLength*qSin(angle+M_PI/2.0)),
                     QPointF(p1.x()-tipLength*qCos(angle+M_PI/2.0),
                             p1.y()+tipLength*qSin(angle+M_PI/2.0)));

  _painter->drawLine(QPointF(p2.x()+tipLength*qCos(angle+M_PI/2.0),
                             p2.y()-tipLength*qSin(angle+M_PI/2.0)),
                     QPointF(p2.x()-tipLength*qCos(angle+M_PI/2.0),
                             p2.y()+tipLength*qSin(angle+M_PI/2.0)));

  // Value
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << this->dataPtr->value << " m";

  double margin = 10;
  float textWidth = _painter->fontMetrics().width(stream.str().c_str());
  float textHeight = _painter->fontMetrics().height();

  float posX = (p1.x()+p2.x())/2;
  float posY = (p1.y()+p2.y())/2;
  double textAngle = angle;
  if (textAngle > M_PI)
    textAngle = textAngle - M_PI;

  if (textAngle > 0 && textAngle <= M_PI/2.0)
  {
    posX = (p1.x()+p2.x())/2 + margin*qCos(textAngle+M_PI/2.0)-textWidth;
    posY = (p1.y()+p2.y())/2 - margin*qSin(textAngle+M_PI/2.0);
  }
  else if (textAngle > M_PI/2.0 && textAngle < M_PI)
  {
    posX = (p1.x()+p2.x())/2 + margin*qCos(textAngle-M_PI/2.0);
    posY = (p1.y()+p2.y())/2 - margin*qSin(textAngle-M_PI/2.0);
  }
  else if (fabs(textAngle) < 0.01 || fabs(textAngle - M_PI) < 0.01)
  {
    posX = (p1.x()+p2.x())/2 - textWidth/2;
    posY = (p1.y()+p2.y())/2 - margin;
  }

  measurePen.setColor(Qt::white);
  measurePen.setWidth(textHeight*1.5);
  _painter->setPen(measurePen);
  _painter->drawLine(posX+textWidth*0.1, posY-textHeight*0.4,
                     posX+textWidth*0.9, posY-textHeight*0.4);

  measurePen.setColor(QColor(247, 142, 30));
  _painter->setPen(measurePen);
  _painter->drawText(posX, posY, stream.str().c_str());
}

/////////////////////////////////////////////////
double MeasureItem::GetDistance() const
{
  return this->line().length();
}

/////////////////////////////////////////////////
void MeasureItem::SetValue(double _value)
{
//  auto this->dataPtr = static_cast<MeasureItemPrivate *>(this->dataPtr);

  this->dataPtr->value = _value;
}
