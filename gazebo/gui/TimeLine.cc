/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include <boost/math/special_functions/round.hpp>

#include "boost/lexical_cast.hpp"
#include "gazebo/gui/TimeLine.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TimeLine::TimeLine(QWidget *_parent)
  : QSlider(_parent)
{
  this->setMinimumHeight(50);
}

/////////////////////////////////////////////////
TimeLine::~TimeLine()
{
}

/////////////////////////////////////////////////
void TimeLine::SetTime(const common::Time &_time)
{
  this->time = _time;
}

/////////////////////////////////////////////////
void TimeLine::mousePressEvent(QMouseEvent *_event)
{
  std::cout << "XY[" << _event->x() << "]\n";
 // if (_event->x())
}

/////////////////////////////////////////////////
void TimeLine::mouseDoubleClickEvent(QMouseEvent * /*_event*/)
{
}

/////////////////////////////////////////////////
void TimeLine::mouseMoveEvent(QMouseEvent * /*_event*/)
{
}

/////////////////////////////////////////////////
void TimeLine::mouseReleaseEvent(QMouseEvent * /*_event*/)
{
}

/////////////////////////////////////////////////
void TimeLine::paintEvent(QPaintEvent *_evt)
{
  QPainter painter(this);

  float rx = _evt->rect().x();
  float ry = _evt->rect().x();
  float rwidth = _evt->rect().width();
  float rheight = _evt->rect().height();

  QColor black(10, 10, 10);
  QColor white(255, 255, 255);
  QColor orange(245, 129, 19);

  int blackHeight = 40;

  // Draw the box that surrounds the time line
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rx, ry, rwidth, blackHeight, black);

  int timeWidth = 80;
  // Draw the current time
  {
    std::ostringstream stream;
    unsigned int day, hour, min, sec, msec;

    stream.str("");

    sec = this->time.sec;

    day = sec / 86400;
    sec -= day * 86400;

    hour = sec / 3600;
    sec -= hour * 3600;

    min = sec / 60;
    sec -= min * 60;

    msec = boost::math::round(this->time.nsec * 1e-6);

    stream << std::setw(2) << std::setfill('0') << day << " ";
    stream << std::setw(2) << std::setfill('0') << hour << ":";
    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec << ".";
    stream << std::setw(3) << std::setfill('0') << msec;

    painter.setPen(QPen(QBrush(white), 1));
    painter.setFont(QFont("Arial", 12, QFont::Light));
    painter.drawText(QRectF(rx, ry, timeWidth, blackHeight),
        Qt::AlignRight | Qt::AlignVCenter, tr(stream.str().c_str()));
  }

  int triangleWidth = 9;
  int triangleHeight = 12;
  QPointF trianglePts[3];
  trianglePts[0] = QPointF(rx + timeWidth - triangleWidth + this->value(),
                           ry + blackHeight + triangleHeight);
  trianglePts[1] = QPointF(rx + timeWidth + triangleWidth + this->value(),
                           ry + blackHeight + triangleHeight);
  trianglePts[2] = QPointF(rx + timeWidth + this->value(), ry + blackHeight);

  painter.setBrush(orange);
  painter.setPen(orange);
  painter.drawPolygon(trianglePts, 3);

  painter.setPen(QPen(QBrush(orange), 2));
  painter.drawLine(rx + timeWidth + this->value(), ry,
                   rx + timeWidth + this->value(),
                   ry + timeWidth + blackHeight);

  painter.setFont(QFont("Arial", 8, QFont::Light));
  for (int i = 0; i < 100; ++i)
  {
    float textX = i * 30 + timeWidth;
    float textY = ry;
    std::ostringstream timeMark;
    timeMark << i;
    painter.setPen(white);
    painter.drawText(QRectF(textX, textY, 30, 15), Qt::AlignCenter,
        tr(timeMark.str().c_str()));

    painter.drawLine(textX + 15, textY + 15, textX + 15, textY + blackHeight);
  }
}
