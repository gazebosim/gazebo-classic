/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <map>

#include <qwt/qwt_symbol.h>
#include <qwt/qwt_plot_curve.h>

#include "gazebo/common/Assert.hh"

#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/PlotCurve.hh"

using namespace gazebo;
using namespace gui;

// The number of unique color
static const int ColorCount = 5;

// The unique colors
static const QColor Colors[ColorCount] =
{
  QColor(255, 0, 0),
  QColor(0, 255, 0),
  QColor(0, 0, 255),
  QColor(255, 255, 0),
  QColor(255, 0, 255)
};


namespace gazebo
{
  namespace gui
  {
    /// \brief A class that manages plotting data
    class CurveData: public QwtArraySeriesData<QPointF>
    {
      public: CurveData()
              {}

      public: virtual QRectF boundingRect() const
              {
                if (this->d_boundingRect.width() < 0.0)
                  this->d_boundingRect = qwtBoundingRect(*this);

                return this->d_boundingRect;
              }

      public: inline void Add(const QPointF &_point)
              {
                this->d_samples += _point;
                if (this->d_samples.size() > 11000)
                  this->d_samples.remove(0, 1000);
              }

      public: void Clear()
              {
                this->d_samples.clear();
                this->d_samples.squeeze();
                this->d_boundingRect = QRectF(0.0, 0.0, -1.0, -1.0);
              }

      public: QVector<QPointF> Samples() const
              {
                return this->d_samples;
              }
    };


    /// \internal
    /// \brief PlotCurve private data
    struct PlotCurvePrivate
    {
      /// \brief Unique id;
      public: unsigned int id;

      /// \brief Curve label.
      public: std::string label;

      /// \brief Qwt Curve object.
      public: QwtPlotCurve *curve = NULL;

      /// \brief Curve data in the form of QwtArraySeriesData
      public: CurveData *curveData;

      /// \brief Global id incremented on every new curve
      public: static unsigned int globalCurveId;
    };
  }
}

// global curve id counter
unsigned int PlotCurvePrivate::globalCurveId = 0;

/////////////////////////////////////////////////
PlotCurve::PlotCurve(const std::string &_label)
  : dataPtr(new PlotCurvePrivate())
{
  QwtPlotCurve *curve = new QwtPlotCurve(QString::fromStdString(_label));

  curve->setStyle(QwtPlotCurve::Lines);
  curve->setData(new CurveData());

  QColor penColor = Colors[0];

  /// \todo The following will add the curve to the right hand axis. Need
  /// a better way to do this based on user input.
  // this->enableAxis(QwtPlot::yRight);
  // this->axisAutoScale(QwtPlot::yRight);
  // curve->setYAxis(QwtPlot::yRight);

  QPen pen(penColor);
  pen.setWidth(1.0);
  curve->setPen(pen);
  curve->setStyle(QwtPlotCurve::Lines);

  curve->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,
        Qt::NoBrush, QPen(penColor), QSize(2, 2)));

  this->dataPtr->curve = curve;

  this->dataPtr->curveData =
      static_cast<CurveData *>(this->dataPtr->curve->data());
  GZ_ASSERT(this->dataPtr->curveData != NULL, "Curve data is NULL");

  this->dataPtr->id = PlotCurvePrivate::globalCurveId++;

  this->dataPtr->label = _label;
}

/////////////////////////////////////////////////
PlotCurve::~PlotCurve()
{
  this->dataPtr->curve->detach();
  delete this->dataPtr->curve;
}

/////////////////////////////////////////////////
void PlotCurve::AddPoint(const ignition::math::Vector2d &_pt)
{
  // Add a point
  this->dataPtr->curveData->Add(QPointF(_pt.X(), _pt.Y()));
}

/////////////////////////////////////////////////
void PlotCurve::AddPoints(const std::vector<ignition::math::Vector2d> &_pts)
{
  // Add all the points
  for (const auto &pt : _pts)
  {
    this->dataPtr->curveData->Add(QPointF(pt.X(), pt.Y()));
  }
}

/////////////////////////////////////////////////
void PlotCurve::Clear()
{
  this->dataPtr->curveData->Clear();
}

/////////////////////////////////////////////////
void PlotCurve::Detach()
{
  this->dataPtr->curve->detach();
}

/////////////////////////////////////////////////
void PlotCurve::Attach(IncrementalPlot *_plot)
{
  this->dataPtr->curve->attach(_plot);
}

/////////////////////////////////////////////////
void PlotCurve::SetLabel(const std::string &_label)
{
  this->dataPtr->label = _label;
  this->dataPtr->curve->setTitle(QString::fromStdString(_label));
}

/////////////////////////////////////////////////
std::string PlotCurve::Label() const
{
  return this->dataPtr->label;
}

/////////////////////////////////////////////////
void PlotCurve::SetId(const unsigned int _id)
{
  this->dataPtr->id = _id;
}

/////////////////////////////////////////////////
unsigned int PlotCurve::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
unsigned int PlotCurve::Size() const
{
  return static_cast<unsigned int>(this->dataPtr->curveData->samples().size());
}

/////////////////////////////////////////////////
ignition::math::Vector2d PlotCurve::Point(const unsigned int _index) const
{
  if (_index >= static_cast<unsigned int>(
      this->dataPtr->curveData->samples().size()))
  {
    return ignition::math::Vector2d();
  }

  const QPointF &pt = this->dataPtr->curveData->samples()[_index];
  return ignition::math::Vector2d(pt.x(), pt.y());
}

/////////////////////////////////////////////////
QwtPlotCurve *PlotCurve::Curve()
{
  return this->dataPtr->curve;
}
