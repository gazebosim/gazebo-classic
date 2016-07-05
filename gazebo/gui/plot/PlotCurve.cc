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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Color.hh"

#include "gazebo/gui/Conversions.hh"
#include "gazebo/gui/plot/qwt_gazebo.h"
#include "gazebo/gui/plot/IncrementalPlot.hh"
#include "gazebo/gui/plot/PlotCurve.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Color palette for the PlotCurve
    class ColorPalette
    {
      /// \brief Number of unique colors in a color group.
      public: static const int ColorCount = 3;

      /// \brief Number of color groups.
      public: static const int ColorGroupCount = 4;

      /// \brief Unique colors
      public: static const common::Color Colors[ColorGroupCount][ColorCount];
    };

    /// \brief A class that manages curve data
    class CurveData: public QwtArraySeriesData<QPointF>
    {
      public: CurveData()
              {}

      /// \brief Add a point to the sample.
      /// \return Bounding box of the sample.
      public: virtual QRectF boundingRect() const
              {
                if (this->d_boundingRect.width() < 0.0)
                  this->d_boundingRect = qwtBoundingRect(*this);

                // set a minimum bounding box height
                // this prevents plot's auto scale to zoom in on near-zero
                // floating point noise.
                double minHeight = 1e-3;
                double absHeight = std::fabs(this->d_boundingRect.height());
                if (absHeight < minHeight)
                {
                  double halfMinHeight = minHeight * 0.5;
                  double mid = this->d_boundingRect.top() +
                      (absHeight * 0.5);
                  this->d_boundingRect.setTop(mid - halfMinHeight);
                  this->d_boundingRect.setBottom(mid + halfMinHeight);
                }

                return this->d_boundingRect;
              }

      /// \brief Add a point to the sample.
      /// \param[in] _point Point to add.
      public: inline void Add(const QPointF &_point)
              {
                this->d_samples += _point;

                if (this->d_samples.size() > maxSampleSize)
                {
                  // remove sample window
                  // update bounding rect?
                  this->d_samples.remove(0, windowSize);
                }

                if (this->d_samples.size() == 1)
                {
                  // init bounding rect
                  this->d_boundingRect.setTopLeft(_point);
                  this->d_boundingRect.setBottomRight(_point);
                  return;
                }

                // expand bounding rect
                if (_point.x() < this->d_boundingRect.left())
                  this->d_boundingRect.setLeft(_point.x());
                else if (_point.x() > this->d_boundingRect.right())
                  this->d_boundingRect.setRight(_point.x());
                if (_point.y() < this->d_boundingRect.top())
                  this->d_boundingRect.setTop(_point.y());
                else if (_point.y() > this->d_boundingRect.bottom())
                  this->d_boundingRect.setBottom(_point.y());
              }

      /// \brief Clear the sample data.
      public: void Clear()
              {
                this->d_samples.clear();
                this->d_samples.squeeze();
                this->d_boundingRect = QRectF(0.0, 0.0, -1.0, -1.0);
              }

      /// \brief Get the sample data.
      /// \return A vector of same points.
      public: QVector<QPointF> Samples() const
              {
                return this->d_samples;
              }

      /// \brief maxium sample size of this curve.
      private: int maxSampleSize = 11000;

      /// \brief Size of samples to remove when maxSampleSize is reached.
      private: int windowSize = 1000;
    };


    /// \internal
    /// \brief PlotCurve private data
    class PlotCurvePrivate
    {
      /// \brief Unique id;
      public: unsigned int id;

      /// \brief Curve label.
      public: std::string label;

      /// \brief Active state of the plot curve;
      public: bool active = true;

      /// \brief Age of the curve since the first restart;
      public: unsigned int age = 0;

      /// \brief Qwt Curve object.
      public: QwtPlotCurve *curve = nullptr;

      /// \brief Curve data in the form of QwtArraySeriesData
      public: CurveData *curveData;

      /// \brief Global id incremented on every new curve
      public: static unsigned int globalCurveId;

      /// \brief Color counter to cycle through all available colors
      public: static unsigned int colorCounter;
    };
  }
}

const common::Color ColorPalette::Colors
    [ColorPalette::ColorGroupCount][ColorPalette::ColorCount] =
    {
      // purple
      {
        // 0x882e72
        common::Color(136, 46, 114),
        // 0xb178a6
        common::Color(177, 120, 166),
        // 0xd6c1de
        common::Color(214, 193, 222)
      },
      // blue
      {
        // 0x1965b0
        common::Color(25, 101, 176),
        // 0x5289c7
        common::Color(82, 137, 199),
        // 0x7bafde
        common::Color(123, 175, 222)
      },
      // green
      {
        // 0x4eb265
        common::Color(78, 178, 101),
        // 0x90c987
        common::Color(144, 201, 135),
        // 0xcae0ab
        common::Color(202, 224, 171)
      },
      // red
      {
        // 0xdc050c
        common::Color(220, 5, 12),
        // 0xe8601c
        common::Color(232, 96, 28),
        // 0xf1932d
        common::Color(241, 147, 45)
      }
    };

// global curve id counter
unsigned int PlotCurvePrivate::globalCurveId = 0;

// curve color counter
unsigned int PlotCurvePrivate::colorCounter = 0;

/////////////////////////////////////////////////
PlotCurve::PlotCurve(const std::string &_label)
  : dataPtr(new PlotCurvePrivate())
{
  QwtPlotCurve *curve = new QwtPlotCurve(QString::fromStdString(_label));
  this->dataPtr->curve = curve;

  curve->setYAxis(QwtPlot::yLeft);
  curve->setStyle(QwtPlotCurve::Lines);
  curve->setData(new CurveData());

  int colorGroup = this->dataPtr->colorCounter % ColorPalette::ColorGroupCount;
  int color = static_cast<int>(
      this->dataPtr->colorCounter / ColorPalette::ColorGroupCount)
      % ColorPalette::ColorCount;
  this->dataPtr->colorCounter++;
  QColor penColor =
      Conversions::Convert(ColorPalette::Colors[colorGroup][color]);

  QPen pen(penColor);
  pen.setWidth(1.0);
  curve->setPen(pen);
  curve->setStyle(QwtPlotCurve::Lines);

  curve->setSymbol(new QwtSymbol(QwtSymbol::Ellipse,
        Qt::NoBrush, QPen(penColor), QSize(2, 2)));

  this->dataPtr->curveData =
      static_cast<CurveData *>(this->dataPtr->curve->data());
  GZ_ASSERT(this->dataPtr->curveData != nullptr, "Curve data is nullptr");

  this->dataPtr->id = PlotCurvePrivate::globalCurveId++;

  this->dataPtr->label = _label;
}

/////////////////////////////////////////////////
PlotCurve::~PlotCurve()
{
}

/////////////////////////////////////////////////
void PlotCurve::AddPoint(const ignition::math::Vector2d &_pt)
{
  if (!this->dataPtr->active)
    return;

  // Add a point
  this->dataPtr->curveData->Add(QPointF(_pt.X(), _pt.Y()));
}

/////////////////////////////////////////////////
void PlotCurve::AddPoints(const std::vector<ignition::math::Vector2d> &_pts)
{
  if (!this->dataPtr->active)
    return;

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
bool PlotCurve::Active() const
{
  return this->dataPtr->active;
}

/////////////////////////////////////////////////
void PlotCurve::SetActive(const bool _active)
{
  this->dataPtr->active = _active;
}

/////////////////////////////////////////////////
unsigned int PlotCurve::Age() const
{
  return this->dataPtr->age;
}

/////////////////////////////////////////////////
void PlotCurve::SetAge(const unsigned int _age)
{
  this->dataPtr->age = _age;
}

/////////////////////////////////////////////////
unsigned int PlotCurve::Size() const
{
  return static_cast<unsigned int>(this->dataPtr->curveData->samples().size());
}

/////////////////////////////////////////////////
ignition::math::Vector2d PlotCurve::Min()
{
  return ignition::math::Vector2d(this->dataPtr->curve->minXValue(),
      this->dataPtr->curve->minYValue());
}

/////////////////////////////////////////////////
ignition::math::Vector2d PlotCurve::Max()
{
  return ignition::math::Vector2d(this->dataPtr->curve->maxXValue(),
      this->dataPtr->curve->maxYValue());
}

/////////////////////////////////////////////////
ignition::math::Vector2d PlotCurve::Point(const unsigned int _index) const
{
  if (_index >= static_cast<unsigned int>(
      this->dataPtr->curveData->samples().size()))
  {
    return ignition::math::Vector2d(ignition::math::NAN_D,
        ignition::math::NAN_D);
  }

  const QPointF &pt = this->dataPtr->curveData->samples()[_index];
  return ignition::math::Vector2d(pt.x(), pt.y());
}

/////////////////////////////////////////////////
QwtPlotCurve *PlotCurve::Curve()
{
  return this->dataPtr->curve;
}
