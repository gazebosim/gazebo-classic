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

#include <ignition/math/Helpers.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/PlotTracker.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Zoom to mouse position
    class PlotMagnifier : public QwtPlotMagnifier
    {
      /// \brief Constructor
      /// \param[in] _canvas Canvas the magnifier will be attached to.
#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
      public: PlotMagnifier(QwtPlotCanvas *_canvas)
#else
      public: PlotMagnifier(QWidget *_canvas)
#endif
                : QwtPlotMagnifier(_canvas)
              {
                // invert the wheel direction
                double f = this->wheelFactor();
                if (!ignition::math::equal(f, 0.0))
                  this->setWheelFactor(1/f);
              }

      /// \brief Callback for a mouse wheel event
      /// \param[in] _wheelEvent Qt mouse wheel event
      protected: virtual void widgetWheelEvent(QWheelEvent *_wheelEvent)
              {
                this->mousePos = _wheelEvent->pos();

                QwtMagnifier::widgetWheelEvent(_wheelEvent);
              }

      /// \brief Update plot scale by changing bounds of x and y axes.
      /// \param[in] _factor Factor to scale the plot by.
      protected: virtual void rescale(double _factor)
              {
                QwtPlot *plt = plot();
                if (plt == nullptr)
                    return;

                double factor = qAbs(_factor);
                if (ignition::math::equal(factor, 1.0) ||
                    ignition::math::equal(factor, 0.0))
                {
                  return;
                }

                const bool autoReplot = plt->autoReplot();
                plt->setAutoReplot(false);

                // canvas maps are used to transform between widget (paint) and
                // canvas (scale) coordinates
                const QwtScaleMap xScaleMap = plt->canvasMap(QwtPlot::xBottom);
                const QwtScaleMap yScaleMap = plt->canvasMap(QwtPlot::yLeft);

                // get coordinates of lower and upper x and y axis bounds
                double xV1 = xScaleMap.s1();
                double xV2 = xScaleMap.s2();
                double yV1 = yScaleMap.s1();
                double yV2 = yScaleMap.s2();

                // transform to work in widget coordinates
                if (xScaleMap.transformation())
                {
                  xV1 = xScaleMap.transform(xV1);
                  xV2 = xScaleMap.transform(xV2);
                }
                if (yScaleMap.transformation())
                {
                  yV1 = yScaleMap.transform(yV1);
                  yV2 = yScaleMap.transform(yV2);
                }

                // zoom in at mouse point
                // 1. center the canvas at mouse point
                // 2. zoom by specified factor
                // 3. translate the the canvas back so that the zoom point is
                // under the mouse

                double xHalfWidth = (xV2 - xV1) * 0.5;
                double yHalfHeight = (yV2 - yV1) * 0.5;

                double xHalfWidthScaled = xHalfWidth * factor;
                double yHalfHeightScaled = yHalfHeight * factor;

                double xCenter = 0.5 * (xV1 + xV2);
                double yCenter = 0.5 * (yV1 + yV2);

                QPoint zoomPos = this->mousePos;
                QPointF trans = zoomPos - QPointF(xCenter, yCenter);
                trans = trans*factor;

                xV1 = zoomPos.x() - xHalfWidthScaled - trans.x();
                xV2 = zoomPos.x() + xHalfWidthScaled - trans.x();

                yV1 = zoomPos.y() - yHalfHeightScaled - trans.y();
                yV2 = zoomPos.y() + yHalfHeightScaled - trans.y();

                // transform back to canvas coordinates
                if (xScaleMap.transformation())
                {
                  xV1 = xScaleMap.invTransform(xV1);
                  xV2 = xScaleMap.invTransform(xV2);
                }
                if (yScaleMap.transformation())
                {
                  yV1 = yScaleMap.invTransform(yV1);
                  yV2 = yScaleMap.invTransform(yV2);
                }

                // zoom by setting axis scale
                plt->setAxisScale(QwtPlot::xBottom, xV1, xV2);
                plt->setAxisScale(QwtPlot::yLeft, yV1, yV2);

                plt->setAutoReplot(autoReplot);
                plt->replot();
              }

        /// \brief Mouse position
        private: QPoint mousePos;
    };

    /// \internal
    /// \brief IncrementalPlot private data
    class IncrementalPlotPrivate
    {
      /// \brief A map of unique ids to plot curves.
      public: typedef std::map<unsigned int, PlotCurvePtr > CurveMap;

      /// \brief The curve to draw.
      public: CurveMap curves;

      /// \brief Drawing utility
      public: QwtPlotDirectPainter *directPainter;

      /// \brief Pointer to the plot magnifier.
      public: PlotMagnifier *magnifier;

      /// \brief Pointer to the plot tracker.
      public: PlotTracker *tracker;

      /// \brief Pointer to the plot zoomer.
      public: QwtPlotZoomer *zoomer;

      /// \brief Pointer to the plot panner.
      public: QwtPlotPanner *panner;

      /// \brief Pointer to the grid lines.
      public: QwtPlotGrid *grid;

      /// \brief Period duration in seconds.
      public: double period;

      /// \brief Previous last point on plot
      public: ignition::math::Vector2d prevPoint;
    };
  }
}

/////////////////////////////////////////////////
IncrementalPlot::IncrementalPlot(QWidget *_parent)
  : QwtPlot(_parent),
    dataPtr(new IncrementalPlotPrivate)
{
  this->setObjectName("incrementalPlot");

  this->dataPtr->period = 10;
  this->dataPtr->directPainter = new QwtPlotDirectPainter(this);

  // panning with the left mouse button
  this->dataPtr->panner = new QwtPlotPanner(this->canvas());

  // line hover display
  this->dataPtr->tracker = new PlotTracker(this->canvas());
  this->dataPtr->tracker->setEnabled(false);

  // box zoom
  this->dataPtr->zoomer = new QwtPlotZoomer(this->canvas());
  this->dataPtr->zoomer->setMousePattern(QwtEventPattern::MouseSelect1,
      Qt::MidButton);
  this->dataPtr->zoomer->setMousePattern(QwtEventPattern::MouseSelect2,
      Qt::RightButton, Qt::ControlModifier);
  this->dataPtr->zoomer->setMousePattern(QwtEventPattern::MouseSelect3,
      Qt::NoButton);
  this->dataPtr->zoomer->setTrackerMode(QwtPicker::AlwaysOff);

  // zoom in/out with the wheel
  this->dataPtr->magnifier = new PlotMagnifier(this->canvas());

#if defined(Q_WS_X11)
  this->canvas()->setAttribute(Qt::WA_PaintOutsidePaintEvent, true);
  this->canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif

  this->setAutoReplot(false);

  this->setFrameStyle(QFrame::NoFrame);
  this->setLineWidth(0);

  this->plotLayout()->setAlignCanvasToScales(true);

  QwtLegend *qLegend = new QwtLegend;
  this->insertLegend(qLegend, QwtPlot::RightLegend, 0.2);


  this->dataPtr->grid = new QwtPlotGrid;

#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
  this->dataPtr->grid->setMajPen(QPen(Qt::gray, 0, Qt::DotLine));
#else
  this->dataPtr->grid->setMajorPen(QPen(Qt::gray, 0, Qt::DotLine));
#endif
  this->dataPtr->grid->attach(this);

  this->enableAxis(QwtPlot::yLeft);
  this->setAxisScaleEngine(QwtPlot::yLeft, new QwtLinearScaleEngine());
  this->setAxisAutoScale(QwtPlot::yLeft, true);

  this->enableAxis(QwtPlot::xBottom);
  this->setAxisScale(QwtPlot::xBottom, 0, this->dataPtr->period);

  this->ShowAxisLabel(X_BOTTOM_AXIS, true);
  this->ShowAxisLabel(Y_LEFT_AXIS, true);

  this->replot();
  this->setAcceptDrops(true);
}

/////////////////////////////////////////////////
IncrementalPlot::~IncrementalPlot()
{
  this->dataPtr->curves.clear();
}

/////////////////////////////////////////////////
PlotCurveWeakPtr IncrementalPlot::Curve(const std::string &_label) const
{
  for (const auto &it : this->dataPtr->curves)
  {
    if (it.second->Label() == _label)
      return it.second;
  }

  return PlotCurveWeakPtr();
}

/////////////////////////////////////////////////
PlotCurveWeakPtr IncrementalPlot::Curve(const unsigned int _id) const
{
  auto it = this->dataPtr->curves.find(_id);
  if (it != this->dataPtr->curves.end())
    return it->second;
  else
  {
    return PlotCurveWeakPtr();
  }
}

/////////////////////////////////////////////////
void IncrementalPlot::AddPoints(const unsigned int _id,
    const std::vector<ignition::math::Vector2d> &_pts)
{
  PlotCurveWeakPtr plotCurve = this->Curve(_id);

  auto c = plotCurve.lock();
  if (!c)
  {
    gzerr << "Unable to add points. "
        << "Curve with id' " << _id << "' is not found" << std::endl;
    return;
  }

  c->AddPoints(_pts);
}

/////////////////////////////////////////////////
void IncrementalPlot::AddPoint(const unsigned int _id,
    const ignition::math::Vector2d &_pt)
{
  PlotCurveWeakPtr plotCurve = this->Curve(_id);

  auto c = plotCurve.lock();
  if (!c)
  {
    gzerr << "Unable to add point. "
        << "Curve with id' " << _id << "' is not found" << std::endl;
    return;
  }

  c->AddPoint(_pt);
}

/////////////////////////////////////////////////
PlotCurveWeakPtr IncrementalPlot::AddCurve(const std::string &_label)
{
  PlotCurveWeakPtr plotCurve = this->Curve(_label);
  if (!plotCurve.expired())
  {
    gzerr << "Curve '" << _label << "' already exists" << std::endl;
    return plotCurve;
  }

  PlotCurvePtr newPlotCurve(new PlotCurve(_label));
  newPlotCurve->Attach(this);
  this->dataPtr->curves[newPlotCurve->Id()] = newPlotCurve;

  return newPlotCurve;
}

/////////////////////////////////////////////////
void IncrementalPlot::Clear()
{
  for (auto &c : this->dataPtr->curves)
    c.second->Clear();

  this->dataPtr->curves.clear();

  this->replot();
}

/////////////////////////////////////////////////
void IncrementalPlot::Update()
{
  if (this->dataPtr->curves.empty())
    return;

  ignition::math::Vector2d lastPoint;
  for (auto &curve : this->dataPtr->curves)
  {
    if (!curve.second->Active())
      continue;

    unsigned int pointCount = curve.second->Size();
    if (pointCount == 0u)
      continue;

    lastPoint = curve.second->Point(pointCount-1);

    if (ignition::math::isnan(lastPoint.X()) ||
        ignition::math::isnan(lastPoint.Y()))
    {
      continue;
    }

    ignition::math::Vector2d minPt = curve.second->Min();
    ignition::math::Vector2d maxPt = curve.second->Max();

    this->dataPtr->directPainter->drawSeries(curve.second->Curve(),
      pointCount - 1, pointCount - 1);
  }

  // get x axis lower and upper bounds
  const QwtScaleMap xScaleMap = this->canvasMap(QwtPlot::xBottom);
  double lastX = lastPoint.X();
  double prevX = this->dataPtr->prevPoint.X();
  double dx = lastX - prevX;
  double minX = 0;
  double maxX = 0;

  // plot the line until time = period then start moving the x window.
  // Checking the min X bound (xScaleMap.s1()) helps to detect a
  // user zoom at beginning of the plot. At any time the scale is changed
  // (from zooming), the moving window size is updated.
  if ((ignition::math::equal(xScaleMap.s1(), 0.0) &&
      lastX < this->dataPtr->period) || ignition::math::equal(prevX, 0.0))
  {
    // update moving window based on specified period
    minX = std::max(0.0, lastX - this->dataPtr->period);
    maxX = std::max(1.0, lastX);
  }
  else
  {
    // update moving window based on current x scale
    dx = std::max(0.0, dx);
    minX = xScaleMap.s1() + dx;
    maxX = xScaleMap.s2() + dx;
    minX = std::max(0.0, minX);
    maxX = std::max(1.0, maxX);
  }

  this->dataPtr->prevPoint = lastPoint;
  this->setAxisScale(QwtPlot::xBottom, minX, maxX);

  this->dataPtr->tracker->Update();
  this->replot();
}

/////////////////////////////////////////////////
void IncrementalPlot::SetPeriod(const common::Time &_time)
{
  this->dataPtr->period = _time.Double();
}

/////////////////////////////////////////////////
void IncrementalPlot::AttachCurve(PlotCurveWeakPtr _plotCurve)
{
  auto c = _plotCurve.lock();
  if (!c)
    return;

  c->Attach(this);
  this->dataPtr->curves[c->Id()] = c;
}

/////////////////////////////////////////////////
PlotCurvePtr IncrementalPlot::DetachCurve(const unsigned int _id)
{
  PlotCurveWeakPtr plotCurve =  this->Curve(_id);

  auto c = plotCurve.lock();
  if (!c)
    return c;

  c->Detach();
  this->dataPtr->curves.erase(_id);
  return c;
}

/////////////////////////////////////////////////
void IncrementalPlot::RemoveCurve(const unsigned int _id)
{
  this->DetachCurve(_id);
}

/////////////////////////////////////////////////
void IncrementalPlot::SetCurveLabel(const unsigned int _id,
    const std::string &_label)
{
  if (_label.empty())
    return;

  PlotCurveWeakPtr plotCurve = this->Curve(_id);

  auto c = plotCurve.lock();
  if (!c)
    return;

  c->SetLabel(_label);
}

/////////////////////////////////////////////////
void IncrementalPlot::ShowAxisLabel(const PlotAxis _axis, const bool _show)
{
  /// TODO Figure out a way to properly label the x and y axis
  std::string axisLabel;
  QFont axisLabelFont(fontInfo().family(), 10, QFont::Bold);
  switch (_axis)
  {
    case X_BOTTOM_AXIS:
    {
      if (_show)
        axisLabel = "Sim Time (seconds)";
      QwtText xtitle(QString::fromStdString(axisLabel));
      xtitle.setFont(axisLabelFont);
      this->setAxisTitle(QwtPlot::xBottom, xtitle);
      break;
    }
    case X_TOP_AXIS:
    {
      break;
    }
    case Y_LEFT_AXIS:
    {
      if (_show)
        axisLabel = "Variable values";
      QwtText ytitle(QString::fromStdString(axisLabel));
      ytitle.setFont(axisLabelFont);
      this->setAxisTitle(QwtPlot::yLeft, ytitle);
      break;
    }
    case Y_RIGHT_AXIS:
    {
      break;
    }
    default:
      return;
  }
}

/////////////////////////////////////////////////
void IncrementalPlot::ShowGrid(const bool _show)
{
  this->dataPtr->grid->setVisible(_show);
  this->replot();
}

/////////////////////////////////////////////////
bool IncrementalPlot::IsShowGrid() const
{
  return this->dataPtr->grid->isVisible();
}

/////////////////////////////////////////////////
void IncrementalPlot::ShowHoverLine(const bool _show)
{
  this->dataPtr->tracker->setEnabled(_show);
  this->replot();
}

/////////////////////////////////////////////////
bool IncrementalPlot::IsShowHoverLine() const
{
  return this->dataPtr->tracker->isEnabled();
}

/////////////////////////////////////////////////
std::vector<PlotCurveWeakPtr> IncrementalPlot::Curves() const
{
  std::vector<PlotCurveWeakPtr> curves;
  for (const auto &it : this->dataPtr->curves)
    curves.push_back(it.second);

  return curves;
}

/////////////////////////////////////////////////
QSize IncrementalPlot::sizeHint() const
{
  int padding = 50;
  QSize s = QWidget::sizeHint();
  s.setWidth(s.width()+padding);
  s.setHeight(s.height()+padding);
  return s;
}

/////////////////////////////////////////////////
void IncrementalPlot::dragEnterEvent(QDragEnterEvent *_evt)
{
  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString mimeData =
        _evt->mimeData()->data("application/x-item");
    _evt->setDropAction(Qt::LinkAction);
    if (!mimeData.isEmpty())
    {
      _evt->acceptProposedAction();
      return;
    }
  }
  _evt->ignore();
}

/////////////////////////////////////////////////
void IncrementalPlot::dropEvent(QDropEvent *_evt)
{
  if (_evt->mimeData()->hasFormat("application/x-item"))
  {
    QString mimeData =
        _evt->mimeData()->data("application/x-item");
    if (!mimeData.isEmpty())
    {
      emit VariableAdded(mimeData.toStdString());
      return;
    }
  }
  _evt->ignore();
}
