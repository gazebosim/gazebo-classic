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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <map>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_scale_widget.h>
#include <qwt/qwt_plot_panner.h>
#include <qwt/qwt_plot_layout.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_curve_fitter.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_legend_item.h>
#include <qwt/qwt_plot_directpainter.h>
#include <qwt/qwt_plot_magnifier.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/math/Helpers.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"

using namespace gazebo;
using namespace gui;


namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief IncrementalPlot private data
    struct IncrementalPlotPrivate
    {
      /// \brief A map of unique ids to plot curves.
      public: typedef std::map<unsigned int, PlotCurvePtr > CurveMap;

      /// \brief The curve to draw.
      public: CurveMap curves;

      /// \brief Drawing utility
      public: QwtPlotDirectPainter *directPainter;

      /// \brief Pointer to the plot maginfier
      public: QwtPlotMagnifier *magnifier;

      /// \brief Period duration in seconds.
      public: unsigned int period;
    };
  }
}

/////////////////////////////////////////////////
IncrementalPlot::IncrementalPlot(QWidget *_parent)
  : QwtPlot(_parent),
    dataPtr(new IncrementalPlotPrivate)
{
  this->dataPtr->period = 10;
  this->dataPtr->directPainter = new QwtPlotDirectPainter(this);

  // panning with the left mouse button
  (void) new QwtPlotPanner(this->canvas());

  // zoom in/out with the wheel
  this->dataPtr->magnifier = new QwtPlotMagnifier(this->canvas());

#if defined(Q_WS_X11)
  this->canvas()->setAttribute(Qt::WA_PaintOutsidePaintEvent, true);
  this->canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif

  this->setAutoReplot(false);

  this->setFrameStyle(QFrame::NoFrame);
  this->setLineWidth(0);
  this->setCanvasLineWidth(2);

  this->plotLayout()->setAlignCanvasToScales(true);

  QwtLegend *qLegend = new QwtLegend;
  qLegend->setItemMode(QwtLegend::CheckableItem);
  this->insertLegend(qLegend, QwtPlot::RightLegend);

  QwtPlotGrid *grid = new QwtPlotGrid;
  grid->setMajPen(QPen(Qt::gray, 0, Qt::DotLine));
  grid->attach(this);

  /// \todo Figure out a way to properly label the y-axis
  QwtText ytitle("Duration (ms)");
  ytitle.setFont(QFont(fontInfo().family(), 10, QFont::Bold));
  this->setAxisTitle(QwtPlot::yLeft, ytitle);

  this->setAxisAutoScale(QwtPlot::yRight, true);
  this->setAxisAutoScale(QwtPlot::yLeft, true);

  this->replot();

  this->setAcceptDrops(true);
}

/////////////////////////////////////////////////
IncrementalPlot::~IncrementalPlot()
{
  for (auto iter : this->dataPtr->curves)
  {
    iter.second->Clear();
    iter.second->Detach();
  }

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
    return PlotCurveWeakPtr();
}

/////////////////////////////////////////////////
void IncrementalPlot::Add(const std::string &_label,
    const std::vector<ignition::math::Vector2d> &_pts)
{
  if (_label.empty())
    return;

  PlotCurveWeakPtr plotCurve;

  // add curve if not found
  PlotCurveWeakPtr curve = this->Curve(_label);
  if (curve.expired())
    plotCurve = this->AddCurve(_label);
  else
    plotCurve = curve;

  GZ_ASSERT(!plotCurve.expired(), "Curve is NULL");

  auto c = plotCurve.lock();
  c->AddPoints(_pts);
}

/////////////////////////////////////////////////
void IncrementalPlot::Add(const std::string &_label,
    const ignition::math::Vector2d &_pt)
{
  if (_label.empty())
    return;

  PlotCurveWeakPtr plotCurve;

  PlotCurveWeakPtr curve = this->Curve(_label);
  if (curve.expired())
    plotCurve = this->AddCurve(_label);
  else
    plotCurve = curve;

  GZ_ASSERT(!plotCurve.expired(), "Curve is NULL");

  auto c = plotCurve.lock();
  c->AddPoint(_pt);
}

/////////////////////////////////////////////////
void IncrementalPlot::AddVLine(const std::string &_label, const double _x)
{
  QwtPlotMarker *marker = new QwtPlotMarker();
  marker->setValue(_x, 0.0);
  marker->setLineStyle(QwtPlotMarker::VLine);
  marker->setLabelAlignment(Qt::AlignRight | Qt::AlignBottom);
  marker->setLinePen(QPen(Qt::green, 0, Qt::DashDotLine));
  marker->attach(this);
  marker->setLabel(QString::fromStdString(_label));
}

/////////////////////////////////////////////////
void IncrementalPlot::AdjustCurve(PlotCurvePtr _plotCurve)
{
  if (!_plotCurve)
    return;

  unsigned int pointCount = _plotCurve->Size();
  if (pointCount == 0u)
    return;

  ignition::math::Vector2d lastPoint = _plotCurve->Point(pointCount-1);

/*  const bool doClip = !this->canvas()->testAttribute(Qt::WA_PaintOnScreen);

  if (doClip)
  {
    // Depending on the platform setting a clip might be an important
    // performance issue. F.e. for Qt Embedded this reduces the
    // part of the backing store that has to be copied out - maybe
    // to an unaccelerated frame buffer device.
    const QwtScaleMap xMap = this->canvasMap(curve->xAxis());
    const QwtScaleMap yMap = this->canvasMap(curve->yAxis());

    QRegion clipRegion;

    const QSize symbolSize = curve->symbol()->size();
    QRect r(0, 0, symbolSize.width() + 2, symbolSize.height() + 2);

    const QPointF center = QwtScaleMap::transform(xMap, yMap, lastPoint);
    r.moveCenter(center.toPoint());
    clipRegion += r;

    this->dataPtr->directPainter->setClipRegion(clipRegion);
  }*/

  this->setAxisScale(this->xBottom,
      std::max(0.0, static_cast<double>(lastPoint.X() - this->dataPtr->period)),
      std::max(1.0, static_cast<double>(lastPoint.X())));

  // this->setAxisScale(curve->yAxis(), 0.0, curve->maxYValue() * 2.0);

  // this->setAxisAutoScale(this->yRight, true);
  // this->setAxisAutoScale(this->yLeft, true);

  this->dataPtr->directPainter->drawSeries(_plotCurve->Curve(),
      pointCount - 1, pointCount - 1);

  this->replot();
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
void IncrementalPlot::Clear(const std::string &_label)
{
  PlotCurveWeakPtr plotCurve = this->Curve(_label);
  if (plotCurve.expired())
    return;

  auto c = plotCurve.lock();
  c->Clear();
  c->Detach();
  this->dataPtr->curves.erase(c->Id());

  this->replot();
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
QSize IncrementalPlot::sizeHint() const
{
  return QSize(540, 400);
}

/////////////////////////////////////////////////
void IncrementalPlot::dragEnterEvent(QDragEnterEvent *_evt)
{
  if (_evt->mimeData()->hasFormat("application/x-item") &&
      _evt->source() != this)
  {
    _evt->setDropAction(Qt::LinkAction);
    _evt->acceptProposedAction();
  }
  else
    _evt->ignore();
}

/////////////////////////////////////////////////
void IncrementalPlot::dropEvent(QDropEvent *_evt)
{
  QString name = _evt->mimeData()->data("application/x-item");
  this->AddCurve(name.toStdString());
}

/////////////////////////////////////////////////
bool IncrementalPlot::HasCurve(const std::string &_label)
{
  return !this->Curve(_label).expired();
}

/////////////////////////////////////////////////
void IncrementalPlot::Update()
{
  for (auto &c : this->dataPtr->curves)
    this->AdjustCurve(c.second);
}

/////////////////////////////////////////////////
void IncrementalPlot::SetPeriod(const unsigned int _seconds)
{
  this->dataPtr->period = _seconds;
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
  auto it = this->dataPtr->curves.find(_id);

  if (!it->second)
    return;

  this->dataPtr->curves.erase(it);
}

/////////////////////////////////////////////////
void IncrementalPlot::SetCurveLabel(const unsigned int _id,
    const std::string &_label)
{
  if (_label.empty())
    return;

  PlotCurveWeakPtr plotCurve = this->Curve(_id);

  if (plotCurve.expired())
    return;

  auto c = plotCurve.lock();
  c->SetLabel(_label);
}

/////////////////////////////////////////////////
std::vector<PlotCurveWeakPtr> IncrementalPlot::Curves() const
{
  std::vector<PlotCurveWeakPtr> curves;
  for (const auto &it : this->dataPtr->curves)
    curves.push_back(it.second);

  return curves;
}
