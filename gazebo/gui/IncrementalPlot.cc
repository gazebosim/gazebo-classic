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

#include <qwt/qwt_plot.h>
#include <qwt/qwt_scale_widget.h>
#include <qwt/qwt_plot_panner.h>
#include <qwt/qwt_plot_layout.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_curve_fitter.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_legend_item.h>
#include <qwt/qwt_plot_directpainter.h>

#include "gazebo/common/Assert.hh"

#include "gazebo/math/Helpers.hh"
#include "gazebo/gui/IncrementalPlot.hh"

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

// A class that manages plotting data
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
};

/////////////////////////////////////////////////
IncrementalPlot::IncrementalPlot(QWidget *_parent)
  : QwtPlot(_parent)
{
  this->period = 10;
  this->directPainter = new QwtPlotDirectPainter(this);

  // panning with the left mouse button
  (void) new QwtPlotPanner(this->canvas());

  // zoom in/out with the wheel
  this->magnifier = new QwtPlotMagnifier(this->canvas());

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

  /// \todo Figure out a way to properly lable the y-axis
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
  for (CurveMap::iterator iter = this->curves.begin();
       iter != this->curves.end(); ++iter)
  {
    delete iter->second;
  }

  this->curves.clear();
}

/////////////////////////////////////////////////
void IncrementalPlot::Add(const QString &_label,
                          const std::list<QPointF> &_pts)
{
  if (_label.isEmpty())
    return;

  QwtPlotCurve *curve = NULL;

  CurveMap::iterator iter = this->curves.find(_label);
  if (iter == this->curves.end())
    curve = this->AddCurve(_label);
  else
    curve = iter->second;

  GZ_ASSERT(curve != NULL, "Curve is NULL");

  // Get the  curve data
  CurveData *curveData = static_cast<CurveData *>(curve->data());

  GZ_ASSERT(curveData != NULL, "Curve data is NULL");

  // Add all the points
  for (std::list<QPointF>::const_iterator ptIter = _pts.begin();
       ptIter != _pts.end(); ++ptIter)
  {
    curveData->Add(*ptIter);
  }

  // Adjust the curve
  this->AdjustCurve(curve);
}

/////////////////////////////////////////////////
void IncrementalPlot::Add(const QString &_label, const QPointF &_pt)
{
  if (_label.isEmpty())
    return;

  QwtPlotCurve *curve = NULL;

  CurveMap::iterator iter = this->curves.find(_label);
  if (iter == this->curves.end())
    curve = this->AddCurve(_label);
  else
    curve = iter->second;

  GZ_ASSERT(curve != NULL, "Curve is NULL");

  // Get the curve data
  CurveData *curveData = static_cast<CurveData *>(curve->data());

  GZ_ASSERT(curveData != NULL, "Curve data is NULL");

  // Add a point
  curveData->Add(_pt);
}

/////////////////////////////////////////////////
void IncrementalPlot::AddVLine(const QString &_label, double _x)
{
  QwtPlotMarker *marker = new QwtPlotMarker();
  marker->setValue(_x, 0.0);
  marker->setLineStyle(QwtPlotMarker::VLine);
  marker->setLabelAlignment(Qt::AlignRight | Qt::AlignBottom);
  marker->setLinePen(QPen(Qt::green, 0, Qt::DashDotLine));
  marker->attach(this);
  marker->setLabel(_label);
}

/////////////////////////////////////////////////
void IncrementalPlot::AdjustCurve(QwtPlotCurve *_curve)
{
  GZ_ASSERT(_curve != NULL, "Curve is NULL");

  CurveData *curveData = static_cast<CurveData *>(_curve->data());
  const QPointF &lastPoint = curveData->samples().back();

  const bool doClip = !this->canvas()->testAttribute(Qt::WA_PaintOnScreen);

  if (doClip)
  {
    // Depending on the platform setting a clip might be an important
    // performance issue. F.e. for Qt Embedded this reduces the
    // part of the backing store that has to be copied out - maybe
    // to an unaccelerated frame buffer device.
    const QwtScaleMap xMap = this->canvasMap(_curve->xAxis());
    const QwtScaleMap yMap = this->canvasMap(_curve->yAxis());

    QRegion clipRegion;

    const QSize symbolSize = _curve->symbol()->size();
    QRect r(0, 0, symbolSize.width() + 2, symbolSize.height() + 2);

    const QPointF center = QwtScaleMap::transform(xMap, yMap, lastPoint);
    r.moveCenter(center.toPoint());
    clipRegion += r;

    this->directPainter->setClipRegion(clipRegion);
  }

  this->setAxisScale(this->xBottom,
      std::max(0.0, static_cast<double>(lastPoint.x() - this->period)),
      std::max(1.0, static_cast<double>(lastPoint.x())));

  // this->setAxisScale(_curve->yAxis(), 0.0, _curve->maxYValue() * 2.0);

  // this->setAxisAutoScale(this->yRight, true);
  // this->setAxisAutoScale(this->yLeft, true);

  this->directPainter->drawSeries(_curve,
      curveData->size() - 1, curveData->size() - 1);

  this->replot();
}

/////////////////////////////////////////////////
QwtPlotCurve *IncrementalPlot::AddCurve(const QString &_label)
{
  QwtPlotCurve *curve = new QwtPlotCurve(_label);

  curve->setStyle(QwtPlotCurve::Lines);
  curve->setData(new CurveData());

  // Delete an old curve if it exists.
  if (this->curves.find(_label) != this->curves.end())
  {
    CurveData *curveData = static_cast<CurveData*>(
        this->curves[_label]->data());
    curveData->Clear();
    delete this->curves[_label];
  }

  this->curves[_label] = curve;

  QColor penColor = Colors[(this->curves.size()-1) % ColorCount];

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

  curve->attach(this);

  return curve;
}

/////////////////////////////////////////////////
void IncrementalPlot::Clear(const QString &_label)
{
  CurveMap::iterator iter = this->curves.find(_label);

  if (iter == this->curves.end())
    return;

  CurveData *curveData = static_cast<CurveData *>(iter->second->data());
  curveData->Clear();

  delete iter->second;
  this->curves.erase(iter);

  this->replot();
}

/////////////////////////////////////////////////
void IncrementalPlot::Clear()
{
  for (CurveMap::iterator iter = this->curves.begin();
       iter != this->curves.end(); ++iter)
  {
    CurveData *curveData = static_cast<CurveData *>(iter->second->data());
    curveData->Clear();
    delete iter->second;
  }

  this->curves.clear();

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
bool IncrementalPlot::HasCurve(const QString &_label)
{
  return this->curves.find(_label) != this->curves.end();
}

/////////////////////////////////////////////////
void IncrementalPlot::dropEvent(QDropEvent *_evt)
{
  QString name = _evt->mimeData()->data("application/x-item");
  this->AddCurve(name);
}

/////////////////////////////////////////////////
void IncrementalPlot::Update()
{
  for (CurveMap::iterator iter = this->curves.begin();
       iter != this->curves.end(); ++iter)
  {
    this->AdjustCurve(iter->second);
  }
}

/////////////////////////////////////////////////
void IncrementalPlot::SetPeriod(double _seconds)
{
  this->period = _seconds;
}

/////////////////////////////////////////////////
void IncrementalPlot::wheelEvent(QWheelEvent *_event)
{
  if (_event->modifiers() & Qt::ControlModifier)
  {
    double sign = _event->delta() > 0 ? -1.0 : 1.0;
    double scale = 1 + 0.1*sign;
    this->period *= scale;
    this->Update();
    _event->accept();
  }
}
