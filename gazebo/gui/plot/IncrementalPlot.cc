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
#include <qwt/qwt_plot_canvas.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_marker.h>
#include <qwt/qwt_curve_fitter.h>
#include <qwt/qwt_symbol.h>
#include <qwt/qwt_legend.h>
#include <qwt/qwt_legend_item.h>
#include <qwt/qwt_plot_directpainter.h>
#include <qwt/qwt_plot_magnifier.h>

#include "gazebo/common/Assert.hh"

#include "gazebo/math/Helpers.hh"
#include "gazebo/gui/plot/IncrementalPlot.hh"

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

namespace gazebo
{
  namespace gui
  {
    /// \internal
    /// \brief IncrementalPlot private data
    struct IncrementalPlotPrivate
    {
      /// \brief A map of strings to qwt plot curves.
      // public: typedef std::map<QString, QwtPlotCurve *> CurveMap;

      /// \brief A map of unique ids to plot curves.
      public: typedef std::map<unsigned int, PlotCurve *> CurveMap;

      /// \brief The curve to draw.
      public: CurveMap curves;

      /// \brief Drawing utility
      public: QwtPlotDirectPainter *directPainter;

      /// \brief Pointer to the plot maginfier
      public: QwtPlotMagnifier *magnifier;

      /// \brief Period duration in seconds.
      public: unsigned int period;

      /// \brief Global id incremented on every new curve
      public: static unsigned int globalCurveId;
    };
  }
}

// global variable id counter
unsigned int IncrementalPlotPrivate::globalCurveId = 0;

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
    delete iter.second->curve;
    delete iter.second;
  }

  this->dataPtr->curves.clear();
}

/////////////////////////////////////////////////
PlotCurve *IncrementalPlot::FindCurve(const std::string &_label) const
{
  for (const auto &it : this->dataPtr->curves)
  {
    if (it.second->label == _label)
      return it.second;
  }

  return NULL;
}

/////////////////////////////////////////////////
PlotCurve *IncrementalPlot::FindCurve(const unsigned int _id) const
{
  auto it = this->dataPtr->curves.find(_id);
  if (it != this->dataPtr->curves.end())
    return it->second;
  else
    return NULL;
}

/////////////////////////////////////////////////
void IncrementalPlot::Add(const std::string &_label,
    const std::list<ignition::math::Vector2d> &_pts)
{
  if (_label.empty())
    return;

/*  QwtPlotCurve *curve = NULL;

  CurveMap::iterator iter = this->dataPtr->curves.find(_label);
  if (iter == this->dataPtr->curves.end())
    curve = this->AddCurve(_label);
  else
    curve = iter->second;*/

  PlotCurve *plotCurve = NULL;
  PlotCurve *curve = this->FindCurve(_label);
  if (curve == NULL)
    this->AddCurve(_label);
  else
    plotCurve = curve;

  GZ_ASSERT(plotCurve != NULL, "Curve is NULL");

  // Get the  curve data
  CurveData *curveData = static_cast<CurveData *>(plotCurve->curve->data());

  GZ_ASSERT(curveData != NULL, "Curve data is NULL");

  // Add all the points
  for (const auto &pt : _pts)
  {
    curveData->Add(QPointF(pt.X(), pt.Y()));
  }

  // Adjust the curve
  this->AdjustCurve(curve);
}

/////////////////////////////////////////////////
void IncrementalPlot::Add(const std::string &_label,
    const ignition::math::Vector2d &_pt)
{
  if (_label.empty())
    return;

/*  QwtPlotCurve *curve = NULL;

  CurveMap::iterator iter = this->dataPtr->curves.find(_label);
  if (iter == this->dataPtr->curves.end())
    curve = this->AddCurve(_label);
  else
    curve = iter->second;*/

  PlotCurve *plotCurve = NULL;
  PlotCurve *curve = this->FindCurve(_label);
  if (curve == NULL)
    this->AddCurve(_label);
  else
    plotCurve = curve;

  GZ_ASSERT(plotCurve != NULL, "Curve is NULL");

  // Get the curve data
  CurveData *curveData = static_cast<CurveData *>(plotCurve->curve->data());

  GZ_ASSERT(curveData != NULL, "Curve data is NULL");

  // Add a point
  curveData->Add(QPointF(_pt.X(), _pt.Y()));
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
void IncrementalPlot::AdjustCurve(PlotCurve *_plotCurve)
{
  GZ_ASSERT(_plotCurve != NULL, "Plot curve is NULL");

  QwtPlotCurve *curve = _plotCurve->curve;
  GZ_ASSERT(curve != NULL, "Qwt curve is NULL");

  CurveData *curveData = static_cast<CurveData *>(curve->data());
  const QPointF &lastPoint = curveData->samples().back();

  const bool doClip = !this->canvas()->testAttribute(Qt::WA_PaintOnScreen);

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
  }

  this->setAxisScale(this->xBottom,
      std::max(0.0, static_cast<double>(lastPoint.x() - this->dataPtr->period)),
      std::max(1.0, static_cast<double>(lastPoint.x())));

  // this->setAxisScale(curve->yAxis(), 0.0, curve->maxYValue() * 2.0);

  // this->setAxisAutoScale(this->yRight, true);
  // this->setAxisAutoScale(this->yLeft, true);

  this->dataPtr->directPainter->drawSeries(curve,
      curveData->size() - 1, curveData->size() - 1);

  this->replot();
}

/////////////////////////////////////////////////
PlotCurve *IncrementalPlot::AddCurve(const std::string &_label)
{
  QwtPlotCurve *curve = new QwtPlotCurve(QString::fromStdString(_label));

  curve->setStyle(QwtPlotCurve::Lines);
  curve->setData(new CurveData());

  PlotCurve *plotCurve = this->FindCurve(_label);
  if (plotCurve != NULL)
  {
    return plotCurve;
    /*CurveData *curveData = static_cast<CurveData*>(plotCurve->curve->data());
    curveData->Clear();
    delete plotCurve->curve;
    delete plotCurve;*/
  }

  // Delete an old curve if it exists.
/*  if (this->dataPtr->curves.find(_label) != this->dataPtr->curves.end())
  {
    CurveData *curveData = static_cast<CurveData*>(
        this->dataPtr->curves[_label]->data());
    curveData->Clear();
    delete this->dataPtr->curves[_label];
  }

  this->dataPtr->curves[_label] = curve;*/

  QColor penColor = Colors[(this->dataPtr->curves.size()-1) % ColorCount];

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

  plotCurve = new PlotCurve;
  plotCurve->id = IncrementalPlotPrivate::globalCurveId++;
  plotCurve->label = _label;
  plotCurve->curve = curve;

  this->dataPtr->curves[plotCurve->id] = plotCurve;

  return plotCurve;
}

/////////////////////////////////////////////////
void IncrementalPlot::Clear(const std::string &_label)
{
/*  CurveMap::iterator iter = this->dataPtr->curves.find(_label);

  if (iter == this->dataPtr->curves.end())
    return;*/

  PlotCurve *plotCurve = this->FindCurve(_label);
  if (!plotCurve)
    return;

  CurveData *curveData = static_cast<CurveData *>(plotCurve->curve->data());
  curveData->Clear();

  this->dataPtr->curves.erase(plotCurve->id);
  delete plotCurve->curve;
  delete plotCurve;

  this->replot();
}

/////////////////////////////////////////////////
void IncrementalPlot::Clear()
{
/*  for (CurveMap::iterator iter = this->dataPtr->curves.begin();
       iter != this->dataPtr->curves.end(); ++iter)
  {
    CurveData *curveData = static_cast<CurveData *>(iter->second->data());
    curveData->Clear();
    delete iter->second;
  }*/

  for (auto &c : this->dataPtr->curves)
  {
    CurveData *curveData = static_cast<CurveData *>(c.second->curve->data());
    curveData->Clear();
    delete c.second->curve;
    delete c.second;
  }

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
  return this->FindCurve(_label) != NULL;
}

/////////////////////////////////////////////////
void IncrementalPlot::Update()
{
/*  for (CurveMap::iterator iter = this->dataPtr->curves.begin();
       iter != this->dataPtr->curves.end(); ++iter)
  {
    this->AdjustCurve(iter->second);
  }*/
  for (auto &c : this->dataPtr->curves)
  {
    this->AdjustCurve(c.second);
  }
}

/////////////////////////////////////////////////
void IncrementalPlot::SetPeriod(const unsigned int _seconds)
{
  this->dataPtr->period = _seconds;
}

/////////////////////////////////////////////////
void IncrementalPlot::AttachCurve(PlotCurve *_plotCurve)
{
  std::cerr << " attach curve? " << std::endl;
  if (!_plotCurve || !_plotCurve->curve)
    return;

  std::cerr << " attach curve done !!" << _plotCurve->label << std::endl;
  _plotCurve->curve->attach(this);
  this->dataPtr->curves[_plotCurve->id] = _plotCurve;
}

/////////////////////////////////////////////////
PlotCurve *IncrementalPlot::DetachCurve(const unsigned int _id)
{

  std::cerr << " detaching " << _id << std::endl;
  for (auto &c : this->dataPtr->curves)
  {
    std::cerr << " << " << c.first << std::endl;
  }

  PlotCurve *plotCurve =  this->FindCurve(_id);

  if (!plotCurve || !plotCurve->curve)
  {
    std::cerr << " detach curve id not found " << _id << std::endl;
    return NULL;
  }

  plotCurve->curve->detach();
  this->dataPtr->curves.erase(_id);
  return plotCurve;
}

/////////////////////////////////////////////////
void IncrementalPlot::RemoveCurve(unsigned int _id)
{
  auto it = this->dataPtr->curves.find(_id);

  if (!it->second || !it->second->curve)
    return;

  it->second->curve->detach();

  delete it->second->curve;
  delete it->second;

  this->dataPtr->curves.erase(it);
}

/////////////////////////////////////////////////
void IncrementalPlot::SetCurveLabel(const unsigned int _id,
    const std::string &_label)
{
  if (_label.empty())
    return;

  PlotCurve *plotCurve = this->FindCurve(_id);

  if (!plotCurve)
    return;

  this->setTitle(QString::fromStdString(_label));
}
