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

#include <ignition/math/Helpers.hh>

#include "gazebo/gui/qt.h"

#include "gazebo/gui/plot/PlotTracker.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
    /// \brief A widget that renders the hover line inside the main plot canvas.
    class HoverLineWidget : public QWidget
    {
      /// \brief Constructor
      /// \param[in] _picker Plot picker that provides mouse tracking over a
      /// plot canvas.
      /// \param[in] _parent Parent wiget.
      public: HoverLineWidget(QwtPicker *_picker, QWidget *_parent)
        : QWidget(_parent), picker(_picker)
      {
        this->setAttribute(Qt::WA_TransparentForMouseEvents);
        this->setAttribute(Qt::WA_NoSystemBackground);
        this->setFocusPolicy(Qt::NoFocus);
      }

      /// \brief Draw the hover line
      /// \param[in] Qt paint event
      public: virtual void paintEvent(QPaintEvent *_e)
      {
        QPainter painter(this);
        painter.setClipRegion(_e->region());

        painter.setPen(this->picker->rubberBandPen());
        this->Draw(&painter);
      }

      /// \brief Paint function to draw the hover line
      /// \param[in] _painter Qt painter object.
      public: void Draw(QPainter *_painter) const
      {
        const QWidget *widget = this->picker->parentWidget();
        if (!widget)
          return;

        const QPoint trackerPos = this->picker->trackerPosition();
        QPainterPath path;
        path.addRect(widget->contentsRect());
        const QRect pRect = path.boundingRect().toRect();
        int trackerX = trackerPos.x();
        int top = pRect.top();
        int bottom = pRect.bottom();
        QwtPainter::drawLine(_painter, trackerX, top, trackerX, bottom);
      }

      /// \brief Picker object. In this case, the PlotTracker.
      private: QwtPicker *picker;
    };
#endif

    /// \internal
    /// \brief PlotTracker private data
    class PlotTrackerPrivate
    {
#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
      /// \brief The hover line widget drawn over the canvas.
      public: HoverLineWidget *hoverLineWidget = nullptr;
#endif
    };
  }
}

/////////////////////////////////////////////////
#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
PlotTracker::PlotTracker(QwtPlotCanvas *_canvas)
#else
PlotTracker::PlotTracker(QWidget *_canvas)
#endif
  : QwtPlotPicker(_canvas), dataPtr(new PlotTrackerPrivate)
{
  this->setTrackerMode(QwtPlotPicker::AlwaysOn);
  this->setRubberBand(QwtPicker::VLineRubberBand);

#if (QWT_VERSION >= ((6 << 16) | (1 << 8) | 0))
  this->setStateMachine(new QwtPickerTrackerMachine());
#endif
}

/////////////////////////////////////////////////
void PlotTracker::Update()
{
#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
  // default behavior of a tracker widget is that it only updates when mouse
  // moves. This is a workaround to force the tracker text to update given
  // that we are constantly updating the x-axis when simulation is playing.
  if (!this->isActive())
  {
    QWidget *w = const_cast<QWidget *>(this->trackerWidget());
    if (!w)
      return;
    w->update();
  }
#endif
}

/////////////////////////////////////////////////
void PlotTracker::updateDisplay()
{
  // this updates default rubberband and tracker text
  QwtPicker::updateDisplay();

#if (QWT_VERSION < ((6 << 16) | (1 << 8) | 0))
  // update hover line only when zoom is not active
  if (!this->isActive())
  {
    // update display
    QWidget *widget = this->parentWidget();
    if (!widget)
      return;

    if (!this->dataPtr->hoverLineWidget)
      this->dataPtr->hoverLineWidget = new HoverLineWidget(this, widget);

    if (this->trackerPosition().x() < 0 ||
        this->rubberBand() == QwtPicker::NoRubberBand)
    {
      this->dataPtr->hoverLineWidget->hide();
      return;
    }

    // resize in case parent widget size changed.
    this->dataPtr->hoverLineWidget->resize(widget->size());
    this->dataPtr->hoverLineWidget->show();

    // update mask
    QBitmap bm(widget->width(), widget->height());
    bm.fill(Qt::color0);
    QPainter painter(&bm);
    QPen pen = this->rubberBandPen();
    pen.setColor(Qt::color1);
    painter.setPen(pen);

    // draw hover line
    this->dataPtr->hoverLineWidget->Draw(&painter);

    QRegion mask;
    mask = QRegion(bm);
    if (widget && !widget->testAttribute(Qt::WA_PaintOnScreen))
    {
        // The parent widget gets an update for its complete rectangle
        // when the mask is changed in visible state.
        // With this hide/show we only get an update for the
        // previous mask.
        this->dataPtr->hoverLineWidget->hide();
    }
    this->dataPtr->hoverLineWidget->setMask(mask);
    this->dataPtr->hoverLineWidget->setVisible(!mask.isEmpty());

    // update
    this->dataPtr->hoverLineWidget->update();
  }
  else
  {
    if (this->dataPtr->hoverLineWidget)
    {
      this->dataPtr->hoverLineWidget->hide();
    }
  }
#endif
}

/////////////////////////////////////////////////
void PlotTracker::widgetMousePressEvent(QMouseEvent *_e)
{
  this->setRubberBand(QwtPicker::NoRubberBand);
  QwtPicker::widgetMousePressEvent(_e);
}

/////////////////////////////////////////////////
void PlotTracker::widgetMouseReleaseEvent(QMouseEvent *_e)
{
  this->setRubberBand(QwtPicker::VLineRubberBand);
  QwtPicker::widgetMouseReleaseEvent(_e);
}

/////////////////////////////////////////////////
QwtText PlotTracker::trackerTextF(const QPointF &_pos) const
{
  // format hover text
  QwtText tracker;
  tracker.setColor(Qt::white);
  QColor c("#f0f0f0");
  c.setAlpha(200);
  tracker.setBackgroundBrush(c);
  tracker.setRenderFlags(Qt::AlignLeft);

  QString info;
  const QwtPlotItemList curves =
      this->plot()->itemList(QwtPlotItem::Rtti_PlotCurve);
  for (int i = 0; i < curves.size(); ++i)
  {
    const QString curveInfo = this->CurveInfoAt(
        static_cast<const QwtPlotCurve *>(curves[i]), _pos);

    if (!curveInfo.isEmpty())
    {
      if (!info.isEmpty())
        info += "<br>";
      info += curveInfo;
    }
  }
  tracker.setText(info);
  return tracker;
}

/////////////////////////////////////////////////
QString PlotTracker::CurveInfoAt(const QwtPlotCurve *curve,
    const QPointF &_pos) const
{
  const QLineF line = this->CurveLineAt(curve, _pos.x());
  if (line.isNull())
    return QString::null;

  // interpolate
  const double y = line.pointAt(
      (_pos.x() - line.p1().x()) / line.dx()).y();

  // return value string in curve color
  QString info("<font color=""%1"">(%2, %3)</font>");
  return info.arg(curve->pen().color().name()).arg(_pos.x()).arg(y);
}

/////////////////////////////////////////////////
QLineF PlotTracker::CurveLineAt(const QwtPlotCurve *_curve,
    const double _x) const
{
  // line segment of curve at x
  QLineF line;

  if (_curve->dataSize() >= 2)
  {
    const QRectF br = _curve->boundingRect();
    if ((br.width() > 0) && (_x >= br.left()) && (_x <= br.right()))
    {
      // get closest index in the array of points
      int index = this->UpperSampleIndex(*_curve->data(), _x);
      double cmp = _curve->sample(_curve->dataSize() - 1).x();

      if (index == -1 && ignition::math::equal(_x, cmp))
      {
        // the last sample is excluded from UpperSampleIndex
        index = _curve->dataSize() - 1;
      }

      if (index > 0)
      {
        line.setP1(_curve->sample(index - 1));
        line.setP2(_curve->sample(index));
      }
    }
  }
  return line;
}

/////////////////////////////////////////////////
int PlotTracker::UpperSampleIndex(const QwtSeriesData<QPointF> &_series,
    const double _value) const
{
  // binary search to find index in series with the closest value
  const int indexMax = _series.size() - 1;
  if (indexMax < 0 || _value >= _series.sample(indexMax).x())
    return -1;

  int indexMin = 0;
  int n = indexMax;
  while (n > 0)
  {
    const int half = n >> 1;
    const int indexMid = indexMin + half;
    if (_value < _series.sample(indexMid).x())
    {
      n = half;
    }
    else
    {
      indexMin = indexMid + 1;
      n -= half + 1;
    }
  }
  return indexMin;
}
