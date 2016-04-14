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

#include <iostream>

#include <qwt/qwt_painter.h>
#include "gazebo/gui/qt.h"

#include "CurveTracker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
struct compareX
{
  inline bool operator()(const double x, const QPointF &pos) const
  {
    return (x < pos.x());
  }
};


/*/////////////////////////////////////////////////
void CurveTracker::widgetMousePressEvent(QMouseEvent *_e)
{
}

/////////////////////////////////////////////////
void CurveTracker::widgetMouseMoveEvent(QMouseEvent *_e)
{
}

/////////////////////////////////////////////////
void CurveTracker::widgetMouseReleaseEvent(QMouseEvent *_e)
{
}*/

/////////////////////////////////////////////////
void CurveTracker::widgetEnterEvent(QEvent *_event)
{
  QwtPicker::widgetEnterEvent(_event);
}

/////////////////////////////////////////////////
void CurveTracker::widgetLeaveEvent(QEvent */*_event*/)
{
  this->remove();

//  QwtPicker::widgetLeaveEvent(_event);
  QWidget *rw = const_cast<QWidget *>(this->rubberBandWidget());
  if (rw)
    rw->hide();
  QWidget *tw = const_cast<QWidget *>(this->trackerWidget());
  if (tw)
    tw->hide();
  std::cerr << " leave " << std::endl;
}

/////////////////////////////////////////////////
CurveTracker::CurveTracker(QwtPlotCanvas *_canvas)
  : QwtPlotPicker(_canvas)
{
  this->setMousePattern(QwtEventPattern::MouseSelect1,
      Qt::NoButton);
}

/*/////////////////////////////////////////////////
void CurveTracker::drawRubberBand( QPainter *painter ) const
{
  if (!isActive() || rubberBand() == NoRubberBand ||
      rubberBandPen().style() == Qt::NoPen)
  {
    return;
  }

  if (this->pickedPoints().isEmpty())
    return;

  const QPolygon pa = adjustedPoints( this->pickedPoints() );

  QwtPickerMachine::SelectionType selectionType =
      QwtPickerMachine::NoSelection;

  if (this->stateMachine())
      selectionType = this->stateMachine()->selectionType();

  switch (selectionType)
  {
    case QwtPickerMachine::NoSelection:
    case QwtPickerMachine::PointSelection:
    {
      if (pa.count() < 1)
        return;
      const QPoint pos = pa[0];
      QPainterPath path;
      const QWidget *widget = parentWidget();
      if (widget)
        path.addRect( widget->contentsRect() );

      const QRect pRect = path.boundingRect().toRect();
      switch (rubberBand())
      {
        case VLineRubberBand:
        {
          int x = pos.x();
          int top = pRect.top();
          int bottom = pRect.bottom();
          QwtPainter::drawLine( painter, x, top, x, bottom);
          break;
        }
        default:
          break;
      }
      break;
    }
    default:
      break;
  }
}

/////////////////////////////////////////////////
QRect CurveTracker::trackerRect(const QFont &_font) const
{
  QRect r = QwtPlotPicker::trackerRect( _font );
  // align r to the first curve
  const QwtPlotItemList curves = plot()->itemList(
      QwtPlotItem::Rtti_PlotCurve );
  if ( curves.size() > 0 )
  {
    QPainterPath path;

    const QWidget *widget = parentWidget();
    if ( widget )
        path.addRect( widget->contentsRect() );

    const QRect pRect = path.boundingRect().toRect();

    QPointF pos = invTransform( trackerPosition() );
    const QLineF line = curveLineAt(
        static_cast<const QwtPlotCurve *>( curves[0] ), pos.x() );
    if ( !line.isNull() )
    {
      const double curveY = line.pointAt(
          ( pos.x() - line.p1().x() ) / line.dx() ).y();

      pos.setY( curveY );
      pos = transform( pos );

      r.moveBottom( pos.y() );

      r.setLeft( std::max(r.left(), 10));
      r.setRight( std::min(r.right(), pRect.right()-10));
    }
  }
  std::cerr << r.topLeft().x() << " " << r.topLeft().y() << " " <<
      r.bottomRight().x() << " " << r.bottomRight().y() << std::endl;
  return r;
}

/////////////////////////////////////////////////
QwtText CurveTracker::trackerTextF( const QPointF &_pos ) const
{
  QwtText tracker;
  tracker.setColor(Qt::black);
  QColor c( "#333333" );
  // tracker.setBorderPen( QPen( c, 2 ) );
  c.setAlpha( 200 );
  tracker.setBackgroundBrush(c);

  QString info;
  const QwtPlotItemList curves =
      plot()->itemList( QwtPlotItem::Rtti_PlotCurve);
  for ( int i = 0; i < curves.size(); i++)
  {
    const QString curveInfo = curveInfoAt(
        static_cast<const QwtPlotCurve *>(curves[i] ), _pos);

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
QString CurveTracker::curveInfoAt( const QwtPlotCurve *curve,
                                   const QPointF &_pos) const
{
  const QLineF line = curveLineAt(curve, _pos.x());
  if (line.isNull())
    return QString::null;

  const double y = line.pointAt(
      (_pos.x() - line.p1().x() ) / line.dx() ).y();

  QString info("<font color=""%1"">[%2, %3]</font>");
  return info.arg(curve->pen().color().name()).arg(_pos.x()).arg(y);
}

/////////////////////////////////////////////////
int CurveTracker::UpperSampleIndex( const QwtSeriesData<QPointF> &series,
                                    double value ) const
{
  auto lessThan = compareX();
  const int indexMax = series.size() - 1;
  if (indexMax < 0 || !lessThan(value, series.sample(indexMax )))
    return -1;

  int indexMin = 0;
  int n = indexMax;
  while (n > 0)
  {
    const int half = n >> 1;
    const int indexMid = indexMin + half;
    if (lessThan(value, series.sample(indexMid)))
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

/////////////////////////////////////////////////
QLineF CurveTracker::curveLineAt( const QwtPlotCurve *_curve, double _x) const
{
  QLineF line;

  if (_curve->dataSize() >= 2)
  {
    const QRectF br = _curve->boundingRect();
    if ((br.width() > 0) && (_x >= br.left()) && (_x <= br.right()))
    {
      int index = this->UpperSampleIndex(*_curve->data(), _x);
      double cmp = _curve->sample(_curve->dataSize() - 1 ).x();

      if ( index == -1 && (fabs(_x-cmp) < 0.000001) )
      {
        // the last sample is excluded from qwtUpperSampleIndex
        index = _curve->dataSize() - 1;
      }

      if ( index > 0 )
      {
        line.setP1(_curve->sample(index - 1 ));
        line.setP2(_curve->sample(index ));
      }
    }
  }
  return line;
}
*/
