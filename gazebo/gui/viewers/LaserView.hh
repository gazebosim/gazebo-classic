/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _LASERVIEW_HH_
#define _LASERVIEW_HH_

#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/viewers/TopicView.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    class GZ_GUI_VISIBLE LaserView : public TopicView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to the parent widget.
      public: LaserView(QWidget *_parent = NULL);

      /// \brief Destructor
      public: virtual ~LaserView();

      // Documentation inherited
      public: virtual void SetTopic(const std::string &_topicName);

      // Documentation inherited
      private: virtual void UpdateImpl();

      /// \brief QT event, called when view is resized.
      /// \param[in] _event Pointer to the resize event info.
      protected: virtual void resizeEvent(QResizeEvent *_event);

      /// \brief Receives incoming laser scan messages.
      /// \param[in] _msg New laser scan message.
      private: void OnScan(ConstLaserScanStampedPtr &_msg);

      /// \brief QT callback. Used when the "fit in view" button is pressed.
      private slots: void OnFitInView();

      /// \brief QT callback. Used when the "degrees" button is pressed.
      private slots: void OnDegree(bool _toggled);

      /// \brief Class that draws the laser ranges.
      private: class LaserItem : public QGraphicsItem
               {
                 /// \brief Constructor
                 public: LaserItem();

                 /// \brief Clear all the stored ranges.
                 public: void Clear();

                 /// \brief Get the number of range values.
                 /// \return The size of the ranges vector.
                 public: unsigned int GetRangeCount();

                 /// \brief Add a new range value.
                 /// \param[in] _range The new range value.
                 public: void AddRange(double _range);

                 /// \brief Set a specific range value.
                 /// \param[in] _index Index of the range to set.
                 /// \param[in] _range The value of the range.
                 public: void SetRange(unsigned int _index, double _range);

                 /// \brief Return the bounding rectangle for this item.
                 public: QRectF GetBoundingRect() const;

                 /// \brief Get the range of the ray selected by the mouse
                 /// position.
                 /// \return Range reading for ray under mouse
                 public: double GetHoverRange() const;

                 /// \brief Get the angle of the ray selected by the mouse
                 /// position.
                 /// \return Angle reading for ray under mouse
                 public: double GetHoverAngle() const;

                 /// \brief Update the list of points to draw.
                 /// \param[in] _angleMin Minimum angle, in radians.
                 /// \param[in] _angleMax Maximum angle, in radians.
                 /// \param[in] _angleStep Angle step size, in radians.
                 /// \param[in] _rangeMax Maximum range in meters.
                 /// \param[in] _rangeMin Minimum range in meters.
                 public: void Update(double _angleMin, double _angleMax,
                             double _angleStep, double _rangeMax,
                             double _rangeMin);

                 /// \brief A QT pure virtual function that must be defined.
                 /// This calls GetBoundingRect.
                 private: virtual QRectF boundingRect() const;

                 /// \brief A QT function that is called when the item
                 /// should be redrawn.
                 /// \param[in] _painter Pointer to the QPainter, which
                 /// draws shapes in the scene.
                 /// \param[in] _option An unsued graphics options.
                 /// \param[in] _widget Option param, the widget to paint
                 /// on.
                 private: virtual void paint(QPainter *_painter,
                              const QStyleOptionGraphicsItem *_option,
                              QWidget *_widget);

                 /// \brief QT event that is called when the mouse starts to
                 /// hover over this item.
                 /// \param[in] _event The hover event info.
                 private: virtual void hoverEnterEvent(
                              QGraphicsSceneHoverEvent *_event);

                 /// \brief QT event that is called when the mouse leaves
                 /// this item.
                 /// \param[in] _event The hover event info.
                 private: virtual void hoverLeaveEvent(
                              QGraphicsSceneHoverEvent *_event);

                 /// \brief QT event that is called when the mouse moves
                 /// over this item.
                 /// \param[in] _event The hover event info.
                 private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

                 /// \brief The vector of points to draw.
                 private: std::vector<QPointF> points;

                 /// \brief The vector of range values, as returned by the
                 /// laser sensor.
                 private: std::vector<double> ranges;

                 /// \brief The minimum angle of the laser.
                 private: double angleMin;

                 /// \brief The maximum angle of the laser.
                 private: double angleMax;

                 /// \brief The size of the step between rays, in radians.
                 private: double angleStep;

                 /// \brief The maximum range value.
                 private: double rangeMax;

                 /// \brief The minimum range value.
                 private: double rangeMin;

                 /// \brief Angle that is highlighted by the mouse position.
                 private: double indexAngle;

                 /// \brief Scaling factor that is applied to range data.
                 private: double scale;

                 /// \brief True to draw the hover angles in radians
                 public: bool radians;

                 /// \brief Mutex to protect the laser data.
                 private: mutable boost::mutex mutex;
               };

      /// \brief This class exists so that we can properly capture the
      /// QT wheelEvent.
      private: class CustomView : public QGraphicsView
               {
                 /// \brief Constructor
                 /// \param[in] _parent Pointer to the parent widget.
                 public: CustomView(QWidget *_parent)
                         : QGraphicsView(_parent), viewZoomed(false) {}

                 /// \brief QT callback. Used when a wheel event occurs.
                 /// \param[in] _event QT wheel event info.
                 private: void wheelEvent(QWheelEvent *_event)
                 {
                   this->viewZoomed = true;
                   _event->delta() > 0 ? this->scale(1.15, 1.15) :
                                         this->scale(1.0 / 1.15, 1.0 / 1.15);
                   _event->accept();
                 }

                 /// \brief True if the view has been zoomed in/out by the user.
                 public: bool viewZoomed;
               };

      /// \brief The item that draws the laser data.
      private: LaserItem *laserItem;

      /// \brief The view that displays the laser item.
      private: CustomView *view;

      /// \brief Flag used to determine if a recieved message is the first.
      private: bool firstMsg;

      /// \brief Range output
      private: QLineEdit *rangeEdit;

      /// \brief Angle output
      private: QLineEdit *angleEdit;

      /// \brief Spin box for choosing vertical ray.
      private: QSpinBox *vertScanSpin;
    };
  }
}
#endif
