/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/viewers/TopicView.hh"

namespace gazebo
{
  namespace gui
  {
    class LaserView : public TopicView
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Pointer to the parent widget.
      public: LaserView(QWidget *_parent);

      /// \brief Destructor
      public: virtual ~LaserView();

      // Documentation inherited
      public: virtual void SetTopic(const std::string &_topicName);

      // Documentation inherited
      private: virtual void UpdateImpl();

      /// \brief Receives incoming laser scan messages.
      /// \param[in] _msg New laser scan message.
      private: void OnScan(ConstLaserScanStampedPtr &_msg);

      private: void wheelEvent(QWheelEvent *_event);

      private: class LaserItem : public QGraphicsItem
               {
                 public: LaserItem();

                 public: void ClearPoints();

                 public: unsigned int GetPointCount();

                 public: void AddRange(double _range);

                 public: void SetRange(unsigned int _index, double _range);

                 public: QRectF GetBoundingRect() const;

                 public: void Update();

                 private: virtual QRectF boundingRect() const;

                 private: virtual void paint (QPainter *_painter,
                              const QStyleOptionGraphicsItem *_option,
                              QWidget *_widget);

                 private: virtual void hoverEnterEvent(
                              QGraphicsSceneHoverEvent *_event);

                 private: virtual void hoverLeaveEvent(
                              QGraphicsSceneHoverEvent *_event);

                 private: void hoverMoveEvent(QGraphicsSceneHoverEvent *_event);

                 private: std::vector<QPointF> points;
                 private: std::vector<double> ranges;

                 public: double angleMin;
                 public: double angleMax;
                 public: double angleStep;

                 public: double rangeMax;

                 private: double indexAngle;

                 private: double scale;

                 /// \brief Mutex to protect the laser data.
                 private: boost::mutex mutex;
               };

      private: LaserItem *laserItem;
      private: QGraphicsView *view;
    };
  }
}
#endif
