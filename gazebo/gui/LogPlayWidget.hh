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
#ifndef _DATA_PLAYBACK_HH_
#define _DATA_PLAYBACK_HH_

#include <vector>
#include <list>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gui/qt.h"

class QLineEdit;
class QLabel;

namespace gazebo
{
  namespace gui
  {
    class LogPlayWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: LogPlayWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~LogPlayWidget();

      /// \brief A signal used to set the scrubber max range.
      /// \param[in] _max Maximum value for the scrubber.
      signals: void SetRange(unsigned int _max);

      /// \brief QT callback for the scrubber slider.
      /// \param[in] _value New value of the slider.
      private slots: void OnScrubber(int _value);

      /// \brief QT callback for setting the slider range.
      /// \param[in] _max Maximum value for the scrubber.
      private slots: void OnSetRange(unsigned int _max);

      /// \brief QT callback when slider is pressed.
      private slots: void OnScrubberPressed();

      /// \brief QT callback when slider is released.
      private slots: void OnScrubberReleased();

      /// \brief Callback for status messages.
      private: void OnStatusMsg(ConstLogPlayStatusPtr &_msg);

      /// \brief Slidder to scrub through a log file.
      private: QSlider *scrubber;

      /// \brief Node for communication.
      private: transport::NodePtr node;

      /// \brief Subscribs to status messages.
      private: transport::SubscriberPtr statusSub;

      /// \brief Publish log play control messages.
      private: transport::PublisherPtr controlPub;

      /// \brief True when the slider is pressed.
      private: bool sliderPressed;
    };
  }
}
#endif
