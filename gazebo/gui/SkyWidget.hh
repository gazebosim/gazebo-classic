/*
 * Copyright 2012 Nate Koenig
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
#ifndef _SKY_WIDGET_HH_
#define _SKY_WIDGET_HH_

#include "gui/qt.h"
#include "gazebo/transport/TransportTypes.hh"

class QPushButton;

namespace gazebo
{
  namespace gui
  {
    class SkyWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: SkyWidget(QWidget *_parent = 0);

      /// \brief Destructor
      public: virtual ~SkyWidget();

      private slots: void OnTime(double _v);
      private slots: void OnSunrise(double _v);
      private slots: void OnSunset(double _v);
      private slots: void OnWindSpeed(double _v);
      private slots: void OnWindDirection(double _v);
      private slots: void OnHumidity(double _v);
      private slots: void OnAvgCloudSize(double _v);

      private: transport::NodePtr node;
      private: transport::PublisherPtr pub;

      private: QDoubleSpinBox *timeLineEdit;
      private: QDoubleSpinBox *sunriseLineEdit;
      private: QDoubleSpinBox *sunsetLineEdit;

      private: QDoubleSpinBox *windSpeedLineEdit;
      private: QDoubleSpinBox *windDirectionLineEdit;
      private: QPushButton *cloudAmbientColorButton;
      private: QDoubleSpinBox *humidityLineEdit;
      private: QDoubleSpinBox *avgCloudSizeLineEdit;
    };
  }
}
#endif
