/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef JOINT_CONTROL_WIDGET_HH
#define JOINT_CONTROL_WIDGET_HH

#include <string>
#include <map>
#include "msgs/msgs.h"
#include "gui/qt.h"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class MySlider;
    class JointControlWidget : public QWidget
    {
      Q_OBJECT
      public: JointControlWidget(const std::string &_model,
                                 QWidget *_parent = 0);
      public: virtual ~JointControlWidget();

      public: void Load(const std::string &_modelName);
      private slots: void OnChanged(double _value, const std::string &_name);

      private: transport::NodePtr node;
      private: transport::PublisherPtr jointPub;

      private: msgs::Request *requestMsg;
      private: std::map<std::string, MySlider*> sliders;
    };

    class MySlider : public QWidget
    {
      Q_OBJECT
      public: MySlider(const std::string &_name, QWidget *_parent);
      public slots: void OnChanged(int _value);
      Q_SIGNALS: void changed(double /*_value*/, const std::string & /*_name*/);
      private: std::string name;
      private: QSlider *slider;
      private: QLabel *label;
      private: QDoubleSpinBox *multiplier;
    };
  }
}

#endif
