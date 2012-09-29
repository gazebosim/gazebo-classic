/*
 * Copyright 2011 Nate Koenig
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
#include "msgs/msgs.hh"
#include "gui/qt.h"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class JointForceControl;
    class JointPIDPosControl;
    class JointPIDVelControl;

    class JointControlWidget : public QWidget
    {
      Q_OBJECT
      public: JointControlWidget(const std::string &_model,
                                 QWidget *_parent = 0);
      public: virtual ~JointControlWidget();

      public: void Load(const std::string &_modelName);

      private slots: void OnReset();
      private slots: void OnForceChanged(double _value,
                                         const std::string &_name);
      private slots: void OnPIDPosChanged(double _value,
          const std::string &_name);

      private slots: void OnPPosGainChanged(double _value,
                                         const std::string &_name);
      private slots: void OnDPosGainChanged(double _value,
                                         const std::string &_name);
      private slots: void OnIPosGainChanged(double _value,
                                         const std::string &_name);

      private slots: void OnPIDVelChanged(double _value,
          const std::string &_name);

      private slots: void OnPVelGainChanged(double _value,
                                         const std::string &_name);
      private slots: void OnDVelGainChanged(double _value,
                                         const std::string &_name);
      private slots: void OnIVelGainChanged(double _value,
                                         const std::string &_name);

      private slots: void OnPIDPosUnitsChanged(int _index);
      private: transport::NodePtr node;
      private: transport::PublisherPtr jointPub;

      private: msgs::Request *requestMsg;
      private: std::map<std::string, JointForceControl*> sliders;
      private: std::map<std::string, JointPIDPosControl*> pidPosSliders;
      private: std::map<std::string, JointPIDVelControl*> pidVelSliders;
    };

    class JointForceControl : public QWidget
    {
      Q_OBJECT
      public: JointForceControl(const std::string &_name,
                  QGridLayout *_layout, QWidget *_parent);
      public: void Reset();
      public slots: void OnChanged(double _value);
      Q_SIGNALS: void changed(double /*_value*/, const std::string & /*_name*/);
      private: std::string name;
      private: QDoubleSpinBox *forceSpin;
    };

    class JointPIDPosControl : public QWidget
    {
      Q_OBJECT
      public: JointPIDPosControl(const std::string &_name,
                  QGridLayout *_layout, QWidget *_parent);

      public: void Reset();
      public: void SetToRadians();
      public: void SetToDegrees();

      public slots: void OnChanged(double _value);
      public slots: void OnPChanged(double _value);
      public slots: void OnIChanged(double _value);
      public slots: void OnDChanged(double _value);

      Q_SIGNALS: void changed(double /*_value*/, const std::string &/*_name*/);
      Q_SIGNALS: void pChanged(double /*_value*/, const std::string &/*_name*/);
      Q_SIGNALS: void dChanged(double /*_value*/, const std::string &/*_name*/);
      Q_SIGNALS: void iChanged(double /*_value*/, const std::string &/*_name*/);

      private: QDoubleSpinBox *posSpin, *pGainSpin, *iGainSpin, *dGainSpin;
      private: std::string name;
      private: bool radians;
    };

    class JointPIDVelControl : public QWidget
    {
      Q_OBJECT
      public: JointPIDVelControl(const std::string &_name,
                  QGridLayout *_layout, QWidget *_parent);

      public: void Reset();
      public slots: void OnChanged(double _value);
      public slots: void OnPChanged(double _value);
      public slots: void OnIChanged(double _value);
      public slots: void OnDChanged(double _value);

      Q_SIGNALS: void changed(double /*_value*/, const std::string &/*_name*/);
      Q_SIGNALS: void pChanged(double /*_value*/, const std::string &/*_name*/);
      Q_SIGNALS: void dChanged(double /*_value*/, const std::string &/*_name*/);
      Q_SIGNALS: void iChanged(double /*_value*/, const std::string &/*_name*/);

      private: QDoubleSpinBox *posSpin, *pGainSpin, *iGainSpin, *dGainSpin;
      private: std::string name;
    };
  }
}

#endif
