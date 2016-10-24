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
#ifndef GAZEBO_GUI_JOINTCONTROLWIDGET_HH_
#define GAZEBO_GUI_JOINTCONTROLWIDGET_HH_

#include <memory>
#include <string>
#include "gazebo/gui/qt.h"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace msgs
  {
    class Model;
  }

  namespace gui
  {
    class JointControlWidgetPrivate;
    class JointForceControl;
    class JointForceControlPrivate;
    class JointPIDPosControl;
    class JointPIDPosControlPrivate;
    class JointPIDVelControl;
    class JointPIDVelControlPrivate;

    /// \class JointControlWidget JointControlWidget
    /// gui/JointControlWidget.hh
    /// \brief Widget to control joints via application of force, position
    /// PID controller, or velocity PID controller.
    class GZ_GUI_VISIBLE JointControlWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _parent Pointer to the parent of the widget.
      public: JointControlWidget(QWidget *_parent = 0);

      /// \brief Destructor.
      public: virtual ~JointControlWidget();

      /// \brief Set the name of the model to control.
      /// \param[in] _modelName Name of the model.
      public: void SetModelName(const std::string &_modelName);

      /// \brief On reset callback.
      private slots: void OnReset();

      /// \brief On force changed callback.
      /// \param[in] _value Value of the force.
      /// \param[in] _name Name of the joint.
      private slots: void OnForceChanged(double _value,
                                         const std::string &_name);

      /// \brief On position PID changed callback.
      /// \param[in] _value Value of the position.
      /// \param[in] _name Name of the joint.
      private slots: void OnPIDPosChanged(double _value,
                                          const std::string &_name);

      /// \brief On position PID P gain changed callback.
      /// \param[in] _value Value of the gain.
      /// \param[in] _name Name of the joint.
      private slots: void OnPPosGainChanged(double _value,
                                            const std::string &_name);

      /// \brief On position PID D gain changed callback.
      /// \param[in] _value Value of the gain.
      /// \param[in] _name Name of the joint.
      private slots: void OnDPosGainChanged(double _value,
                                            const std::string &_name);

      /// \brief On position PID I gain changed callback.
      /// \param[in] _value Value of the gain.
      /// \param[in] _name Name of the joint.
      private slots: void OnIPosGainChanged(double _value,
                                            const std::string &_name);

      /// \brief On velocity PID changed callback.
      /// \param[in] _value Value of the velocity.
      /// \param[in] _name Name of the joint.
      private slots: void OnPIDVelChanged(double _value,
                                          const std::string &_name);

      /// \brief On velocity PID P gain changed callback.
      /// \param[in] _value Value of the gain.
      /// \param[in] _name Name of the joint.
      private slots: void OnPVelGainChanged(double _value,
                                            const std::string &_name);

      /// \brief On velocity PID D gain changed callback.
      /// \param[in] _value Value of the gain.
      /// \param[in] _name Name of the joint.
      private slots: void OnDVelGainChanged(double _value,
                                            const std::string &_name);

      /// \brief On velocity PID I gain changed callback.
      /// \param[in] _value Value of the gain.
      /// \param[in] _name Name of the joint.
      private slots: void OnIVelGainChanged(double _value,
                                            const std::string &_name);

      /// \brief On position PID units changed callback.
      /// \param[in] _index Index into the drop-down menu.
      private slots: void OnPIDPosUnitsChanged(int _index);

      /// \brief Add a tab widget with a scrollable area.
      /// \param[in] _tabPane Tab pane that will have all the controls.
      /// \param[in] _tabLayout Layout for the controls in the tab.
      /// \param[in] _name Name of the tab.
      private: void AddScrollTab(QTabWidget *_tabPane,
                                 QGridLayout *_tabLayout,
                                 const QString &_name);

      /// \brief Layout the force tab.
      /// \param[in] _modelMsg Message used to create the joint controls
      private: void LayoutForceTab(msgs::Model &_modelMsg);

      /// \brief Layout the position PID tab.
      /// \param[in] _modelMsg Message used to create the joint controls
      private: void LayoutPositionTab(msgs::Model &_modelMsg);

      /// \brief Layout the velocity PID tab.
      /// \param[in] _modelMsg Message used to create the joint controls
      private: void LayoutVelocityTab(msgs::Model &_modelMsg);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<JointControlWidgetPrivate> dataPtr;
    };

    /// \class JointForceControl JointForceControl gui/JointForceControl.hh
    /// \brief Widget to control joints via application of force
    class GZ_GUI_VISIBLE JointForceControl : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _name Name of the joint.
      /// \param[in] _layout Layout to add the control to.
      /// \param[in] _parent Parent of the widget.
      /// \param[in] _index Row index into the grid layout.
      public: JointForceControl(const std::string &_name,
                  QGridLayout *_layout, QWidget *_parent, int _index);

      /// \brief Destructor.
      public: virtual ~JointForceControl();

      /// \brief Reset the controls.
      public: void Reset();

      /// \brief On force changed callback.
      /// \param[in] _value Value of the changed slider.
      public slots: void OnChanged(double _value);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void changed(double _value, const std::string &_name);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<JointForceControlPrivate> dataPtr;
    };

    /// \class JointPIDPosControl JointPIDPosControlgui/JointPIDPosControl.hh
    /// \brief Widget to control joints via application of position
    /// PID controller.
    class GZ_GUI_VISIBLE JointPIDPosControl : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _name Name of the joint.
      /// \param[in] _layout Layout to add the control to.
      /// \param[in] _parent Parent of the widget.
      /// \param[in] _index Row index into the grid layout.
      public: JointPIDPosControl(const std::string &_name,
                  QGridLayout *_layout, QWidget *_parent, int _index);

      /// \brief Destructor.
      public: virtual ~JointPIDPosControl();

      /// \brief Reset the controls.
      public: void Reset();

      /// \brief Set the units to radians.
      public: void SetToRadians();

      /// \brief Set the units to degrees.
      public: void SetToDegrees();

      /// \brief Callback when the value of position slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnChanged(double _value);

      /// \brief Callback when the value of P gain slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnPChanged(double _value);

      /// \brief Callback when the value of I gain slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnIChanged(double _value);

      /// \brief Callback when the value of D gain slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnDChanged(double _value);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void changed(double _value, const std::string &_name);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void pChanged(double _value, const std::string &_name);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void dChanged(double _value, const std::string &_name);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void iChanged(double _value, const std::string &_name);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<JointPIDPosControlPrivate> dataPtr;
    };

    /// \class JointPIDVelControl JointPIDVelControl gui/JointPIDVelControl.hh
    /// \brief Widget to control joints via application of a
    /// velocity PID controller.
    class GZ_GUI_VISIBLE JointPIDVelControl : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor.
      /// \param[in] _name Name of the joint.
      /// \param[in] _layout Layout to add the control to.
      /// \param[in] _parent Parent of the widget.
      /// \param[in] _index Row index into the grid layout.
      public: JointPIDVelControl(const std::string &_name,
                  QGridLayout *_layout, QWidget *_parent, int _index);

      /// \brief Destructor.
      public: virtual ~JointPIDVelControl();

      /// \brief Reset the controls.
      public: void Reset();

      /// \brief Callback when the value of velocity slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnChanged(double _value);

      /// \brief Callback when the value of P gain slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnPChanged(double _value);

      /// \brief Callback when the value of I gain slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnIChanged(double _value);

      /// \brief Callback when the value of D gain slider changed.
      /// \param[in] _value Value of the slider.
      public slots: void OnDChanged(double _value);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void changed(double _value, const std::string &_name);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void pChanged(double _value, const std::string &_name);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void dChanged(double _value, const std::string &_name);

      /// \brief QT changed signal, used to report the change to the joint
      /// controller widget.
      /// \param[in] _value Value of the slider.
      /// \param[in] _name Name of the joint.
      Q_SIGNALS: void iChanged(double _value, const std::string &_name);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<JointPIDVelControlPrivate> dataPtr;
    };
  }
}
#endif
