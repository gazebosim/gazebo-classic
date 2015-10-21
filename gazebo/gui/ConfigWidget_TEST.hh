/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_CONFIGWIDGET_TEST_HH_
#define _GAZEBO_CONFIGWIDGET_TEST_HH_

#include <string>

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the config widget.
class ConfigWidget_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Constructor.
  public: ConfigWidget_TEST() = default;

  /// \brief Test creating config widgets from empty messages.
  private slots: void EmptyMsgWidget();

  /// \brief Test creating a config widget from a joint message.
  private slots: void JointMsgWidget();

  /// \brief Test creating a config widget from a visual message.
  private slots: void VisualMsgWidget();

  /// \brief Test creating a config widget from a plugin message.
  private slots: void PluginMsgWidget();

  /// \brief Test setting visibility of a field in config widget.
  private slots: void ConfigWidgetVisible();

  /// \brief Test setting a field to be read-only in config widget.
  private slots: void ConfigWidgetReadOnly();

  /// \brief Test creating and updating a config widget without parsing
  /// messages.
  private slots: void CreatedExternally();

  /// \brief Test functions related to enum config widgets.
  private slots: void EnumConfigWidget();

  /// \brief Test getting a child widget by name.
  private slots: void GetChildWidgetByName();

  /// \brief Test receiving a signal from child uint widget.
  private slots: void ChildUIntSignal();

  /// \brief Test receiving a signal from child int widget.
  private slots: void ChildIntSignal();

  /// \brief Test receiving a signal from child double widget.
  private slots: void ChildDoubleSignal();

  /// \brief Test receiving a signal from child bool widget.
  private slots: void ChildBoolSignal();

  /// \brief Test receiving a signal from child string widget.
  private slots: void ChildStringSignal();

  /// \brief Test receiving a signal from child vector3 widget.
  private slots: void ChildVector3dSignal();

  /// \brief Test receiving a signal from child color widget.
  private slots: void ChildColorSignal();

  /// \brief Test receiving a signal from child pose widget.
  private slots: void ChildPoseSignal();

  /// \brief Test receiving a signal from child geometry widget.
  private slots: void ChildGeometrySignal();

  /// \brief Test receiving a signal from child enum widget.
  private slots: void ChildEnumSignal();

  /// \brief Slot that receives uint signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnUIntValueChanged(const QString &_name,
      const unsigned int _value);

  /// \brief Slot that receives int signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnIntValueChanged(const QString &_name,
      const int _value);

  /// \brief Slot that receives double signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnDoubleValueChanged(const QString &_name,
      const double _value);

  /// \brief Slot that receives bool signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnBoolValueChanged(const QString &_name,
      const bool _value);

  /// \brief Slot that receives string signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnStringValueChanged(const QString &_name,
      const std::string &_value);

  /// \brief Slot that receives vector3 signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnVector3dValueChanged(const QString &_name,
      const ignition::math::Vector3d &_value);

  /// \brief Slot that receives color signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnColorValueChanged(const QString &_name,
      const gazebo::common::Color &_value);

  /// \brief Slot that receives pose signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnPoseValueChanged(const QString &_name,
      const ignition::math::Pose3d &_value);

  /// \brief Slot that receives geometry signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New geometry value.
  /// \param[in] _dimensions New dimensions.
  /// \param[in] _uri New uri for meshes.
  private slots: void OnGeometryValueChanged(const std::string &_name,
      const std::string &_value, const ignition::math::Vector3d &_dimensions,
      const std::string &_uri);

  /// \brief Slot that receives enum signals from child widgets.
  /// \param[in] _name Scoped name of child widget which sent signal.
  /// \param[in] _value New value.
  private slots: void OnEnumValueChanged(const QString &_name,
      const QString &_value);

  /// \brief Check that uint has been received.
  private: bool g_uIntSignalReceived = false;

  /// \brief Check that int has been received.
  private: bool g_intSignalReceived = false;

  /// \brief Check that double has been received.
  private: bool g_doubleSignalReceived = false;

  /// \brief Check that bool has been received.
  private: bool g_boolSignalReceived = false;

  /// \brief Check that string has been received.
  private: bool g_stringSignalReceived = false;

  /// \brief Check how many times the vector3 signal has been received.
  private: int g_vector3SignalCount = 0;

  /// \brief Check that color has been received.
  private: bool g_colorSignalReceived = false;

  /// \brief Check that pose has been received.
  private: bool g_poseSignalReceived = false;

  /// \brief Check that geometry has been received.
  private: bool g_geometrySignalReceived = false;

  /// \brief Check that enum has been received.
  private: bool g_enumSignalReceived = false;
};

#endif
