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

#ifndef _GAZEBO_CONFIG_WIDGET_HH_
#define _GAZEBO_CONFIG_WIDGET_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Color.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/gui/qt.h"

namespace google
{
  namespace protobuf
  {
    class Message;
    class Reflection;
    class FieldDescriptor;
  }
}

namespace gazebo
{
  namespace gui
  {
    class ConfigWidgetPrivate;
    class GroupWidget;

    /// \addtogroup gazebo_gui
    /// \{

    /// \class ConfigChildWidget ConfigWidget.hh
    /// \brief A convenience widget that also holds pointers to a list of its
    /// child widgets
    class GZ_GUI_VISIBLE ConfigChildWidget : public QFrame
    {
      Q_OBJECT

      /// \brief Constructor;
      public: ConfigChildWidget() : groupWidget(NULL) {}

      /// \brief Widget's key value.
      public: std::string key;

      /// \brief Widget's scoped name within parent config widget.
      public: std::string scopedName;

      /// \brief List of widgets holding values, such as Spins and LineEdits.
      public: std::vector<QWidget *> widgets;

      /// \brief Map a widget to the label holding its unit value.
      public: std::map<QWidget *, QLabel *> mapWidgetToUnit;

      /// \brief Pointer to group widget.
      /// NULL if this widget is not contained inside a group widget.
      public: GroupWidget *groupWidget;
    };

    /// \class GeometryConfigWidget ConfigWidget.hh
    /// \brief A widget for configuring geometry properties.
    class GZ_GUI_VISIBLE GeometryConfigWidget : public ConfigChildWidget
    {
      Q_OBJECT

      /// \brief A stacked widget containing widgets for configuring
      /// geometry dimensions.
      public: QStackedWidget *geomDimensionWidget;

      /// \brief A spin box for configuring the length of the geometry.
      public: QWidget *geomLengthSpinBox;

      /// \brief A label for the length widget.
      public: QWidget *geomLengthLabel;

      /// \brief A label for the unit of the length widget.
      public: QWidget *geomLengthUnitLabel;

      /// \brief A line edit for editing the mesh filename.
      public: QWidget *geomFilenameLineEdit;

      /// \brief A label for the mesh filename widget.
      public: QWidget *geomFilenameLabel;

      /// \brief A button for selecting the mesh filename.
      public: QWidget *geomFilenameButton;

      /// brief Callback when the geometry type is changed.
      /// \param[in] _text New geometry type in string.
      private slots: void GeometryChanged(const QString _text);

      /// brief Callback when the file button is clicked.
      private slots: void OnSelectFile();
    };

    /// \class EnumConfigWidget ConfigWidget.hh
    /// \brief A widget for configuring enum values.
    class GZ_GUI_VISIBLE EnumConfigWidget : public ConfigChildWidget
    {
      Q_OBJECT

      /// brief Signal an enum value change event.
      /// \param[in] _value New enum value in string.
      Q_SIGNALS: void EnumValueChanged(const QString &_value);

      /// brief Callback when the enum value is changed.
      /// \param[in] _value New enum value in string.
      private slots: void EnumChanged(const QString &_value);
    };

    /// \class GroupWidget ConfigWidget.hh
    /// \brief A collapsible widget that holds child widgets.
    class GZ_GUI_VISIBLE GroupWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Child widget that can be collapsed or expanded.
      public: QWidget *childWidget;

      /// \brief Callback that collapses or expands the child widget.
      /// _param[in] _checked Whether it is checked or not.
      private slots: void Toggle(bool _checked);
    };

    /// \class ConfigWidget ConfigWidget.hh
    /// \brief A widget generated from a google protobuf message.
    class GZ_GUI_VISIBLE ConfigWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: ConfigWidget();

      /// \brief Destructor
      public: ~ConfigWidget();

      /// \brief Load from a google protobuf message.
      /// \param[in] _msg Message to load from.
      public: void Load(const google::protobuf::Message *_msg);

      /// \brief Get the updated message.
      /// \return Updated message.
      /// \deprecated See Msg() const
      public: google::protobuf::Message *GetMsg();

      /// \brief Get the updated message.
      /// \return Updated message.
      public: google::protobuf::Message *Msg();

      /// \brief Create a human readable key, capitalizing the first letter
      /// and removing characters like "_".
      /// \param[in] _key Non-human-readable key.
      /// \return Human-redadable key.
      /// \deprecated See HumanReadableKey(const std::string &_key) const
      public: std::string GetHumanReadableKey(const std::string &_key)
          GAZEBO_DEPRECATED(7.0);

      /// \brief Create a human readable key, capitalizing the first letter
      /// and removing characters like "_".
      /// \param[in] _key Non-human-readable key.
      /// \return Human-redadable key.
      public: std::string HumanReadableKey(const std::string &_key) const;

      /// \brief Returns the unit for a given key. For example, the key "mass"
      /// returns "kg".
      /// \param[in] _key The key.
      /// \param[in] _jointType In case the field belongs to a joint, the
      /// joint's type.
      /// \return The unit.
      /// \deprecated See UnitFromKey(const std::string &_key,
      ///                             const std::string &_jointType = "") const
      public: std::string GetUnitFromKey(const std::string &_key,
          const std::string &_jointType = "") GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the unit for a given key. For example, the key "mass"
      /// returns "kg".
      /// \param[in] _key The key.
      /// \param[in] _jointType In case the field belongs to a joint, the
      /// joint's type.
      /// \return The unit.
      public: std::string UnitFromKey(const std::string &_key,
          const std::string &_jointType = "") const;

      /// \brief Returns the range for a given key. For example, the key
      /// "transparency" returns min == 0, max == 1.
      /// \param[in] _key The key.
      /// \param[out] _min The minimum value.
      /// \param[out] _max The maximum value.
      /// \deprecated See RangeFromKey(const std::string &_key,
      ///                              double &_min,
      ///                              double &_max) const
      public: void GetRangeFromKey(const std::string &_key,
          double &_min, double &_max) GAZEBO_DEPRECATED(7.0);

      /// \brief Returns the range for a given key. For example, the key
      /// "transparency" returns min == 0, max == 1.
      /// \param[in] _key The key.
      /// \param[out] _min The minimum value.
      /// \param[out] _max The maximum value.
      public: void RangeFromKey(const std::string &_key,
          double &_min, double &_max) const;

      /// \brief Set whether a child widget should be visible.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _visible True to set the widget to be visible.
      public: void SetWidgetVisible(const std::string &_name, bool _visible);

      /// \brief Get whether a child widget is visible.
      /// \param[in] _name Name of the child widget.
      /// \return True if the widget is visible.
      /// \deprecated See WidgetVisible(const std::string &_name) const
      public: bool GetWidgetVisible(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get whether a child widget is visible.
      /// \param[in] _name Name of the child widget.
      /// \return True if the widget is visible.
      public: bool WidgetVisible(const std::string &_name) const;

      /// \brief Set whether a child widget should be read-only.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _visible True to set the widget to be read-only.
      public: void SetWidgetReadOnly(const std::string &_name, bool _readOnly);

      /// \brief Get whether a child widget is read-only.
      /// \param[in] _name Name of the child widget.
      /// \return True if the widget is read-only.
      /// \deprecated See WidgetReadOnly(const std::string &_name) const
      public: bool GetWidgetReadOnly(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get whether a child widget is read-only.
      /// \param[in] _name Name of the child widget.
      /// \return True if the widget is read-only.
      public: bool WidgetReadOnly(const std::string &_name) const;

      /// \brief Update the widgets from a message.
      /// \param[in] _msg Message used for updating the widgets.
      public: void UpdateFromMsg(const google::protobuf::Message *_msg);

      /// \brief Set an integer value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetIntWidgetValue(const std::string &_name, int _value);

      /// \brief Set an unsigned integer value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetUIntWidgetValue(const std::string &_name, unsigned int
          _value);

      /// \brief Set a double value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetDoubleWidgetValue(const std::string &_name,
          double _value);

      /// \brief Set a bool value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetBoolWidgetValue(const std::string &_name, bool _value);

      /// \brief Set a string value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      public: bool SetStringWidgetValue(const std::string &_name,
          const std::string &_value);

      /// \brief Set a vector3 value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetVector3WidgetValue(const std::string &_name,
          const math::Vector3 &_value);

      /// \brief Set a color value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetColorWidgetValue(const std::string &_name,
          const common::Color &_value);

      /// \brief Set a pose value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetPoseWidgetValue(const std::string &_name,
          const math::Pose &_value);

      /// \brief Set a geometry value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Type of geometry.
      /// \param[in] _dimensions Dimensions of geometry.
      /// \return True if the value is set successfully.
      public: bool SetGeometryWidgetValue(const std::string &_name,
          const std::string &_value, const math::Vector3 &_dimensions,
          const std::string &_uri = "");

      /// \brief Set an enum value to a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the value is set successfully.
      public: bool SetEnumWidgetValue(const std::string &_name,
          const std::string &_value);

      /// \brief Add an item to a child enum widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _itemText Enum text value.
      /// \return True if the item is added successfully.
      public: bool AddItemEnumWidget(const std::string &_name,
          const std::string &_itemText);

      /// \brief Remove an item from a child enum widget.
      /// \param[in] _name Name of the child widget.
      /// \param[in] _itemText Text of the enum value.
      /// \return True if the item is removed successfully.
      public: bool RemoveItemEnumWidget(const std::string &_name,
          const std::string &_itemText);

      /// \brief Remove all items from a child enum widget.
      /// \param[in] _name Name of the child widget.
      /// \return True if successful.
      public: bool ClearEnumWidget(const std::string &_name);

      /// \brief Get an integer value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Integer value.
      /// \deprecated See IntWidgetValue(const std::string &_name) const
      public: int GetIntWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get an integer value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Integer value.
      public: int IntWidgetValue(const std::string &_name) const;

      /// \brief Get an unsigned integer value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Unsigned integer value.
      /// \deprecated See UIntWidgetValue(const std::string &_name) const
      public: unsigned int GetUIntWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get an unsigned integer value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Unsigned integer value.
      public: unsigned int UIntWidgetValue(const std::string &_name) const;

      /// \brief Get a double value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Double value.
      /// \deprecated See DoubleWidgetValue(const std::string &_name) const
      public: double GetDoubleWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get a double value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Double value.
      public: double DoubleWidgetValue(const std::string &_name) const;

      /// \brief Get a bool value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Bool value.
      /// \deprecated See BoolWidgetValue(const std::string &_name) const
      public: bool GetBoolWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get a bool value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Bool value.
      public: bool BoolWidgetValue(const std::string &_name) const;

      /// \brief Get a string value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return String value.
      /// \deprecated See StringWidgetValue(const std::string &_name) const
      public: std::string GetStringWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get a string value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return String value.
      public: std::string StringWidgetValue(const std::string &_name) const;

      /// \brief Get a vector3 value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Vector3 value.
      /// \deprecated See Vector3WidgetValue(const std::string &_name) const
      public: math::Vector3 GetVector3WidgetValue(const std::string &_name)
          const GAZEBO_DEPRECATED(7.0);

      /// \brief Get a vector3 value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Vector3 value.
      public: ignition::math::Vector3d Vector3WidgetValue(
          const std::string &_name) const;

      /// \brief Get a color value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Color value.
      /// \deprecated See ColorWidgetValue(const std::string &_name) const
      public: common::Color GetColorWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get a color value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Color value.
      public: common::Color ColorWidgetValue(const std::string &_name) const;

      /// \brief Get a pose value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Pose value.
      /// \deprecated See PoseWidgetValue(const std::string &_name) const
      public: math::Pose GetPoseWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get a pose value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Pose value.
      public: ignition::math::Pose3d PoseWidgetValue(
          const std::string &_name) const;

      /// \brief Get a geometry value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[out] _dimensions Dimensions of geometry.
      /// \param[out] _uri URI of the geometry mesh, if any.
      /// \return Type of geometry.
      /// \deprecated See GeometryWidgetValue() function that accepts an
      /// ignition math object.
      public: std::string GetGeometryWidgetValue(const std::string &_name,
          math::Vector3 &_dimensions, std::string &_uri) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get a geometry value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \param[out] _dimensions Dimensions of geometry.
      /// \param[out] _uri URI of the geometry mesh, if any.
      /// \return Type of geometry.
      public: std::string GeometryWidgetValue(const std::string &_name,
          ignition::math::Vector3d &_dimensions, std::string &_uri) const;

      /// \brief Get an enum value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Enum value.
      /// \deprecated See EnumWidgetValue(const std::string &_name)
      public: std::string GetEnumWidgetValue(const std::string &_name) const
          GAZEBO_DEPRECATED(7.0);

      /// \brief Get an enum value from a child widget.
      /// \param[in] _name Name of the child widget.
      /// \return Enum value.
      public: std::string EnumWidgetValue(const std::string &_name) const;

      /// \brief Create a widget which has a button header which collapses
      /// the field widget.
      /// \param[in] _name Header name.
      /// \param[out] _childWidget Widget which will be collapsed.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The group widget.
      public: GroupWidget *CreateGroupWidget(const std::string &_name,
          ConfigChildWidget *_childWidget, const int _level = 0);

      /// \brief Create a widget for configuring an unsigned integer value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateUIntWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring an integer value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateIntWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring a double value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateDoubleWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring a string value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \param[in] _type Type of string widget, such as "line" or "plain".
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateStringWidget(const std::string &_key,
          const int _level = 0, const std::string &_type = "line");

      /// \brief Create a widget for configuring a bool value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateBoolWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring a vector3 value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateVector3dWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring a color value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateColorWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring a pose value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreatePoseWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring a geometry value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateGeometryWidget(const std::string &_key,
          const int _level = 0);

      /// \brief Create a widget for configuring an enum value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \param[in] _values A list of enum values in string.
      /// \param[in] _level Level of the widget in the tree.
      /// \return The newly created widget.
      public: ConfigChildWidget *CreateEnumWidget(const std::string &_key,
          const std::vector<std::string> &_values, const int _level = 0);

      /// \brief Register a child widget as a child of this widget, so it can
      /// be updated. Note that the widget is not automatically added to a
      /// layout.
      /// \param[in] _name Unique name to indentify the child within this widget
      /// \param[in] _child Child widget to be added. It doesn't need to be a
      /// ConfigChildWidget.
      /// \return True if child successfully added.
      public: bool AddConfigChildWidget(const std::string &_name,
          ConfigChildWidget *_child);

      /// \brief Insert a layout into the config widget's layout at a specific
      /// position.
      /// \param[in] _layout The layout to be inserted.
      /// \param[in] _pos The position to insert at, 0 being the top.
      public: void InsertLayout(QLayout *_layout, int _pos);

      /// \brief Get a config child widget by its name.
      /// \param[in] _name Scoped name of the child widget.
      /// \return The child widget with the given name or NULL if it wasn't
      /// found.
      public: ConfigChildWidget *ConfigChildWidgetByName(
          const std::string &_name) const;

      /// \brief Get the number of child widgets.
      /// \return The number of child widgets.
      public: unsigned int ConfigChildWidgetCount() const;

      /// \brief Get a style sheet in string format, to be applied to a child
      /// config widget with setStyleSheet.
      /// \param[in] _type Type of style sheet, such as "warning", "active",
      /// "normal".
      /// \param[in] _level Level of widget in the tree.
      /// \return Style sheet as string. Returns an empty string if _type is
      /// unknown.
      public: static QString StyleSheet(const std::string &_type,
          const int _level = 0);

      /// \brief List of colors used for the background of widgets according to
      /// their level.
      public: static const std::vector<QString> bgColors;

      /// \brief List of colors used for widget areas according to their level.
      public: static const std::vector<QString> widgetColors;

      /// \brief Red color used for "red" or "x" fields.
      public: static const QString redColor;

      /// \brief Green color used for "green" or "y" fields.
      public: static const QString greenColor;

      /// \brief Blue color used for "blue" or "z" fields.
      public: static const QString blueColor;

      /// \brief Parse the input message and either create widgets for
      /// configuring fields of the message, or update the widgets with values
      /// from the message.
      /// \param[in] _msg Message.
      /// \param[in] _update True to parse only fields that are specified in
      /// the message rather than all the available fields in the message
      /// \param[in] _name Name used when creating new widgets.
      /// \param[in] _level Level of the widget in the tree.
      /// return Updated widget.
      private: QWidget *Parse(google::protobuf::Message *_msg,
          bool _update = false, const std::string &_name = "",
          const int _level = 0);

      /// \brief Parse a vector3 message.
      /// param[in] _msg Input vector3d message.
      /// return Parsed vector.
      private: math::Vector3 ParseVector3(
          const google::protobuf::Message *_msg) const;

      /// \brief Update the message field using values from the widgets.
      /// \param[in] _msg Message to be updated.
      /// \param[in] _name Name of parent widget.
      private: void UpdateMsg(google::protobuf::Message *_msg,
          const std::string &_name = "");

      /// \brief Update a vector3d message.
      /// \param[in] _msg Vector3d message to be updated.
      /// \param[in] _value Vector3 used for updating the message.
      private: void UpdateVector3Msg(google::protobuf::Message *_msg,
          const math::Vector3 &_value);

      /// \brief Update a child widget with an unsigned integer value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateUIntWidget(ConfigChildWidget *_widget,
          const unsigned int _value);

      /// \brief Update a child widget with an integer value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateIntWidget(ConfigChildWidget *_widget,
           const int _value);

      /// \brief Update a child widget with a double value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateDoubleWidget(ConfigChildWidget *_widget,
          const double _value);

      /// \brief Update a child widget with a string value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateStringWidget(ConfigChildWidget *_widget,
          const std::string &_value);

      /// \brief Update a child widget with a bool value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateBoolWidget(ConfigChildWidget *_widget,
          const bool _value);

      /// \brief Update a child widget with a vector3 value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateVector3Widget(ConfigChildWidget *_widget,
          const math::Vector3 &_value);

      /// \brief Update a child widget with a color value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateColorWidget(ConfigChildWidget *_widget,
          const common::Color &_value);

      /// \brief Update a child widget with a pose value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdatePoseWidget(ConfigChildWidget *_widget,
          const math::Pose &_value);

      /// \brief Update a child widget with a geometry type and dimensions.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Type of geometry.
      /// \param[in] _dimensions Dimensions of the geometry.
      /// \param[in] _uri URI of the geometry mesh, if any.
      /// \return True if the update completed successfully.
      private: bool UpdateGeometryWidget(ConfigChildWidget *_widget,
          const std::string &_value, const math::Vector3 &_dimensions,
          const std::string &_uri = "");

      /// \brief Update a child widget with an enum value.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[in] _value Value to set to.
      /// \return True if the update completed successfully.
      private: bool UpdateEnumWidget(ConfigChildWidget *_widget,
          const std::string &_value);

      /// \brief Get an integer value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: int IntWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Get an unsigned integer value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: unsigned int UIntWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Get a double value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: double DoubleWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Get a bool value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: bool BoolWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Get a string value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: std::string StringWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Get a vector3 value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: ignition::math::Vector3d Vector3WidgetValue(
          ConfigChildWidget *_widget) const;

      /// \brief Get a color value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: common::Color ColorWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Get a pose value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: ignition::math::Pose3d PoseWidgetValue(
          ConfigChildWidget *_widget) const;

      /// \brief Get a geometry value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \param[out] _dimensions Dimensions of geometry.
      /// \param[out] _uri URI of the geometry mesh, if any.
      /// \return Type of geometry.
      private: std::string GeometryWidgetValue(ConfigChildWidget *_widget,
          ignition::math::Vector3d &_dimensions, std::string &_uri) const;

      /// \brief Get an enum value from a child widget.
      /// \param[in] _widget Pointer to the child widget.
      /// \return Value of the widget.
      private: std::string EnumWidgetValue(ConfigChildWidget *_widget) const;

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item,
          const int _column);

      /// \brief Callback when a uint config widget's value has changed.
      private slots: void OnUIntValueChanged();

      /// \brief Callback when an int config widget's value has changed.
      private slots: void OnIntValueChanged();

      /// \brief Callback when a double config widget's value has changed.
      private slots: void OnDoubleValueChanged();

      /// \brief Callback when a bool config widget's value has changed.
      private slots: void OnBoolValueChanged();

      /// \brief Callback when a string config widget's value has changed.
      private slots: void OnStringValueChanged();

      /// \brief Callback when a vector3 config widget's value has changed.
      private slots: void OnVector3dValueChanged();

      /// \brief Callback when a vector3 config widget's preset has changed.
      /// \param[in] _index Index of the chosen preset.
      private slots: void OnVector3dPresetChanged(const int _index);

      /// \brief Callback when a color config widget's value has changed.
      private slots: void OnColorValueChanged();

      /// \brief Callback when a pose config widget's value has changed.
      private slots: void OnPoseValueChanged();

      /// \brief Callback when a geometry config widget's value has changed.
      private slots: void OnGeometryValueChanged();

      /// \brief Callback when a geometry config widget's value has changed.
      /// \param[in] _value Value which the QComboBox changed to.
      private slots: void OnGeometryValueChanged(const int _value);

      /// \brief Callback when an enum config widget's enum value has changed.
      /// \param[in] _value New enum value in string.
      private slots: void OnEnumValueChanged(const QString &_value);

      /// \brief Signal that a uint config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New uint.
      Q_SIGNALS: void UIntValueChanged(const QString &_name,
          const unsigned int _value);

      /// \brief Signal that an int config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New int.
      Q_SIGNALS: void IntValueChanged(const QString &_name, const int _value);

      /// \brief Signal that a double config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New double.
      Q_SIGNALS: void DoubleValueChanged(const QString &_name,
          const double _value);

      /// \brief Signal that a bool config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New bool.
      Q_SIGNALS: void BoolValueChanged(const QString &_name,
          const bool _value);

      /// \brief Signal that a string config widget's value has changed.
      /// Note that only single line widgets will emit signals, so plain
      /// text widgets don't emit signals.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New string.
      Q_SIGNALS: void StringValueChanged(const QString &_name,
          const std::string &_value);

      /// \brief Signal that a vector3 config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New vector3.
      Q_SIGNALS: void Vector3dValueChanged(const QString &_name,
          const ignition::math::Vector3d &_value);

      /// \brief Signal that a color config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New color.
      Q_SIGNALS: void ColorValueChanged(const QString &_name,
          const gazebo::common::Color &_value);

      /// \brief Signal that a pose config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _pose New pose.
      Q_SIGNALS: void PoseValueChanged(const QString &_name,
          const ignition::math::Pose3d &_pose);

      /// \brief Signal that a geometry config widget's value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New geometry name, such as "box".
      /// \param[in] _dimensions New dimensions.
      /// \param[in] _uri New uri, for meshes.
      Q_SIGNALS: void GeometryValueChanged(const std::string &_name,
          const std::string &_value,
          const ignition::math::Vector3d &_dimensions,
          const std::string &_uri);

      /// \brief Signal that an enum config widget's enum value has changed.
      /// \param[in] _name Scoped name of widget.
      /// \param[in] _value New enum value string.
      Q_SIGNALS: void EnumValueChanged(const QString &_name,
          const QString &_value);

      /// \brief Qt event filter currently used to filter mouse wheel events.
      /// \param[in] _obj Object that is watched by the event filter.
      /// \param[in] _event Qt event.
      /// \return True if the event is handled.
      private: bool eventFilter(QObject *_obj, QEvent *_event);

      /// \internal
      /// \brief Pointer to private data.
      protected: std::unique_ptr<ConfigWidgetPrivate> dataPtr;
    };
  }
}
#endif
