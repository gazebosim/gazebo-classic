/*
 * Copyright 2014 Open Source Robotics Foundation
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

#ifndef _CONFIG_WIDGET_HH_
#define _CONFIG_WIDGET_HH_

#include <string>
#include <vector>
#include <map>

#include "gazebo/math/Pose.hh"
#include "gazebo/common/Color.hh"
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
    /// \addtogroup gazebo_gui
    /// \{

    /// \class ConfigChildWidget ConfigWidget.hh
    /// \brief A convenience widget that also holds pointers to a list of its
    /// child widgets
    class GAZEBO_VISIBLE ConfigChildWidget : public QWidget
    {
      Q_OBJECT

      /// \brief List of child widgets.
      public: std::vector<QWidget *> widgets;
    };

    /// \class GeometryConfigWidget ConfigWidget.hh
    /// \brief A widget for configuring geometry properties.
    class GAZEBO_VISIBLE GeometryConfigWidget : public ConfigChildWidget
    {
      Q_OBJECT

      /// \brief A stacked widget containing widgets for configuring
      /// geometry dimensions.
      public: QStackedWidget *geomDimensionWidget;

      /// \brief A spin box for configuring the length of the geometry.
      public: QWidget *geomLengthSpinBox;

      /// \brief A label for the length widget.
      public: QWidget *geomLengthLabel;

      /// brief Callback when the geometry type is changed.
      /// \param[in] _text New geometry type in string.
      private slots: void GeometryChanged(const QString _text);
    };

    /// \class GroupWidget ConfigWidget.hh
    /// \brief A collapsible widget that holds child widgets.
    class GAZEBO_VISIBLE GroupWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Child widget that can be collapsed or expanded.
      public: QWidget *childWidget;

      /// \brief Callback that collapses or expands the child widget.
      private slots: void Toggle();
    };

    /// \class ConfigWidget ConfigWidget.hh
    /// \brief A widget generated from a google protobuf message.
    class GAZEBO_VISIBLE ConfigWidget : public QWidget
    {
      Q_OBJECT

      /// \brief Constructor
      public: ConfigWidget();

      /// \brief Destructor
      public: ~ConfigWidget();

      /// \brief Load from a google protobuf message
      /// \param[in] _msg Message to load from
      public: void Load(const google::protobuf::Message *_msg);

      /// \brief Get the updated message.
      /// \return Updated message.
      public: google::protobuf::Message *GetMsg();

      /// \brief Update the widgets from a message.
      /// \param[in] _msg Message used for updating the widgets.
      public: void UpdateFromMsg(const google::protobuf::Message *_msg);

      /// \brief Set an integer value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetIntWidgetProperty(const std::string &_name, int _value);

      /// \brief Set an unsigned integer value to a child wiget in the
      /// config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetUIntWidgetProperty(const std::string &_name, unsigned int
          _value);

      /// \brief Set a double value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetDoubleWidgetProperty(const std::string &_name,
          double _value);

      /// \brief Set a bool value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetBoolWidgetProperty(const std::string &_name, bool _value);

      /// \brief Set a string value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetStringWidgetProperty(const std::string &_name,
          const std::string &_value);

      /// \brief Set a vector3 value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetVector3WidgetProperty(const std::string &_name,
          const math::Vector3 &_value);

      /// \brief Set a color value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetColorWidgetProperty(const std::string &_name,
          const common::Color &_value);

      /// \brief Set a pose value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Value to set to.
      public: void SetPoseWidgetProperty(const std::string &_name,
          const math::Pose &_value);

      /// \brief Set a geometry value to a child wiget in the config widget.
      /// \param[in] _name Name of the widget.
      /// \param[in] _value Type of geometry.
      /// \param[in] _dimensions Dimensions of geometry.
      public: void SetGeometryWidgetProperty(const std::string &_name,
          const std::string &_value, const math::Vector3 &_dimensions);

      /// \brief Parse the input message and either create widgets for
      /// configuring fields of the message, or update the widgets with values
      /// from the message.
      /// \param[in] _name Name used when creating new widgets.
      private: QWidget *Parse(google::protobuf::Message *_msg,
          const std::string &_name = "");

      /// \brief Parse a vector3 message.
      /// param[in] _msg Input vector3d message.
      /// return Parsed vector
      private: math::Vector3 ParseVector3(
          const google::protobuf::Message *_msg);

      /// \brief Update the message field using values from the widgets.
      /// \param[in] _msg Message to be updated.
      /// \param[in] _name Name of parent widget.
      private: void UpdateMsg(google::protobuf::Message *_msg,
          const std::string &_name = "");

      /// \brief Update a vector3d message.
      /// \param[in] _msg Vector3d message to be updated.
      /// \param[in] _value Vector3 used for updating the message.
      private: void UpdateVector3Msg(google::protobuf::Message *_msg,
          const math::Vector3 _value);

      /// \brief Create a widget for configuring an unsigned integer value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateUIntWidget(const std::string &_key);

      /// \brief Create a widget for configuring an integer value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateIntWidget(const std::string &_key);

      /// \brief Create a widget for configuring a double value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateDoubleWidget(const std::string &_key);

      /// \brief Create a widget for configuring a string value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateStringWidget(const std::string &_key);

      /// \brief Create a widget for configuring a bool value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateBoolWidget(const std::string &_key);

      /// \brief Create a widget for configuring a vector3 value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateVector3dWidget(const std::string &_key);

      /// \brief Create a widget for configuring a color value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateColorWidget(const std::string &_key);

      /// \brief Create a widget for configuring a pose value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreatePoseWidget(const std::string &_key);

      /// \brief Create a widget for configuring a geometry value.
      /// \param[in] _key A key that is used as a label for the widget.
      /// \return The newly created widget.
      private: ConfigChildWidget *CreateGeometryWidget(const std::string &_key);

      /// \brief Update the input widget with an unsigned integer value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateUIntWidget(ConfigChildWidget *_widget,
          unsigned int _value);

      /// \brief Update the input widget with an integer value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateIntWidget(ConfigChildWidget *_widget, int _value);

      /// \brief Update the input widget with a double value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateDoubleWidget(ConfigChildWidget *_widget,
          double _value);

      /// \brief Update the input widget with a string value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateStringWidget(ConfigChildWidget *_widget,
          const std::string &_value);

      /// \brief Update the input widget with a bool value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateBoolWidget(ConfigChildWidget *_widget, bool _value);

      /// \brief Update the input widget with a vector3 value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateVector3Widget(ConfigChildWidget *_widget,
          const math::Vector3 &_value);

      /// \brief Update the input widget with a color value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdateColorWidget(ConfigChildWidget *_widget,
          const common::Color &_value);

      /// \brief Update the input widget with a pose value.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value to set to.
      private: void UpdatePoseWidget(ConfigChildWidget *_widget,
          const math::Pose &_value);

      /// \brief Update the input widget with a geometry type and dimensions.
      /// \param[in] _widget Pointer to the widget.
      /// \param[in] _value Value Type of geometry
      /// \param[in] _dimensions Dimensions of the geometry.
      private: void UpdateGeometryWidget(ConfigChildWidget *_widget,
          const std::string &_value, const math::Vector3 &_dimensions);

      /// \brief Received item selection user input.
      /// \param[in] _item Item selected.
      /// \param[in] _column Column index.
      private slots: void OnItemSelection(QTreeWidgetItem *_item, int _column);

      /// \brief A map of unique scoped names to correpsonding widgets.
      private: std::map <std::string, ConfigChildWidget *> configWidgets;

      /// \brief A copy of the message with fields to be configured by widgets.
      private: google::protobuf::Message *configMsg;
    };
  }
}
#endif
