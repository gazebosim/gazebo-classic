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

#include <math.h>
#include <float.h>

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>

#include "gazebo/common/Console.hh"
#include "gazebo/gui/model/DensityModel.hh"
#include "gazebo/gui/model/CollisionConfig.hh"
#include "gazebo/gui/ConfigWidget.hh"

using namespace gazebo;
using namespace gui;

const std::vector<QString> ConfigWidget::bgColors(
      {"#999999", "#777777", "#555555", "#333333"});

const std::vector<QString> ConfigWidget::widgetColors(
      {"#eeeeee", "#cccccc", "#aaaaaa", "#888888"});

const QString ConfigWidget::redColor = "#d42b2b";
const QString ConfigWidget::greenColor = "#3bc43b";
const QString ConfigWidget::blueColor = "#0d0df2";

/////////////////////////////////////////////////
ConfigWidget::ConfigWidget()
{
  this->configMsg = NULL;
  this->setObjectName("configWidget");
}

/////////////////////////////////////////////////
ConfigWidget::~ConfigWidget()
{
  delete this->configMsg;
}

/////////////////////////////////////////////////
void ConfigWidget::Load(const google::protobuf::Message *_msg)
{
  this->configMsg = _msg->New();
  this->configMsg->CopyFrom(*_msg);

  QWidget *widget = this->Parse(this->configMsg, 0);
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setAlignment(Qt::AlignTop);
  mainLayout->addWidget(widget);

  this->setLayout(mainLayout);

  // set up event filter for scrollable widgets to make sure they don't steal
  // focus when embedded in a QScrollArea.
  QList<QAbstractSpinBox *> spinBoxes =
      this->findChildren<QAbstractSpinBox *>();
  for (int i = 0; i < spinBoxes.size(); ++i)
  {
    spinBoxes[i]->installEventFilter(this);
    spinBoxes[i]->setFocusPolicy(Qt::StrongFocus);
  }
  QList<QComboBox *> comboBoxes =
      this->findChildren<QComboBox *>();
  for (int i = 0; i < comboBoxes.size(); ++i)
  {
    comboBoxes[i]->installEventFilter(this);
    comboBoxes[i]->setFocusPolicy(Qt::StrongFocus);
  }
}

/////////////////////////////////////////////////
void ConfigWidget::UpdateFromMsg(const google::protobuf::Message *_msg)
{
  this->configMsg->CopyFrom(*_msg);
  this->Parse(this->configMsg, true);
}

/////////////////////////////////////////////////
google::protobuf::Message *ConfigWidget::GetMsg()
{
  this->UpdateMsg(this->configMsg);
  return this->configMsg;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetHumanReadableKey(const std::string &_key)
{
  std::string humanKey = _key;
  humanKey[0] = std::toupper(humanKey[0]);
  std::replace(humanKey.begin(), humanKey.end(), '_', ' ');
  return humanKey;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetUnitFromKey(const std::string &_key,
    const std::string &_jointType)
{
  if (_key == "pos" || _key == "length" || _key == "min_depth")
  {
    return "m";
  }

  if (_key == "rot")
    return "rad";

  if (_key == "kp" || _key == "kd")
    return "N/m";

  if (_key == "max_vel")
    return "m/s";

  if (_key == "mass")
    return "kg";

  if (_key == "ixx" || _key == "ixy" || _key == "ixz" ||
      _key == "iyy" || _key == "iyz" || _key == "izz")
  {
    return "kg&middot;m<sup>2</sup>";
  }

  if (_key == "limit_lower" || _key == "limit_upper")
  {
    if (_jointType == "PRISMATIC")
      return "m";
    else if (_jointType != "")
      return "rad";
  }

  if (_key == "limit_effort")
  {
    if (_jointType == "PRISMATIC")
      return "N";
    else if (_jointType != "")
      return "Nm";
  }

  if (_key == "limit_velocity" || _key == "velocity")
  {
    if (_jointType == "PRISMATIC")
      return "m/s";
    else if (_jointType != "")
      return "rad/s";
  }

  if (_key == "damping")
  {
    if (_jointType == "PRISMATIC")
      return "Ns/m";
    else if (_jointType != "")
      return "Ns";
  }

  if (_key == "friction")
  {
    if (_jointType == "PRISMATIC")
      return "N";
    else if (_jointType != "")
      return "Nm";
  }

  if (_key == "density")
  {
    return "kg/m<sup>3</sup>";
  }

  return "";
}

/////////////////////////////////////////////////
void ConfigWidget::GetRangeFromKey(const std::string &_key, double &_min,
    double &_max)
{
  // Maximum range by default
  _min = -GZ_DBL_MAX;
  _max = GZ_DBL_MAX;

  if (_key == "mass" || _key == "ixx" || _key == "ixy" || _key == "ixz" ||
      _key == "iyy" || _key == "iyz" || _key == "izz" || _key == "length" ||
      _key == "min_depth" || _key == "density")
  {
    _min = 0;
  }
  else if (_key == "bounce" || _key == "transparency" ||
      _key == "laser_retro" || _key == "ambient" || _key == "diffuse" ||
      _key == "specular" || _key == "emissive" ||
      _key == "restitution_coefficient")
  {
    _min = 0;
    _max = 1;
  }
  else if (_key == "fdir1" || _key == "xyz")
  {
    _min = -1;
    _max = +1;
  }
}

/////////////////////////////////////////////////
bool ConfigWidget::GetWidgetVisible(const std::string &_name) const
{
  auto iter = this->configWidgets.find(_name);
  if (iter != this->configWidgets.end())
  {
    if (iter->second->groupWidget)
    {
      GroupWidget *groupWidget =
          qobject_cast<GroupWidget *>(iter->second->groupWidget);
      if (groupWidget)
      {
        return groupWidget->isVisible();
      }
    }
    return iter->second->isVisible();
  }
  return false;
}

/////////////////////////////////////////////////
void ConfigWidget::SetWidgetVisible(const std::string &_name, bool _visible)
{
  auto iter = this->configWidgets.find(_name);
  if (iter != this->configWidgets.end())
  {
    if (iter->second->groupWidget)
    {
      GroupWidget *groupWidget =
          qobject_cast<GroupWidget *>(iter->second->groupWidget);
      if (groupWidget)
      {
        groupWidget->setVisible(_visible);
        return;
      }
    }
    iter->second->setVisible(_visible);
  }
}

/////////////////////////////////////////////////
bool ConfigWidget::GetWidgetReadOnly(const std::string &_name) const
{
  auto iter = this->configWidgets.find(_name);
  if (iter != this->configWidgets.end())
  {
    if (iter->second->groupWidget)
    {
      GroupWidget *groupWidget =
          qobject_cast<GroupWidget *>(iter->second->groupWidget);
      if (groupWidget)
      {
        return !groupWidget->isEnabled();
      }
    }
    return !iter->second->isEnabled();
  }
  return false;
}

/////////////////////////////////////////////////
void ConfigWidget::SetWidgetReadOnly(const std::string &_name, bool _readOnly)
{
  auto iter = this->configWidgets.find(_name);
  if (iter != this->configWidgets.end())
  {
    if (iter->second->groupWidget)
    {
      GroupWidget *groupWidget =
          qobject_cast<GroupWidget *>(iter->second->groupWidget);
      if (groupWidget)
      {
        groupWidget->setEnabled(!_readOnly);
        return;
      }
    }
    iter->second->setEnabled(!_readOnly);
  }
}

/////////////////////////////////////////////////
bool ConfigWidget::SetIntWidgetValue(const std::string &_name, int _value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateIntWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetUIntWidgetValue(const std::string &_name,
    unsigned int _value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateUIntWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetDoubleWidgetValue(const std::string &_name,
    double _value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateDoubleWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetBoolWidgetValue(const std::string &_name,
    bool _value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateBoolWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetStringWidgetValue(const std::string &_name,
    const std::string &_value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateStringWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetVector3WidgetValue(const std::string &_name,
    const math::Vector3 &_value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateVector3Widget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetColorWidgetValue(const std::string &_name,
    const common::Color &_value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateColorWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetPoseWidgetValue(const std::string &_name,
    const math::Pose &_value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdatePoseWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetGeometryWidgetValue(const std::string &_name,
    const std::string &_value, const math::Vector3 &_dimensions,
    const std::string &_uri)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateGeometryWidget(iter->second, _value, _dimensions, _uri);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetDensityWidgetValue(const std::string &_name,
    double _value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateDensityWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::SetEnumWidgetValue(const std::string &_name,
    const std::string &_value)
{
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    return this->UpdateEnumWidget(iter->second, _value);

  return false;
}

/////////////////////////////////////////////////
int ConfigWidget::GetIntWidgetValue(const std::string &_name) const
{
  int value = 0;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetIntWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
unsigned int ConfigWidget::GetUIntWidgetValue(const std::string &_name) const
{
  unsigned int value = 0;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetUIntWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
double ConfigWidget::GetDoubleWidgetValue(const std::string &_name) const
{
  double value = 0.0;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetDoubleWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
bool ConfigWidget::GetBoolWidgetValue(const std::string &_name) const
{
  bool value = false;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetBoolWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetStringWidgetValue(const std::string &_name) const
{
  std::string value;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetStringWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
math::Vector3 ConfigWidget::GetVector3WidgetValue(const std::string &_name)
    const
{
  math::Vector3 value;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetVector3WidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
common::Color ConfigWidget::GetColorWidgetValue(const std::string &_name) const
{
  common::Color value;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetColorWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
math::Pose ConfigWidget::GetPoseWidgetValue(const std::string &_name) const
{
  math::Pose value;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetPoseWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetGeometryWidgetValue(const std::string &_name,
    math::Vector3 &_dimensions, std::string &_uri) const
{
  ignition::math::Vector3d dimensions;
  std::string type = this->GeometryWidgetValue(_name, dimensions, _uri);
  _dimensions = dimensions;

  return type;
}

/////////////////////////////////////////////////
double ConfigWidget::DensityWidgetValue(const std::string &_name) const
{
  double value = 0.0;

  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
  {
    DensityConfigWidget *widget =
        qobject_cast<DensityConfigWidget *>(iter->second);

    if (widget)
      value = widget->Density();
  }
  return value;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GeometryWidgetValue(const std::string &_name,
    ignition::math::Vector3d &_dimensions, std::string &_uri) const
{
  std::string type;
  std::map <std::string, ConfigChildWidget *>::const_iterator iter =
      this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    type = this->GetGeometryWidgetValue(iter->second, _dimensions, _uri);
  return type;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetEnumWidgetValue(const std::string &_name) const
{
  std::string value;
  auto iter = this->configWidgets.find(_name);

  if (iter != this->configWidgets.end())
    value = this->GetEnumWidgetValue(iter->second);
  return value;
}

/////////////////////////////////////////////////
QWidget *ConfigWidget::Parse(google::protobuf::Message *_msg,
  bool _update, const std::string &_name, const int _level)
{
  std::vector<QWidget *> newWidgets;

  const google::protobuf::Descriptor *d = _msg->GetDescriptor();
  if (!d)
    return NULL;
  unsigned int count = d->field_count();

  for (unsigned int i = 0; i < count ; ++i)
  {
    const google::protobuf::FieldDescriptor *field = d->field(i);

    if (!field)
      return NULL;

    const google::protobuf::Reflection *ref = _msg->GetReflection();

    if (!ref)
      return NULL;

    std::string name = field->name();

    // Parse each field in the message
    // TODO parse repeated fields
    if (!field->is_repeated())
    {
      if (_update && !ref->HasField(*_msg, field))
        continue;

      QWidget *newFieldWidget = NULL;
      ConfigChildWidget *configChildWidget = NULL;

      bool newWidget = true;
      std::string scopedName = _name.empty() ? name : _name + "::" + name;
      if (this->configWidgets.find(scopedName) != this->configWidgets.end())
      {
        newWidget = false;
        configChildWidget = this->configWidgets[scopedName];
      }

      switch (field->type())
      {
        case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
        {
          double value = ref->GetDouble(*_msg, field);
          if (!math::equal(value, value))
            value = 0;
          if (newWidget)
          {
            configChildWidget = this->CreateDoubleWidget(name, _level);
            if (name == "mass")
            {
              QDoubleSpinBox *valueSpinBox = qobject_cast<QDoubleSpinBox *>(
                  configChildWidget->widgets[0]);
              if (valueSpinBox)
              {
                connect(valueSpinBox, SIGNAL(valueChanged(double)),
                    this, SLOT(OnMassValueChanged(double)));
              }
            }
            newFieldWidget = configChildWidget;
          }
          this->UpdateDoubleWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_FLOAT:
        {
          float value = ref->GetFloat(*_msg, field);
          if (!math::equal(value, value))
            value = 0;
          if (newWidget)
          {
            configChildWidget = this->CreateDoubleWidget(name, _level);
            newFieldWidget = configChildWidget;
          }
          this->UpdateDoubleWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT64:
        {
          int64_t value = ref->GetInt64(*_msg, field);
          if (newWidget)
          {
            configChildWidget = this->CreateIntWidget(name, _level);
            newFieldWidget = configChildWidget;
          }
          this->UpdateIntWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT64:
        {
          uint64_t value = ref->GetUInt64(*_msg, field);
          if (newWidget)
          {
            configChildWidget = this->CreateUIntWidget(name, _level);
            newFieldWidget = configChildWidget;
          }
          this->UpdateUIntWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT32:
        {
          int32_t value = ref->GetInt32(*_msg, field);
          if (newWidget)
          {
            configChildWidget = this->CreateIntWidget(name, _level);
            newFieldWidget = configChildWidget;
          }
          this->UpdateIntWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT32:
        {
          uint32_t value = ref->GetUInt32(*_msg, field);
          if (newWidget)
          {
            configChildWidget = this->CreateUIntWidget(name, _level);
            newFieldWidget = configChildWidget;
          }
          this->UpdateUIntWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_BOOL:
        {
          bool value = ref->GetBool(*_msg, field);
          if (newWidget)
          {
            configChildWidget = this->CreateBoolWidget(name, _level);
            newFieldWidget = configChildWidget;
          }
          this->UpdateBoolWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_STRING:
        {
          std::string value = ref->GetString(*_msg, field);
          if (newWidget)
          {
            // Choose either a one-line or a multi-line widget according to name
            std::string type = "line";
            if (name == "innerxml")
              type = "plain";

            configChildWidget = this->CreateStringWidget(name, _level, type);
            newFieldWidget = configChildWidget;
          }
          this->UpdateStringWidget(configChildWidget, value);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
        {
          google::protobuf::Message *valueMsg =
              ref->MutableMessage(_msg, field);

          // parse and create custom geometry widgets
          if (field->message_type()->name() == "Geometry")
          {
            if (newWidget)
            {
              configChildWidget = this->CreateGeometryWidget(name, _level);
              newFieldWidget = configChildWidget;
            }

            // type
            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();
            const google::protobuf::FieldDescriptor *typeField =
                valueDescriptor->FindFieldByName("type");

            if (valueMsg->GetReflection()->HasField(*valueMsg, typeField))
            {
              const google::protobuf::EnumValueDescriptor *typeValueDescriptor =
                  valueMsg->GetReflection()->GetEnum(*valueMsg, typeField);

              std::string geometryTypeStr;
              if (typeValueDescriptor)
              {
                geometryTypeStr =
                    QString(typeValueDescriptor->name().c_str()).toLower().
                    toStdString();
              }

              math::Vector3 dimensions;
              // dimensions
              for (int k = 0; k < valueDescriptor->field_count() ; ++k)
              {
                const google::protobuf::FieldDescriptor *geomField =
                    valueDescriptor->field(k);

                if (geomField->is_repeated())
                    continue;

                if (geomField->type() !=
                    google::protobuf::FieldDescriptor::TYPE_MESSAGE ||
                    !valueMsg->GetReflection()->HasField(*valueMsg, geomField))
                  continue;

                google::protobuf::Message *geomValueMsg =
                    valueMsg->GetReflection()->MutableMessage(
                    valueMsg, geomField);
                const google::protobuf::Descriptor *geomValueDescriptor =
                    geomValueMsg->GetDescriptor();

                std::string geomMsgName = geomField->message_type()->name();
                if (geomMsgName == "BoxGeom" || geomMsgName == "MeshGeom")
                {
                  int fieldIdx = (geomMsgName == "BoxGeom") ? 0 : 1;
                  google::protobuf::Message *geomDimMsg =
                      geomValueMsg->GetReflection()->MutableMessage(
                      geomValueMsg, geomValueDescriptor->field(fieldIdx));
                  dimensions = this->ParseVector3(geomDimMsg);
                  break;
                }
                else if (geomMsgName == "CylinderGeom")
                {
                  const google::protobuf::FieldDescriptor *geomRadiusField =
                      geomValueDescriptor->FindFieldByName("radius");
                  double radius = geomValueMsg->GetReflection()->GetDouble(
                      *geomValueMsg, geomRadiusField);
                  const google::protobuf::FieldDescriptor *geomLengthField =
                      geomValueDescriptor->FindFieldByName("length");
                  double length = geomValueMsg->GetReflection()->GetDouble(
                      *geomValueMsg, geomLengthField);
                  dimensions.x = radius * 2.0;
                  dimensions.y = dimensions.x;
                  dimensions.z = length;
                  break;
                }
                else if (geomMsgName == "SphereGeom")
                {
                  const google::protobuf::FieldDescriptor *geomRadiusField =
                      geomValueDescriptor->FindFieldByName("radius");
                  double radius = geomValueMsg->GetReflection()->GetDouble(
                      *geomValueMsg, geomRadiusField);
                  dimensions.x = radius * 2.0;
                  dimensions.y = dimensions.x;
                  dimensions.z = dimensions.x;
                  break;
                }
                else if (geomMsgName == "PolylineGeom")
                {
                  continue;
                }
              }
              this->UpdateGeometryWidget(configChildWidget,
                  geometryTypeStr, dimensions);
            }
          }
          // parse and create custom pose widgets
          else if (field->message_type()->name() == "Pose")
          {
            if (newWidget)
            {
              configChildWidget = this->CreatePoseWidget(name, _level);
              newFieldWidget = configChildWidget;
            }

            math::Pose value;
            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();
            int valueMsgFieldCount = valueDescriptor->field_count();
            for (int j = 0; j < valueMsgFieldCount ; ++j)
            {
              const google::protobuf::FieldDescriptor *valueField =
                  valueDescriptor->field(j);

              if (valueField->type() !=
                  google::protobuf::FieldDescriptor::TYPE_MESSAGE)
                continue;

              if (valueField->message_type()->name() == "Vector3d")
              {
                // pos
                google::protobuf::Message *posValueMsg =
                    valueMsg->GetReflection()->MutableMessage(
                    valueMsg, valueField);
                math::Vector3 vec3 = this->ParseVector3(posValueMsg);
                value.pos = vec3;
              }
              else if (valueField->message_type()->name() == "Quaternion")
              {
                // rot
                google::protobuf::Message *quatValueMsg =
                    valueMsg->GetReflection()->MutableMessage(
                    valueMsg, valueField);
                const google::protobuf::Descriptor *quatValueDescriptor =
                    quatValueMsg->GetDescriptor();
                std::vector<double> quatValues;
                for (unsigned int k = 0; k < 4; ++k)
                {
                  const google::protobuf::FieldDescriptor *quatValueField =
                      quatValueDescriptor->field(k);
                  quatValues.push_back(quatValueMsg->GetReflection()->GetDouble(
                      *quatValueMsg, quatValueField));
                }
                math::Quaternion quat(quatValues[3], quatValues[0],
                    quatValues[1], quatValues[2]);
                value.rot = quat;
              }
            }
            this->UpdatePoseWidget(configChildWidget, value);
          }
          // parse and create custom vector3 widgets
          else if (field->message_type()->name() == "Vector3d")
          {
            if (newWidget)
            {
              configChildWidget = this->CreateVector3dWidget(name, _level);
              newFieldWidget = configChildWidget;
            }

            math::Vector3 vec3 = this->ParseVector3(valueMsg);
            this->UpdateVector3Widget(configChildWidget, vec3);
          }
          // parse and create custom color widgets
          else if (field->message_type()->name() == "Color")
          {
            if (newWidget)
            {
              configChildWidget = this->CreateColorWidget(name, _level);
              newFieldWidget = configChildWidget;
            }

            common::Color color;
            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();
            std::vector<double> values;
            for (unsigned int j = 0; j < configChildWidget->widgets.size(); ++j)
            {
              const google::protobuf::FieldDescriptor *valueField =
                  valueDescriptor->field(j);
              if (valueMsg->GetReflection()->HasField(*valueMsg, valueField))
              {
                values.push_back(valueMsg->GetReflection()->GetFloat(
                    *valueMsg, valueField));
              }
              else
                values.push_back(0);
            }
            color.r = values[0];
            color.g = values[1];
            color.b = values[2];
            color.a = values[3];
            this->UpdateColorWidget(configChildWidget, color);
          }
          // parse and create custom density widgets
          else if (field->message_type()->name() == "Density")
          {
            if (newWidget)
            {
              configChildWidget = this->CreateDensityWidget(name, _level);
              newFieldWidget = configChildWidget;
            }
            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();

            double density = 1.0;

            int valueMsgFieldCount = valueDescriptor->field_count();
            for (int j = 0; j < valueMsgFieldCount ; ++j)
            {
              const google::protobuf::FieldDescriptor *valueField =
                  valueDescriptor->field(j);

              if (valueField && valueField->name() == "density")
                density = valueMsg->GetReflection()->GetDouble(
                    *valueMsg, valueField);
            }
            this->UpdateDensityWidget(configChildWidget, density);
          }
          else
          {
            // parse the message fields recursively
            QWidget *groupBoxWidget =
                this->Parse(valueMsg, _update, scopedName, _level+1);
            if (groupBoxWidget)
            {
              newFieldWidget = new ConfigChildWidget();
              QVBoxLayout *groupBoxLayout = new QVBoxLayout;
              groupBoxLayout->setContentsMargins(0, 0, 0, 0);
              groupBoxLayout->addWidget(groupBoxWidget);
              newFieldWidget->setLayout(groupBoxLayout);
              qobject_cast<ConfigChildWidget *>(newFieldWidget)->
                  widgets.push_back(groupBoxWidget);
            }
          }

          if (newWidget)
          {
            // Make it into a group widget
            ConfigChildWidget *childWidget =
                qobject_cast<ConfigChildWidget *>(newFieldWidget);
            if (childWidget)
            {
              newFieldWidget = this->CreateGroupWidget(name, childWidget,
                  _level);
            }
          }

          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_ENUM:
        {
          const google::protobuf::EnumValueDescriptor *value =
              ref->GetEnum(*_msg, field);

          if (!value)
          {
            gzerr << "Error retrieving enum value for '" << name << "'"
                << std::endl;
            break;
          }

          if (newWidget)
          {
            std::vector<std::string> enumValues;
            const google::protobuf::EnumDescriptor *descriptor = value->type();
            if (!descriptor)
              break;

            for (int j = 0; j < descriptor->value_count(); ++j)
            {
              const google::protobuf::EnumValueDescriptor *valueDescriptor =
                  descriptor->value(j);
              if (valueDescriptor)
                enumValues.push_back(valueDescriptor->name());
            }
            configChildWidget =
                this->CreateEnumWidget(name, enumValues, _level);

            if (!configChildWidget)
            {
              gzerr << "Error creating an enum widget for '" << name << "'"
                  << std::endl;
              break;
            }

            newFieldWidget = configChildWidget;
          }
          this->UpdateEnumWidget(configChildWidget, value->name());
          break;
        }
        default:
          break;
      }

      // Style widgets without parent (level 0)
      if (newFieldWidget && _level == 0 &&
          !qobject_cast<GroupWidget *>(newFieldWidget))
      {
        newFieldWidget->setStyleSheet(
            "QWidget\
            {\
              background-color: " + this->bgColors[0] +
            "}\
            QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox, QPlainTextEdit\
            {\
              background-color: " + this->widgetColors[0] +
            "}");
      }

      if (newWidget && newFieldWidget)
      {
        newWidgets.push_back(newFieldWidget);

        // store the newly created widget in a map with a unique scoped name.
        if (qobject_cast<GroupWidget *>(newFieldWidget))
        {
          GroupWidget *groupWidget =
              qobject_cast<GroupWidget *>(newFieldWidget);
          ConfigChildWidget *childWidget = qobject_cast<ConfigChildWidget *>(
              groupWidget->childWidget);
          this->AddConfigChildWidget(scopedName, childWidget);
        }
        else if (qobject_cast<ConfigChildWidget *>(newFieldWidget))
        {
          this->AddConfigChildWidget(scopedName,
              qobject_cast<ConfigChildWidget *>(newFieldWidget));
        }
      }
    }
  }

  if (!newWidgets.empty())
  {
    // create a group box to hold child widgets.
    QGroupBox *widget = new QGroupBox();
    QVBoxLayout *widgetLayout = new QVBoxLayout;

    for (unsigned int i = 0; i < newWidgets.size(); ++i)
    {
      widgetLayout->addWidget(newWidgets[i]);
    }

    widgetLayout->setContentsMargins(0, 0, 0, 0);
    widgetLayout->setSpacing(0);
    widgetLayout->setAlignment(Qt::AlignTop);
    widget->setLayout(widgetLayout);
    return widget;
  }

  return NULL;
}

/////////////////////////////////////////////////
GroupWidget *ConfigWidget::CreateGroupWidget(const std::string &_name,
    ConfigChildWidget *_childWidget, const int _level)
{
  // Button label
  QLabel *buttonLabel = new QLabel(
      tr(this->GetHumanReadableKey(_name).c_str()));
  buttonLabel->setToolTip(tr(_name.c_str()));

  // Button icon
  QCheckBox *buttonIcon = new QCheckBox();
  buttonIcon->setChecked(true);
  buttonIcon->setStyleSheet(
      "QCheckBox::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QCheckBox::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");

  // Button layout
  QHBoxLayout *buttonLayout = new QHBoxLayout();
  buttonLayout->addItem(new QSpacerItem(20*_level, 1,
      QSizePolicy::Fixed, QSizePolicy::Fixed));
  buttonLayout->addWidget(buttonLabel);
  buttonLayout->addWidget(buttonIcon);
  buttonLayout->setAlignment(buttonIcon, Qt::AlignRight);

  // Button frame
  QFrame *buttonFrame = new QFrame();
  buttonFrame->setFrameStyle(QFrame::Box);
  buttonFrame->setLayout(buttonLayout);

  // Set color for top level button
  if (_level == 0)
  {
    buttonFrame->setStyleSheet(
        "QWidget\
        {\
          background-color: " + this->bgColors[0] +
        "}");
  }

  // Child widgets are contained in a group box which can be collapsed
  GroupWidget *groupWidget = new GroupWidget;
  groupWidget->setStyleSheet(
      "QGroupBox {\
        border : 0;\
        margin : 0;\
        padding : 0;\
      }");

  connect(buttonIcon, SIGNAL(toggled(bool)), groupWidget, SLOT(Toggle(bool)));

  // Set the child widget
  groupWidget->childWidget = _childWidget;
  _childWidget->groupWidget = groupWidget;
  _childWidget->setContentsMargins(0, 0, 0, 0);

  // Set color for children
  if (_level == 0)
  {
    _childWidget->setStyleSheet(
        "QWidget\
        {\
          background-color: " + this->bgColors[1] +
        "}\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox, QPlainTextEdit\
        {\
          background-color: " + this->widgetColors[1] +
        "}");
  }
  else if (_level == 1)
  {
    _childWidget->setStyleSheet(
        "QWidget\
        {\
          background-color: " + this->bgColors[2] +
        "}\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox, QPlainTextEdit\
        {\
          background-color: " + this->widgetColors[2] +
        "}");
  }
  else if (_level == 2)
  {
    _childWidget->setStyleSheet(
        "QWidget\
        {\
          background-color: " + this->bgColors[3] +
        "}\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox, QPlainTextEdit\
        {\
          background-color: " + this->widgetColors[3] +
        "}");
  }

  // Group Layout
  QGridLayout *configGroupLayout = new QGridLayout;
  configGroupLayout->setContentsMargins(0, 0, 0, 0);
  configGroupLayout->setSpacing(0);
  configGroupLayout->addWidget(buttonFrame, 0, 0);
  configGroupLayout->addWidget(_childWidget, 1, 0);
  groupWidget->setLayout(configGroupLayout);

  return groupWidget;
}

/////////////////////////////////////////////////
math::Vector3 ConfigWidget::ParseVector3(const google::protobuf::Message *_msg)
{
  math::Vector3 vec3;
  const google::protobuf::Descriptor *valueDescriptor = _msg->GetDescriptor();
  std::vector<double> values;
  for (unsigned int i = 0; i < 3; ++i)
  {
    const google::protobuf::FieldDescriptor *valueField =
        valueDescriptor->field(i);
    values.push_back(_msg->GetReflection()->GetDouble(*_msg, valueField));
  }
  vec3.x = values[0];
  vec3.y = values[1];
  vec3.z = values[2];
  return vec3;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateUIntWidget(const std::string &_key,
    const int _level)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Label
  QLabel *keyLabel = new QLabel(tr(this->GetHumanReadableKey(_key).c_str()));
  keyLabel->setToolTip(tr(_key.c_str()));

  // SpinBox
  QSpinBox *valueSpinBox = new QSpinBox(widget);
  valueSpinBox->setRange(0, 1e8);
  valueSpinBox->setAlignment(Qt::AlignRight);
  connect(valueSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnUIntValueChanged()));

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  if (_level != 0)
  {
    widgetLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  widgetLayout->addWidget(keyLabel);
  widgetLayout->addWidget(valueSpinBox);

  // ChildWidget
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(valueSpinBox);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateIntWidget(const std::string &_key,
    const int _level)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Label
  QLabel *keyLabel = new QLabel(tr(this->GetHumanReadableKey(_key).c_str()));
  keyLabel->setToolTip(tr(_key.c_str()));

  // SpinBox
  QSpinBox *valueSpinBox = new QSpinBox(widget);
  valueSpinBox->setRange(-1e8, 1e8);
  valueSpinBox->setAlignment(Qt::AlignRight);
  connect(valueSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnIntValueChanged()));

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  if (_level != 0)
  {
    widgetLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  widgetLayout->addWidget(keyLabel);
  widgetLayout->addWidget(valueSpinBox);

  // ChildWidget
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(valueSpinBox);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateDoubleWidget(const std::string &_key,
    const int _level)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Label
  QLabel *keyLabel = new QLabel(tr(this->GetHumanReadableKey(_key).c_str()));
  keyLabel->setToolTip(tr(_key.c_str()));

  // SpinBox
  double min = 0;
  double max = 0;
  this->GetRangeFromKey(_key, min, max);

  QDoubleSpinBox *valueSpinBox = new QDoubleSpinBox(widget);
  valueSpinBox->setRange(min, max);
  valueSpinBox->setSingleStep(0.01);
  valueSpinBox->setDecimals(8);
  valueSpinBox->setAlignment(Qt::AlignRight);
  connect(valueSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnDoubleValueChanged()));

  // Unit
  std::string jointType = this->GetEnumWidgetValue("type");
  std::string unit = this->GetUnitFromKey(_key, jointType);

  QLabel *unitLabel = new QLabel();
  unitLabel->setMaximumWidth(40);
  unitLabel->setText(QString::fromStdString(unit));

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  if (_level != 0)
  {
    widgetLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  widgetLayout->addWidget(keyLabel);
  widgetLayout->addWidget(valueSpinBox);
  if (unitLabel->text() != "")
    widgetLayout->addWidget(unitLabel);

  // ChildWidget
  widget->key = _key;
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(valueSpinBox);
  widget->mapWidgetToUnit[valueSpinBox] = unitLabel;

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateStringWidget(const std::string &_key,
    const int _level, const std::string &_type)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Label
  QLabel *keyLabel = new QLabel(tr(this->GetHumanReadableKey(_key).c_str()));
  keyLabel->setToolTip(tr(_key.c_str()));

  // Line or Text Edit based on key
  QWidget *valueEdit;
  if (_type == "plain")
  {
    valueEdit = new QPlainTextEdit(widget);
    valueEdit->setMinimumHeight(50);
    // QPlainTextEdit's don't have editingFinished signals
  }
  else if (_type == "line")
  {
    valueEdit = new QLineEdit(widget);
    connect(valueEdit, SIGNAL(editingFinished()), this,
        SLOT(OnStringValueChanged()));
  }
  else
  {
    gzerr << "Unknown type [" << _type << "]. Not creating string widget" <<
        std::endl;
    return NULL;
  }

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  if (_level != 0)
  {
    widgetLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  widgetLayout->addWidget(keyLabel);
  widgetLayout->addWidget(valueEdit);

  // ChildWidget
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(valueEdit);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateBoolWidget(const std::string &_key,
    const int _level)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Label
  QLabel *keyLabel = new QLabel(tr(this->GetHumanReadableKey(_key).c_str()));
  keyLabel->setToolTip(tr(_key.c_str()));

  // Buttons
  QRadioButton *valueTrueRadioButton = new QRadioButton(widget);
  valueTrueRadioButton->setText(tr("True"));
  connect(valueTrueRadioButton, SIGNAL(toggled(bool)), this,
      SLOT(OnBoolValueChanged()));

  QRadioButton *valueFalseRadioButton = new QRadioButton(widget);
  valueFalseRadioButton->setText(tr("False"));
  connect(valueFalseRadioButton, SIGNAL(toggled(bool)), this,
      SLOT(OnBoolValueChanged()));

  QButtonGroup *boolButtonGroup = new QButtonGroup;
  boolButtonGroup->addButton(valueTrueRadioButton);
  boolButtonGroup->addButton(valueFalseRadioButton);
  boolButtonGroup->setExclusive(true);

  QHBoxLayout *buttonLayout = new QHBoxLayout;
  buttonLayout->addWidget(valueTrueRadioButton);
  buttonLayout->addWidget(valueFalseRadioButton);

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  if (_level != 0)
  {
    widgetLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  widgetLayout->addWidget(keyLabel);
  widgetLayout->addLayout(buttonLayout);

  // ChildWidget
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(valueTrueRadioButton);
  widget->widgets.push_back(valueFalseRadioButton);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateVector3dWidget(
    const std::string &_key, const int _level)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Labels
  QLabel *vecXLabel = new QLabel(tr("X"));
  QLabel *vecYLabel = new QLabel(tr("Y"));
  QLabel *vecZLabel = new QLabel(tr("Z"));
  vecXLabel->setToolTip(tr("x"));
  vecYLabel->setToolTip(tr("y"));
  vecZLabel->setToolTip(tr("z"));

  // SpinBoxes
  double min = 0;
  double max = 0;
  this->GetRangeFromKey(_key, min, max);

  QDoubleSpinBox *vecXSpinBox = new QDoubleSpinBox(widget);
  vecXSpinBox->setRange(min, max);
  vecXSpinBox->setSingleStep(0.01);
  vecXSpinBox->setDecimals(6);
  vecXSpinBox->setAlignment(Qt::AlignRight);
  vecXSpinBox->setMaximumWidth(100);
  connect(vecXSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnVector3dValueChanged()));

  QDoubleSpinBox *vecYSpinBox = new QDoubleSpinBox(widget);
  vecYSpinBox->setRange(min, max);
  vecYSpinBox->setSingleStep(0.01);
  vecYSpinBox->setDecimals(6);
  vecYSpinBox->setAlignment(Qt::AlignRight);
  vecYSpinBox->setMaximumWidth(100);
  connect(vecYSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnVector3dValueChanged()));

  QDoubleSpinBox *vecZSpinBox = new QDoubleSpinBox(widget);
  vecZSpinBox->setRange(min, max);
  vecZSpinBox->setSingleStep(0.01);
  vecZSpinBox->setDecimals(6);
  vecZSpinBox->setAlignment(Qt::AlignRight);
  vecZSpinBox->setMaximumWidth(100);
  connect(vecZSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnVector3dValueChanged()));

  // This is inside a group
  int level = _level + 1;

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  widgetLayout->addItem(new QSpacerItem(20*level, 1,
      QSizePolicy::Fixed, QSizePolicy::Fixed));
  widgetLayout->addWidget(vecXLabel);
  widgetLayout->addWidget(vecXSpinBox);
  widgetLayout->addWidget(vecYLabel);
  widgetLayout->addWidget(vecYSpinBox);
  widgetLayout->addWidget(vecZLabel);
  widgetLayout->addWidget(vecZSpinBox);

  widgetLayout->setAlignment(vecXLabel, Qt::AlignRight);
  widgetLayout->setAlignment(vecYLabel, Qt::AlignRight);
  widgetLayout->setAlignment(vecZLabel, Qt::AlignRight);

  // ChildWidget
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(vecXSpinBox);
  widget->widgets.push_back(vecYSpinBox);
  widget->widgets.push_back(vecZSpinBox);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateColorWidget(const std::string &_key,
    const int _level)
{
  // ChildWidget
  ConfigChildWidget *widget = new ConfigChildWidget();

  // Labels
  QLabel *colorRLabel = new QLabel(tr("R"));
  QLabel *colorGLabel = new QLabel(tr("G"));
  QLabel *colorBLabel = new QLabel(tr("B"));
  QLabel *colorALabel = new QLabel(tr("A"));
  colorRLabel->setToolTip(tr("r"));
  colorGLabel->setToolTip(tr("g"));
  colorBLabel->setToolTip(tr("b"));
  colorALabel->setToolTip(tr("a"));

  // SpinBoxes
  double min = 0;
  double max = 0;
  this->GetRangeFromKey(_key, min, max);

  QDoubleSpinBox *colorRSpinBox = new QDoubleSpinBox(widget);
  colorRSpinBox->setRange(0, 1.0);
  colorRSpinBox->setSingleStep(0.1);
  colorRSpinBox->setDecimals(3);
  colorRSpinBox->setAlignment(Qt::AlignRight);
  colorRSpinBox->setMaximumWidth(10);
  connect(colorRSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnColorValueChanged()));

  QDoubleSpinBox *colorGSpinBox = new QDoubleSpinBox(widget);
  colorGSpinBox->setRange(0, 1.0);
  colorGSpinBox->setSingleStep(0.1);
  colorGSpinBox->setDecimals(3);
  colorGSpinBox->setAlignment(Qt::AlignRight);
  colorGSpinBox->setMaximumWidth(10);
  connect(colorGSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnColorValueChanged()));

  QDoubleSpinBox *colorBSpinBox = new QDoubleSpinBox(widget);
  colorBSpinBox->setRange(0, 1.0);
  colorBSpinBox->setSingleStep(0.1);
  colorBSpinBox->setDecimals(3);
  colorBSpinBox->setAlignment(Qt::AlignRight);
  colorBSpinBox->setMaximumWidth(10);
  connect(colorBSpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnColorValueChanged()));

  QDoubleSpinBox *colorASpinBox = new QDoubleSpinBox(widget);
  colorASpinBox->setRange(0, 1.0);
  colorASpinBox->setSingleStep(0.1);
  colorASpinBox->setDecimals(3);
  colorASpinBox->setAlignment(Qt::AlignRight);
  colorASpinBox->setMaximumWidth(10);
  connect(colorASpinBox, SIGNAL(editingFinished()), this,
      SLOT(OnColorValueChanged()));

  // This is inside a group
  int level = _level + 1;

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  widgetLayout->addItem(new QSpacerItem(20*level, 1,
      QSizePolicy::Fixed, QSizePolicy::Fixed));
  widgetLayout->addWidget(colorRLabel);
  widgetLayout->addWidget(colorRSpinBox);
  widgetLayout->addWidget(colorGLabel);
  widgetLayout->addWidget(colorGSpinBox);
  widgetLayout->addWidget(colorBLabel);
  widgetLayout->addWidget(colorBSpinBox);
  widgetLayout->addWidget(colorALabel);
  widgetLayout->addWidget(colorASpinBox);

  widgetLayout->setAlignment(colorRLabel, Qt::AlignRight);
  widgetLayout->setAlignment(colorGLabel, Qt::AlignRight);
  widgetLayout->setAlignment(colorBLabel, Qt::AlignRight);
  widgetLayout->setAlignment(colorALabel, Qt::AlignRight);

  // ChildWidget
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  widget->widgets.push_back(colorRSpinBox);
  widget->widgets.push_back(colorGSpinBox);
  widget->widgets.push_back(colorBSpinBox);
  widget->widgets.push_back(colorASpinBox);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreatePoseWidget(const std::string &/*_key*/,
    const int _level)
{
  // Labels
  std::vector<std::string> elements;
  elements.push_back("x");
  elements.push_back("y");
  elements.push_back("z");
  elements.push_back("roll");
  elements.push_back("pitch");
  elements.push_back("yaw");

  // This is inside a group
  int level = _level+1;

  // Layout
  QGridLayout *widgetLayout = new QGridLayout;
  widgetLayout->setColumnStretch(3, 1);
  widgetLayout->addItem(new QSpacerItem(20*level, 1, QSizePolicy::Fixed,
      QSizePolicy::Fixed), 0, 0);

  // ChildWidget
  double min = 0;
  double max = 0;
  this->GetRangeFromKey("", min, max);

  ConfigChildWidget *widget = new ConfigChildWidget();
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  for (unsigned int i = 0; i < elements.size(); ++i)
  {
    QDoubleSpinBox *spin = new QDoubleSpinBox(widget);
    connect(spin, SIGNAL(editingFinished()), this, SLOT(OnPoseValueChanged()));
    widget->widgets.push_back(spin);

    spin->setRange(min, max);
    spin->setSingleStep(0.01);
    spin->setDecimals(6);
    spin->setAlignment(Qt::AlignRight);
    spin->setMaximumWidth(100);

    QLabel *label = new QLabel(this->GetHumanReadableKey(elements[i]).c_str());
    label->setToolTip(tr(elements[i].c_str()));
    if (i == 0)
      label->setStyleSheet("QLabel{color: " + this->redColor + ";}");
    else if (i == 1)
      label->setStyleSheet("QLabel{color: " + this->greenColor + ";}");
    else if (i == 2)
      label->setStyleSheet("QLabel{color:" + this->blueColor + ";}");

    QLabel *unitLabel = new QLabel();
    unitLabel->setMaximumWidth(40);
    unitLabel->setMinimumWidth(40);
    if (i < 3)
      unitLabel->setText(QString::fromStdString(this->GetUnitFromKey("pos")));
    else
      unitLabel->setText(QString::fromStdString(this->GetUnitFromKey("rot")));

    widgetLayout->addWidget(label, i%3, std::floor(i/3)*3+1);
    widgetLayout->addWidget(spin, i%3, std::floor(i/3)*3+2);
    widgetLayout->addWidget(unitLabel, i%3, std::floor(i/3)*3+3);

    widgetLayout->setAlignment(label, Qt::AlignLeft);
    widgetLayout->setAlignment(spin, Qt::AlignLeft);
    widgetLayout->setAlignment(unitLabel, Qt::AlignLeft);
  }

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateGeometryWidget(
    const std::string &/*_key*/, const int _level)
{
  // Geometry ComboBox
  QLabel *geometryLabel = new QLabel(tr("Geometry"));
  geometryLabel->setToolTip(tr("geometry"));
  QComboBox *geometryComboBox = new QComboBox;
  geometryComboBox->addItem(tr("box"));
  geometryComboBox->addItem(tr("cylinder"));
  geometryComboBox->addItem(tr("sphere"));
  geometryComboBox->addItem(tr("mesh"));
  geometryComboBox->addItem(tr("polyline"));

  // Size XYZ
  double min = 0;
  double max = 0;
  this->GetRangeFromKey("length", min, max);

  QDoubleSpinBox *geomSizeXSpinBox = new QDoubleSpinBox;
  geomSizeXSpinBox->setRange(min, max);
  geomSizeXSpinBox->setSingleStep(0.01);
  geomSizeXSpinBox->setDecimals(6);
  geomSizeXSpinBox->setValue(1.000);
  geomSizeXSpinBox->setAlignment(Qt::AlignRight);
  geomSizeXSpinBox->setMaximumWidth(100);

  QDoubleSpinBox *geomSizeYSpinBox = new QDoubleSpinBox;
  geomSizeYSpinBox->setRange(min, max);
  geomSizeYSpinBox->setSingleStep(0.01);
  geomSizeYSpinBox->setDecimals(6);
  geomSizeYSpinBox->setValue(1.000);
  geomSizeYSpinBox->setAlignment(Qt::AlignRight);
  geomSizeYSpinBox->setMaximumWidth(100);

  QDoubleSpinBox *geomSizeZSpinBox = new QDoubleSpinBox;
  geomSizeZSpinBox->setRange(min, max);
  geomSizeZSpinBox->setSingleStep(0.01);
  geomSizeZSpinBox->setDecimals(6);
  geomSizeZSpinBox->setValue(1.000);
  geomSizeZSpinBox->setAlignment(Qt::AlignRight);
  geomSizeZSpinBox->setMaximumWidth(100);

  QLabel *geomSizeXLabel = new QLabel(tr("X"));
  QLabel *geomSizeYLabel = new QLabel(tr("Y"));
  QLabel *geomSizeZLabel = new QLabel(tr("Z"));
  geomSizeXLabel->setStyleSheet("QLabel{color: " + this->redColor + ";}");
  geomSizeYLabel->setStyleSheet("QLabel{color: " + this->greenColor + ";}");
  geomSizeZLabel->setStyleSheet("QLabel{color: " + this->blueColor + ";}");
  geomSizeXLabel->setToolTip(tr("x"));
  geomSizeYLabel->setToolTip(tr("y"));
  geomSizeZLabel->setToolTip(tr("z"));

  std::string unit = this->GetUnitFromKey("length");
  QLabel *geomSizeXUnitLabel = new QLabel(QString::fromStdString(unit));
  QLabel *geomSizeYUnitLabel = new QLabel(QString::fromStdString(unit));
  QLabel *geomSizeZUnitLabel = new QLabel(QString::fromStdString(unit));

  QHBoxLayout *geomSizeLayout = new QHBoxLayout;
  geomSizeLayout->addWidget(geomSizeXLabel);
  geomSizeLayout->addWidget(geomSizeXSpinBox);
  geomSizeLayout->addWidget(geomSizeXUnitLabel);
  geomSizeLayout->addWidget(geomSizeYLabel);
  geomSizeLayout->addWidget(geomSizeYSpinBox);
  geomSizeLayout->addWidget(geomSizeYUnitLabel);
  geomSizeLayout->addWidget(geomSizeZLabel);
  geomSizeLayout->addWidget(geomSizeZSpinBox);
  geomSizeLayout->addWidget(geomSizeZUnitLabel);

  geomSizeLayout->setAlignment(geomSizeXLabel, Qt::AlignRight);
  geomSizeLayout->setAlignment(geomSizeYLabel, Qt::AlignRight);
  geomSizeLayout->setAlignment(geomSizeZLabel, Qt::AlignRight);

  // Uri
  QLabel *geomFilenameLabel = new QLabel(tr("Uri"));
  geomFilenameLabel->setToolTip(tr("uri"));
  QLineEdit *geomFilenameLineEdit = new QLineEdit;
  QPushButton *geomFilenameButton = new QPushButton(tr("..."));
  geomFilenameButton->setMaximumWidth(30);

  QHBoxLayout *geomFilenameLayout = new QHBoxLayout;
  geomFilenameLayout->addWidget(geomFilenameLabel);
  geomFilenameLayout->addWidget(geomFilenameLineEdit);
  geomFilenameLayout->addWidget(geomFilenameButton);

  QVBoxLayout *geomSizeFilenameLayout = new QVBoxLayout;
  geomSizeFilenameLayout->addLayout(geomSizeLayout);
  geomSizeFilenameLayout->addLayout(geomFilenameLayout);

  QWidget *geomSizeWidget = new QWidget;
  geomSizeWidget->setLayout(geomSizeFilenameLayout);

  // Radius / Length
  QLabel *geomRadiusLabel = new QLabel(tr("Radius"));
  QLabel *geomLengthLabel = new QLabel(tr("Length"));
  QLabel *geomRadiusUnitLabel = new QLabel(QString::fromStdString(unit));
  QLabel *geomLengthUnitLabel = new QLabel(QString::fromStdString(unit));
  geomRadiusLabel->setToolTip(tr("radius"));
  geomLengthLabel->setToolTip(tr("length"));

  QDoubleSpinBox *geomRadiusSpinBox = new QDoubleSpinBox;
  geomRadiusSpinBox->setRange(min, max);
  geomRadiusSpinBox->setSingleStep(0.01);
  geomRadiusSpinBox->setDecimals(6);
  geomRadiusSpinBox->setValue(0.500);
  geomRadiusSpinBox->setAlignment(Qt::AlignRight);
  geomRadiusSpinBox->setMaximumWidth(100);

  QDoubleSpinBox *geomLengthSpinBox = new QDoubleSpinBox;
  geomLengthSpinBox->setRange(min, max);
  geomLengthSpinBox->setSingleStep(0.01);
  geomLengthSpinBox->setDecimals(6);
  geomLengthSpinBox->setValue(1.000);
  geomLengthSpinBox->setAlignment(Qt::AlignRight);
  geomLengthSpinBox->setMaximumWidth(100);

  QHBoxLayout *geomRLLayout = new QHBoxLayout;
  geomRLLayout->addWidget(geomRadiusLabel);
  geomRLLayout->addWidget(geomRadiusSpinBox);
  geomRLLayout->addWidget(geomRadiusUnitLabel);
  geomRLLayout->addWidget(geomLengthLabel);
  geomRLLayout->addWidget(geomLengthSpinBox);
  geomRLLayout->addWidget(geomLengthUnitLabel);

  geomRLLayout->setAlignment(geomRadiusLabel, Qt::AlignRight);
  geomRLLayout->setAlignment(geomLengthLabel, Qt::AlignRight);

  QWidget *geomRLWidget = new QWidget;
  geomRLWidget->setLayout(geomRLLayout);

  // Dimensions
  QStackedWidget *geomDimensionWidget = new QStackedWidget;
  geomDimensionWidget->insertWidget(0, geomSizeWidget);

  geomDimensionWidget->insertWidget(1, geomRLWidget);
  geomDimensionWidget->setCurrentIndex(0);
  geomDimensionWidget->setSizePolicy(
      QSizePolicy::Minimum, QSizePolicy::Minimum);

  // This is inside a group
  int level = _level + 1;

  // Layout
  QGridLayout *widgetLayout = new QGridLayout;
  widgetLayout->addItem(new QSpacerItem(20*level, 1,
      QSizePolicy::Fixed, QSizePolicy::Fixed), 0, 0);
  widgetLayout->addWidget(geometryLabel, 0, 1);
  widgetLayout->addWidget(geometryComboBox, 0, 2, 1, 2);
  widgetLayout->addWidget(geomDimensionWidget, 2, 1, 1, 3);

  // ChildWidget
  GeometryConfigWidget *widget = new GeometryConfigWidget;
  widget->setFrameStyle(QFrame::Box);
  widget->geomDimensionWidget = geomDimensionWidget;
  widget->geomLengthSpinBox = geomLengthSpinBox;
  widget->geomLengthLabel = geomLengthLabel;
  widget->geomLengthUnitLabel = geomLengthUnitLabel;
  widget->geomFilenameLabel = geomFilenameLabel;
  widget->geomFilenameLineEdit = geomFilenameLineEdit;
  widget->geomFilenameButton = geomFilenameButton;

  geomFilenameLabel->setVisible(false);
  geomFilenameLineEdit->setVisible(false);
  geomFilenameButton->setVisible(false);

  connect(geometryComboBox, SIGNAL(currentIndexChanged(const QString)),
      widget, SLOT(OnGeometryTypeChanged(const QString)));

  connect(geomFilenameButton, SIGNAL(clicked()),
      widget, SLOT(OnSelectFile()));

  connect(widget, SIGNAL(GeometryChanged()),
      this, SLOT(OnGeometryChanged()));

  connect(geomSizeXSpinBox, SIGNAL(valueChanged(double)),
      widget, SLOT(OnGeometrySizeChanged(double)));

  connect(geomSizeYSpinBox, SIGNAL(valueChanged(double)),
      widget, SLOT(OnGeometrySizeChanged(double)));

  connect(geomSizeZSpinBox, SIGNAL(valueChanged(double)),
      widget, SLOT(OnGeometrySizeChanged(double)));

  connect(geomRadiusSpinBox, SIGNAL(valueChanged(double)),
      widget, SLOT(OnGeometrySizeChanged(double)));

  connect(geomLengthSpinBox, SIGNAL(valueChanged(double)),
      widget, SLOT(OnGeometrySizeChanged(double)));

  widget->setLayout(widgetLayout);
  widget->widgets.push_back(geometryComboBox);
  widget->widgets.push_back(geomSizeXSpinBox);
  widget->widgets.push_back(geomSizeYSpinBox);
  widget->widgets.push_back(geomSizeZSpinBox);
  widget->widgets.push_back(geomRadiusSpinBox);
  widget->widgets.push_back(geomLengthSpinBox);
  widget->widgets.push_back(geomFilenameLineEdit);
  widget->widgets.push_back(geomFilenameButton);

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateEnumWidget(
    const std::string &_key, const std::vector<std::string> &_values,
    const int _level)
{
  // Label
  QLabel *enumLabel = new QLabel(this->GetHumanReadableKey(_key).c_str());
  enumLabel->setToolTip(tr(_key.c_str()));

  // ComboBox
  QComboBox *enumComboBox = new QComboBox;

  for (unsigned int i = 0; i < _values.size(); ++i)
    enumComboBox->addItem(tr(_values[i].c_str()));

  // Layout
  QHBoxLayout *widgetLayout = new QHBoxLayout;
  if (_level != 0)
  {
    widgetLayout->addItem(new QSpacerItem(20*_level, 1,
        QSizePolicy::Fixed, QSizePolicy::Fixed));
  }
  widgetLayout->addWidget(enumLabel);
  widgetLayout->addWidget(enumComboBox);

  // ChildWidget
  EnumConfigWidget *widget = new EnumConfigWidget();
  widget->setLayout(widgetLayout);
  widget->setFrameStyle(QFrame::Box);

  connect(enumComboBox, SIGNAL(currentIndexChanged(const QString &)),
      widget, SLOT(EnumChanged(const QString &)));

  widget->widgets.push_back(enumComboBox);

  // connect enum config widget event so that we can fire another
  // event from ConfigWidget that has the name of this field
  connect(widget,
      SIGNAL(EnumValueChanged(const QString &)), this,
      SLOT(OnEnumValueChanged(const QString &)));

  return widget;
}

/////////////////////////////////////////////////
ConfigChildWidget *ConfigWidget::CreateDensityWidget(
    const std::string &/*_key*/, const int _level)
{
  QLabel *densityLabel = new QLabel(tr("Density"));
  densityLabel->setToolTip(tr("density"));

  QComboBox *comboBox = new QComboBox;
  size_t minLen = 0;

  for (auto it : DensityModel::Instance()->Entries())
  {
    minLen = std::max(minLen, it->desc.length());
    comboBox->addItem(tr(it->desc.c_str()), QVariant::fromValue(it->value));
  }
  comboBox->addItem(tr("Custom..."));
  // Longest entry plus check box and space
  comboBox->setMinimumContentsLength(minLen+2);

  double min = 0;
  double max = 0;
  this->GetRangeFromKey("density", min, max);

  QDoubleSpinBox *spinBox = new QDoubleSpinBox;
  spinBox->setRange(min, max);
  spinBox->setSingleStep(0.1);
  spinBox->setDecimals(1);
  spinBox->setValue(1.0);
  spinBox->setAlignment(Qt::AlignRight);
  spinBox->setMaximumWidth(100);

  std::string unit = this->GetUnitFromKey("density");
  QLabel *unitLabel = new QLabel(QString::fromStdString(unit));

  QHBoxLayout *widgetLayout = new QHBoxLayout;

  widgetLayout->addSpacing((_level+1)*20);
  widgetLayout->addWidget(densityLabel);
  widgetLayout->addStretch();
  widgetLayout->addWidget(comboBox);
  widgetLayout->addWidget(spinBox);
  widgetLayout->addWidget(unitLabel);

  DensityConfigWidget *widget = new DensityConfigWidget;
  widget->setFrameStyle(QFrame::Box);
  widget->setLayout(widgetLayout);

  widget->comboBox = comboBox;
  widget->spinBox = spinBox;

  connect(comboBox, SIGNAL(currentIndexChanged(const QString)),
      widget, SLOT(OnComboBoxChanged(const QString)));

  connect(spinBox, SIGNAL(valueChanged(const QString)),
      widget, SLOT(OnSpinBoxChanged(const QString)));

  connect(widget, SIGNAL(DensityValueChanged(const double &)),
      this, SLOT(OnDensityValueChanged(const double &)));

  widget->widgets.push_back(comboBox);
  widget->widgets.push_back(spinBox);

  return widget;
}

/////////////////////////////////////////////////
void ConfigWidget::UpdateMsg(google::protobuf::Message *_msg,
    const std::string &_name)
{
  const google::protobuf::Descriptor *d = _msg->GetDescriptor();
  if (!d)
    return;
  unsigned int count = d->field_count();

  for (unsigned int i = 0; i < count ; ++i)
  {
    const google::protobuf::FieldDescriptor *field = d->field(i);

    if (!field)
      return;

    const google::protobuf::Reflection *ref = _msg->GetReflection();

    if (!ref)
      return;

    std::string name = field->name();

    // Update each field in the message
    // TODO update repeated fields
    if (!field->is_repeated() /*&& ref->HasField(*_msg, field)*/)
    {
      std::string scopedName = _name.empty() ? name : _name + "::" + name;
      if (this->configWidgets.find(scopedName) == this->configWidgets.end())
        continue;

      // don't update msgs field that are associated with read-only widgets
      if (this->GetWidgetReadOnly(scopedName))
        continue;

      ConfigChildWidget *childWidget = this->configWidgets[scopedName];

      switch (field->type())
      {
        case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
        {
          QDoubleSpinBox *valueSpinBox =
              qobject_cast<QDoubleSpinBox *>(childWidget->widgets[0]);
          ref->SetDouble(_msg, field, valueSpinBox->value());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_FLOAT:
        {
          QDoubleSpinBox *valueSpinBox =
              qobject_cast<QDoubleSpinBox *>(childWidget->widgets[0]);
          ref->SetFloat(_msg, field, valueSpinBox->value());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT64:
        {
          QSpinBox *valueSpinBox =
              qobject_cast<QSpinBox *>(childWidget->widgets[0]);
          ref->SetInt64(_msg, field, valueSpinBox->value());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT64:
        {
          QSpinBox *valueSpinBox =
              qobject_cast<QSpinBox *>(childWidget->widgets[0]);
          ref->SetUInt64(_msg, field, valueSpinBox->value());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT32:
        {
          QSpinBox *valueSpinBox =
              qobject_cast<QSpinBox *>(childWidget->widgets[0]);
          ref->SetInt32(_msg, field, valueSpinBox->value());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT32:
        {
          QSpinBox *valueSpinBox =
              qobject_cast<QSpinBox *>(childWidget->widgets[0]);
          ref->SetUInt32(_msg, field, valueSpinBox->value());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_BOOL:
        {
          QRadioButton *valueRadioButton =
              qobject_cast<QRadioButton *>(childWidget->widgets[0]);
          ref->SetBool(_msg, field, valueRadioButton->isChecked());
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_STRING:
        {
          if (qobject_cast<QLineEdit *>(childWidget->widgets[0]))
          {
            QLineEdit *valueLineEdit =
              qobject_cast<QLineEdit *>(childWidget->widgets[0]);
            ref->SetString(_msg, field, valueLineEdit->text().toStdString());
          }
          else if (qobject_cast<QPlainTextEdit *>(childWidget->widgets[0]))
          {
            QPlainTextEdit *valueTextEdit =
                qobject_cast<QPlainTextEdit *>(childWidget->widgets[0]);
            ref->SetString(_msg, field,
                valueTextEdit->toPlainText().toStdString());
          }
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
        {
          google::protobuf::Message *valueMsg =
              (ref->MutableMessage(_msg, field));

          // update geometry msg field
          if (field->message_type()->name() == "Geometry")
          {
            // manually retrieve values from widgets in order to update
            // the message fields.
            QComboBox *valueComboBox =
                qobject_cast<QComboBox *>(childWidget->widgets[0]);
            std::string geomType = valueComboBox->currentText().toStdString();

            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();
            const google::protobuf::Reflection *geomReflection =
                valueMsg->GetReflection();
            const google::protobuf::FieldDescriptor *typeField =
                valueDescriptor->FindFieldByName("type");
            const google::protobuf::EnumDescriptor *typeEnumDescriptor =
                typeField->enum_type();

            if (geomType == "box" || geomType == "mesh")
            {
              double sizeX = qobject_cast<QDoubleSpinBox *>(
                  childWidget->widgets[1])->value();
              double sizeY = qobject_cast<QDoubleSpinBox *>(
                  childWidget->widgets[2])->value();
              double sizeZ = qobject_cast<QDoubleSpinBox *>(
                  childWidget->widgets[3])->value();
              math::Vector3 geomSize(sizeX, sizeY, sizeZ);

              // set type
              std::string typeStr =
                  QString(tr(geomType.c_str())).toUpper().toStdString();
              const google::protobuf::EnumValueDescriptor *geometryType =
                  typeEnumDescriptor->FindValueByName(typeStr);
              geomReflection->SetEnum(valueMsg, typeField, geometryType);

              // set dimensions
              const google::protobuf::FieldDescriptor *geomFieldDescriptor =
                valueDescriptor->FindFieldByName(geomType);
              google::protobuf::Message *geomValueMsg =
                  geomReflection->MutableMessage(valueMsg, geomFieldDescriptor);

              int fieldIdx = (geomType == "box") ? 0 : 1;
              google::protobuf::Message *geomDimensionMsg =
                  geomValueMsg->GetReflection()->MutableMessage(geomValueMsg,
                  geomValueMsg->GetDescriptor()->field(fieldIdx));
              this->UpdateVector3Msg(geomDimensionMsg, geomSize);

              if (geomType == "mesh")
              {
                std::string uri = qobject_cast<QLineEdit *>(
                     childWidget->widgets[6])->text().toStdString();
                const google::protobuf::FieldDescriptor *uriFieldDescriptor =
                    geomValueMsg->GetDescriptor()->field(0);
                geomValueMsg->GetReflection()->SetString(geomValueMsg,
                    uriFieldDescriptor, uri);
              }
            }
            else if (geomType == "cylinder")
            {
              double radius = qobject_cast<QDoubleSpinBox *>(
                  childWidget->widgets[4])->value();
              double length = qobject_cast<QDoubleSpinBox *>(
                  childWidget->widgets[5])->value();

              // set type
              const google::protobuf::EnumValueDescriptor *geometryType =
                  typeEnumDescriptor->FindValueByName("CYLINDER");
              geomReflection->SetEnum(valueMsg, typeField, geometryType);

              // set radius and length
              const google::protobuf::FieldDescriptor *geomFieldDescriptor =
                valueDescriptor->FindFieldByName(geomType);
              google::protobuf::Message *geomValueMsg =
                  geomReflection->MutableMessage(valueMsg, geomFieldDescriptor);

              const google::protobuf::FieldDescriptor *geomRadiusField =
                  geomValueMsg->GetDescriptor()->field(0);
              geomValueMsg->GetReflection()->SetDouble(geomValueMsg,
                  geomRadiusField, radius);
              const google::protobuf::FieldDescriptor *geomLengthField =
                  geomValueMsg->GetDescriptor()->field(1);
              geomValueMsg->GetReflection()->SetDouble(geomValueMsg,
                  geomLengthField, length);
            }
            else if (geomType == "sphere")
            {
              double radius = qobject_cast<QDoubleSpinBox *>(
                  childWidget->widgets[4])->value();

              // set type
              const google::protobuf::EnumValueDescriptor *geometryType =
                  typeEnumDescriptor->FindValueByName("SPHERE");
              geomReflection->SetEnum(valueMsg, typeField, geometryType);

              // set radius
              const google::protobuf::FieldDescriptor *geomFieldDescriptor =
                valueDescriptor->FindFieldByName(geomType);
              google::protobuf::Message *geomValueMsg =
                  geomReflection->MutableMessage(valueMsg, geomFieldDescriptor);

              const google::protobuf::FieldDescriptor *geomRadiusField =
                  geomValueMsg->GetDescriptor()->field(0);
              geomValueMsg->GetReflection()->SetDouble(geomValueMsg,
                  geomRadiusField, radius);
            }
            else if (geomType == "polyline")
            {
              const google::protobuf::EnumValueDescriptor *geometryType =
                  typeEnumDescriptor->FindValueByName("POLYLINE");
              geomReflection->SetEnum(valueMsg, typeField, geometryType);
            }
          }
          // update pose msg field
          else if (field->message_type()->name() == "Pose")
          {
            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();
            int valueMsgFieldCount = valueDescriptor->field_count();

            // loop through the message fields to update:
            // a vector3d field (position)
            // and quaternion field (orientation)
            for (int j = 0; j < valueMsgFieldCount ; ++j)
            {
              const google::protobuf::FieldDescriptor *valueField =
                  valueDescriptor->field(j);

              if (valueField->type() !=
                  google::protobuf::FieldDescriptor::TYPE_MESSAGE)
                continue;

              if (valueField->message_type()->name() == "Vector3d")
              {
                // pos
                google::protobuf::Message *posValueMsg =
                    valueMsg->GetReflection()->MutableMessage(
                    valueMsg, valueField);
                std::vector<double> values;
                for (unsigned int k = 0; k < 3; ++k)
                {
                  QDoubleSpinBox *valueSpinBox =
                      qobject_cast<QDoubleSpinBox *>(childWidget->widgets[k]);
                  values.push_back(valueSpinBox->value());
                }
                math::Vector3 vec3(values[0], values[1], values[2]);
                this->UpdateVector3Msg(posValueMsg, vec3);
              }
              else if (valueField->message_type()->name() == "Quaternion")
              {
                // rot
                google::protobuf::Message *quatValueMsg =
                    valueMsg->GetReflection()->MutableMessage(
                    valueMsg, valueField);
                std::vector<double> rotValues;
                for (unsigned int k = 3; k < 6; ++k)
                {
                  QDoubleSpinBox *valueSpinBox =
                      qobject_cast<QDoubleSpinBox *>(childWidget->widgets[k]);
                  rotValues.push_back(valueSpinBox->value());
                }
                math::Quaternion quat(rotValues[0], rotValues[1], rotValues[2]);

                std::vector<double> quatValues;
                quatValues.push_back(quat.x);
                quatValues.push_back(quat.y);
                quatValues.push_back(quat.z);
                quatValues.push_back(quat.w);
                const google::protobuf::Descriptor *quatValueDescriptor =
                    quatValueMsg->GetDescriptor();
                for (unsigned int k = 0; k < quatValues.size(); ++k)
                {
                  const google::protobuf::FieldDescriptor *quatValueField =
                      quatValueDescriptor->field(k);
                  quatValueMsg->GetReflection()->SetDouble(quatValueMsg,
                      quatValueField, quatValues[k]);
                }
              }
            }
          }
          else if (field->message_type()->name() == "Vector3d")
          {
            std::vector<double> values;
            for (unsigned int j = 0; j < childWidget->widgets.size(); ++j)
            {
              QDoubleSpinBox *valueSpinBox =
                  qobject_cast<QDoubleSpinBox *>(childWidget->widgets[j]);
              values.push_back(valueSpinBox->value());
            }
            math::Vector3 vec3(values[0], values[1], values[2]);
            this->UpdateVector3Msg(valueMsg, vec3);
          }
          else if (field->message_type()->name() == "Color")
          {
            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();
            for (unsigned int j = 0; j < childWidget->widgets.size(); ++j)
            {
              QDoubleSpinBox *valueSpinBox =
                  qobject_cast<QDoubleSpinBox *>(childWidget->widgets[j]);
              const google::protobuf::FieldDescriptor *valueField =
                  valueDescriptor->field(j);
              valueMsg->GetReflection()->SetFloat(valueMsg, valueField,
                  valueSpinBox->value());
            }
          }
          else if (field->message_type()->name() == "Density")
          {
            DensityConfigWidget *densityWidget =
                qobject_cast<DensityConfigWidget *>(childWidget);

            const google::protobuf::Descriptor *valueDescriptor =
                valueMsg->GetDescriptor();

            const google::protobuf::FieldDescriptor *densityField =
                            valueDescriptor->FindFieldByName("density");

            valueMsg->GetReflection()->SetDouble(valueMsg, densityField,
                densityWidget->Density());
          }
          else
          {
            // update the message fields recursively
            this->UpdateMsg(valueMsg, scopedName);
          }

          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_ENUM:
        {
          QComboBox *valueComboBox =
              qobject_cast<QComboBox *>(childWidget->widgets[0]);
          if (valueComboBox)
          {
            std::string valueStr = valueComboBox->currentText().toStdString();
            const google::protobuf::EnumDescriptor *enumDescriptor =
                field->enum_type();
            if (enumDescriptor)
            {
              const google::protobuf::EnumValueDescriptor *enumValue =
                  enumDescriptor->FindValueByName(valueStr);
              if (enumValue)
                ref->SetEnum(_msg, field, enumValue);
              else
                gzerr << "Unable to find enum value: '" << valueStr << "'"
                    << std::endl;
            }
          }
          break;
        }
        default:
          break;
      }
    }
  }
}

/////////////////////////////////////////////////
void ConfigWidget::UpdateVector3Msg(google::protobuf::Message *_msg,
    const math::Vector3 &_value)
{
  const google::protobuf::Descriptor *valueDescriptor = _msg->GetDescriptor();

  std::vector<double> values;
  values.push_back(_value.x);
  values.push_back(_value.y);
  values.push_back(_value.z);

  for (unsigned int i = 0; i < 3; ++i)
  {
    const google::protobuf::FieldDescriptor *valueField =
        valueDescriptor->field(i);
    _msg->GetReflection()->SetDouble(_msg, valueField, values[i]);
  }
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateIntWidget(ConfigChildWidget *_widget,  int _value)
{
  if (_widget->widgets.size() == 1u)
  {
    qobject_cast<QSpinBox *>(_widget->widgets[0])->setValue(_value);
    return true;
  }
  else
  {
    gzerr << "Error updating Int Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateUIntWidget(ConfigChildWidget *_widget,
    unsigned int _value)
{
  if (_widget->widgets.size() == 1u)
  {
    qobject_cast<QSpinBox *>(_widget->widgets[0])->setValue(_value);
    return true;
  }
  else
  {
    gzerr << "Error updating UInt Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateDoubleWidget(ConfigChildWidget *_widget, double _value)
{
  if (_widget->widgets.size() == 1u)
  {
    // Spin value
    QDoubleSpinBox *spin =
        qobject_cast<QDoubleSpinBox *>(_widget->widgets[0]);
    spin->setValue(_value);

    // Unit label
    std::string jointType = this->GetEnumWidgetValue("type");
    std::string unit = this->GetUnitFromKey(_widget->key, jointType);
    qobject_cast<QLabel *>(
        _widget->mapWidgetToUnit[spin])->setText(QString::fromStdString(unit));

    return true;
  }
  else
  {
    gzerr << "Error updating Double Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateStringWidget(ConfigChildWidget *_widget,
    const std::string &_value)
{
  if (_widget->widgets.size() == 1u)
  {
    if (qobject_cast<QLineEdit *>(_widget->widgets[0]))
    {
      qobject_cast<QLineEdit *>(_widget->widgets[0])
          ->setText(tr(_value.c_str()));
      return true;
    }
    else if (qobject_cast<QPlainTextEdit *>(_widget->widgets[0]))
    {
      qobject_cast<QPlainTextEdit *>(_widget->widgets[0])
          ->setPlainText(tr(_value.c_str()));
      return true;
    }
  }
  else
  {
    gzerr << "Error updating String Config Widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateBoolWidget(ConfigChildWidget *_widget, bool _value)
{
  if (_widget->widgets.size() == 2u)
  {
    qobject_cast<QRadioButton *>(_widget->widgets[0])->setChecked(_value);
    qobject_cast<QRadioButton *>(_widget->widgets[1])->setChecked(!_value);
    return true;
  }
  else
  {
    gzerr << "Error updating Bool Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateVector3Widget(ConfigChildWidget *_widget,
    const math::Vector3 &_vec)
{
  if (_widget->widgets.size() == 3u)
  {
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->setValue(_vec.x);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->setValue(_vec.y);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->setValue(_vec.z);
    return true;
  }
  else
  {
    gzerr << "Error updating Vector3 Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateColorWidget(ConfigChildWidget *_widget,
    const common::Color &_color)
{
  if (_widget->widgets.size() == 4u)
  {
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->setValue(_color.r);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->setValue(_color.g);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->setValue(_color.b);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[3])->setValue(_color.a);
    return true;
  }
  else
  {
    gzerr << "Error updating Color Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdatePoseWidget(ConfigChildWidget *_widget,
    const math::Pose &_pose)
{
  if (_widget->widgets.size() == 6u)
  {
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->setValue(_pose.pos.x);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->setValue(_pose.pos.y);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->setValue(_pose.pos.z);

    math::Vector3 rot = _pose.rot.GetAsEuler();
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[3])->setValue(rot.x);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[4])->setValue(rot.y);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[5])->setValue(rot.z);
    return true;
  }
  else
  {
    gzerr << "Error updating Pose Config widget" << std::endl;
  }
  return false;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateGeometryWidget(ConfigChildWidget *_widget,
    const std::string &_value, const math::Vector3 &_dimensions,
    const std::string &_uri)
{
  if (_widget->widgets.size() != 8u)
  {
    gzerr << "Error updating Geometry Config widget " << std::endl;
    return false;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(_widget->widgets[0]);
  int index = valueComboBox->findText(tr(_value.c_str()));

  if (index < 0)
  {
    gzerr << "Error updating Geometry Config widget: '" << _value <<
      "' not found" << std::endl;
    return false;
  }

  qobject_cast<QComboBox *>(_widget->widgets[0])->setCurrentIndex(index);

  bool isMesh =  _value == "mesh";
  if (_value == "box" || isMesh)
  {
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->setValue(
        _dimensions.x);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->setValue(
        _dimensions.y);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[3])->setValue(
        _dimensions.z);
  }
  else if (_value == "cylinder")
  {
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[4])->setValue(
        _dimensions.x*0.5);
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[5])->setValue(
        _dimensions.z);
  }
  else if (_value == "sphere")
  {
    qobject_cast<QDoubleSpinBox *>(_widget->widgets[4])->setValue(
        _dimensions.x*0.5);
  }
  else if (_value == "polyline")
  {
    // do nothing
  }

  if (isMesh)
    qobject_cast<QLineEdit *>(_widget->widgets[6])->setText(tr(_uri.c_str()));

  emit GeometryChanged();
  return true;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateEnumWidget(ConfigChildWidget *_widget,
    const std::string &_value)
{
  if (_widget->widgets.size() != 1u)
  {
    gzerr << "Error updating Enum Config widget" << std::endl;
    return false;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(_widget->widgets[0]);
  if (!valueComboBox)
  {
    gzerr << "Error updating Enum Config widget" << std::endl;
    return false;
  }

  int index = valueComboBox->findText(tr(_value.c_str()));

  if (index < 0)
  {
    gzerr << "Error updating Enum Config widget: '" << _value <<
      "' not found" << std::endl;
    return false;
  }

  qobject_cast<QComboBox *>(_widget->widgets[0])->setCurrentIndex(index);

  return true;
}

/////////////////////////////////////////////////
bool ConfigWidget::UpdateDensityWidget(ConfigChildWidget *_widget,
          double _value)
{
  DensityConfigWidget *densityWidget =
      qobject_cast<DensityConfigWidget *>(_widget);

  if (densityWidget)
  {
    densityWidget->SetDensity(_value);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
int ConfigWidget::GetIntWidgetValue(ConfigChildWidget *_widget) const
{
  int value = 0;
  if (_widget->widgets.size() == 1u)
  {
    value = qobject_cast<QSpinBox *>(_widget->widgets[0])->value();
  }
  else
  {
    gzerr << "Error getting value from Int Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
unsigned int ConfigWidget::GetUIntWidgetValue(ConfigChildWidget *_widget) const
{
  unsigned int value = 0;
  if (_widget->widgets.size() == 1u)
  {
    value = qobject_cast<QSpinBox *>(_widget->widgets[0])->value();
  }
  else
  {
    gzerr << "Error getting value from UInt Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
double ConfigWidget::GetDoubleWidgetValue(ConfigChildWidget *_widget) const
{
  double value = 0.0;
  if (_widget->widgets.size() == 1u)
  {
    value = qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->value();
  }
  else
  {
    gzerr << "Error getting value from Double Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetStringWidgetValue(ConfigChildWidget *_widget) const
{
  std::string value;
  if (_widget->widgets.size() == 1u)
  {
    if (qobject_cast<QLineEdit *>(_widget->widgets[0]))
    {
      value =
          qobject_cast<QLineEdit *>(_widget->widgets[0])->text().toStdString();
    }
    else if (qobject_cast<QPlainTextEdit *>(_widget->widgets[0]))
    {
      value = qobject_cast<QPlainTextEdit *>(_widget->widgets[0])
          ->toPlainText().toStdString();
    }
  }
  else
  {
    gzerr << "Error getting value from String Config Widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
bool ConfigWidget::GetBoolWidgetValue(ConfigChildWidget *_widget) const
{
  bool value = false;
  if (_widget->widgets.size() == 2u)
  {
    value = qobject_cast<QRadioButton *>(_widget->widgets[0])->isChecked();
  }
  else
  {
    gzerr << "Error getting value from Bool Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
math::Vector3 ConfigWidget::GetVector3WidgetValue(ConfigChildWidget *_widget)
    const
{
  math::Vector3 value;
  if (_widget->widgets.size() == 3u)
  {
    value.x = qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->value();
    value.y = qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->value();
    value.z = qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->value();
  }
  else
  {
    gzerr << "Error getting value from Vector3 Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
common::Color ConfigWidget::GetColorWidgetValue(ConfigChildWidget *_widget)
    const
{
  common::Color value;
  if (_widget->widgets.size() == 4u)
  {
    value.r = qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->value();
    value.g = qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->value();
    value.b = qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->value();
    value.a = qobject_cast<QDoubleSpinBox *>(_widget->widgets[3])->value();
  }
  else
  {
    gzerr << "Error getting value from Color Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
math::Pose ConfigWidget::GetPoseWidgetValue(ConfigChildWidget *_widget) const
{
  math::Pose value;
  if (_widget->widgets.size() == 6u)
  {
    value.pos.x = qobject_cast<QDoubleSpinBox *>(_widget->widgets[0])->value();
    value.pos.y = qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->value();
    value.pos.z = qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->value();

    math::Vector3 rot;
    rot.x = qobject_cast<QDoubleSpinBox *>(_widget->widgets[3])->value();
    rot.y = qobject_cast<QDoubleSpinBox *>(_widget->widgets[4])->value();
    rot.z = qobject_cast<QDoubleSpinBox *>(_widget->widgets[5])->value();
    value.rot.SetFromEuler(rot);
  }
  else
  {
    gzerr << "Error getting value from Pose Config widget" << std::endl;
  }
  return value;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetGeometryWidgetValue(ConfigChildWidget *_widget,
    ignition::math::Vector3d &_dimensions, std::string &_uri) const
{
  std::string value;
  if (_widget->widgets.size() != 8u)
  {
    gzerr << "Error getting value from Geometry Config widget " << std::endl;
    return value;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(_widget->widgets[0]);
  value = valueComboBox->currentText().toStdString();

  bool isMesh = value == "mesh";
  if (value == "box" || isMesh)
  {
    _dimensions.X(qobject_cast<QDoubleSpinBox *>(_widget->widgets[1])->value());
    _dimensions.Y(qobject_cast<QDoubleSpinBox *>(_widget->widgets[2])->value());
    _dimensions.Z(qobject_cast<QDoubleSpinBox *>(_widget->widgets[3])->value());
  }
  else if (value == "cylinder")
  {
    _dimensions.X(
        qobject_cast<QDoubleSpinBox *>(_widget->widgets[4])->value()*2.0);
    _dimensions.Y(_dimensions.X());
    _dimensions.Z(qobject_cast<QDoubleSpinBox *>(_widget->widgets[5])->value());
  }
  else if (value == "sphere")
  {
    _dimensions.X(
        qobject_cast<QDoubleSpinBox *>(_widget->widgets[4])->value()*2.0);
    _dimensions.Y(_dimensions.X());
    _dimensions.Z(_dimensions.X());
  }
  else if (value == "polyline")
  {
    // do nothing
  }
  else
  {
    gzerr << "Error getting geometry dimensions for type: '" << value << "'"
        << std::endl;
  }

  if (isMesh)
    _uri = qobject_cast<QLineEdit *>(_widget->widgets[6])->text().toStdString();

  return value;
}

/////////////////////////////////////////////////
std::string ConfigWidget::GetEnumWidgetValue(ConfigChildWidget *_widget) const
{
  std::string value;
  if (_widget->widgets.size() != 1u)
  {
    gzerr << "Error getting value from Enum Config widget " << std::endl;
    return value;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(_widget->widgets[0]);
  value = valueComboBox->currentText().toStdString();

  return value;
}

/////////////////////////////////////////////////
void ConfigWidget::OnItemSelection(QTreeWidgetItem *_item,
                                         int /*_column*/)
{
  if (_item && _item->childCount() > 0)
    _item->setExpanded(!_item->isExpanded());
}

/////////////////////////////////////////////////
void ConfigWidget::OnUIntValueChanged()
{
  QSpinBox *spin =
      qobject_cast<QSpinBox *>(QObject::sender());

  if (!spin)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(spin->parent());

  if (!widget)
    return;

  emit UIntValueChanged(widget->scopedName.c_str(),
      this->GetUIntWidgetValue(widget));
}

/////////////////////////////////////////////////
void ConfigWidget::OnIntValueChanged()
{
  QSpinBox *spin =
      qobject_cast<QSpinBox *>(QObject::sender());

  if (!spin)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(spin->parent());

  if (!widget)
    return;

  emit IntValueChanged(widget->scopedName.c_str(),
      this->GetIntWidgetValue(widget));
}

/////////////////////////////////////////////////
void ConfigWidget::OnDoubleValueChanged()
{
  QDoubleSpinBox *spin =
      qobject_cast<QDoubleSpinBox *>(QObject::sender());

  if (!spin)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(spin->parent());

  if (!widget)
    return;

  emit DoubleValueChanged(widget->scopedName.c_str(),
      this->GetDoubleWidgetValue(widget));
}

/////////////////////////////////////////////////
void ConfigWidget::OnBoolValueChanged()
{
  QRadioButton *radio =
      qobject_cast<QRadioButton *>(QObject::sender());

  if (!radio)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(radio->parent());

  if (!widget)
    return;

  emit BoolValueChanged(widget->scopedName.c_str(),
      this->GetBoolWidgetValue(widget));
}

/////////////////////////////////////////////////
void ConfigWidget::OnStringValueChanged()
{
  QLineEdit *lineEdit = qobject_cast<QLineEdit *>(QObject::sender());
  QPlainTextEdit *plainTextEdit =
      qobject_cast<QPlainTextEdit *>(QObject::sender());

  QWidget *valueEdit;
  if (!lineEdit && !plainTextEdit)
    return;
  else if (lineEdit)
    valueEdit = lineEdit;
  else
    valueEdit = plainTextEdit;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(valueEdit->parent());

  if (!widget)
    return;

  emit StringValueChanged(widget->scopedName.c_str(),
      this->GetStringWidgetValue(widget));
}

/////////////////////////////////////////////////
void ConfigWidget::OnVector3dValueChanged()
{
  QDoubleSpinBox *spin =
      qobject_cast<QDoubleSpinBox *>(QObject::sender());

  if (!spin)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(spin->parent());

  if (!widget)
    return;

  emit Vector3dValueChanged(widget->scopedName.c_str(),
      this->GetVector3WidgetValue(widget).Ign());
}

/////////////////////////////////////////////////
void ConfigWidget::OnColorValueChanged()
{
  QDoubleSpinBox *spin =
      qobject_cast<QDoubleSpinBox *>(QObject::sender());

  if (!spin)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(spin->parent());

  if (!widget)
    return;

  emit ColorValueChanged(widget->scopedName.c_str(),
      this->GetColorWidgetValue(widget));
}

/////////////////////////////////////////////////
void ConfigWidget::OnPoseValueChanged()
{
  QDoubleSpinBox *spin =
      qobject_cast<QDoubleSpinBox *>(QObject::sender());

  if (!spin)
    return;

  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(spin->parent());

  if (!widget)
    return;

  emit PoseValueChanged(widget->scopedName.c_str(),
      this->GetPoseWidgetValue(widget).Ign());
}

/////////////////////////////////////////////////
void ConfigWidget::OnEnumValueChanged(const QString &_value)
{
  ConfigChildWidget *widget =
      qobject_cast<ConfigChildWidget *>(QObject::sender());

  if (!widget)
    return;

  emit EnumValueChanged(widget->scopedName.c_str(), _value);
}

/////////////////////////////////////////////////
bool ConfigWidget::AddConfigChildWidget(const std::string &_name,
    ConfigChildWidget *_child)
{
  if (_name.empty() || _child == NULL)
  {
    gzerr << "Given name or child is invalid. Not adding child widget."
          << std::endl;
    return false;
  }
  if (this->configWidgets.find(_name) != this->configWidgets.end())
  {
    gzerr << "This config widget already has a child with that name. " <<
       "Names must be unique. Not adding child." << std::endl;
    return false;
  }

  _child->scopedName = _name;
  this->configWidgets[_name] = _child;
  return true;
}

/////////////////////////////////////////////////
unsigned int ConfigWidget::ConfigChildWidgetCount() const
{
  return this->configWidgets.size();
}

/////////////////////////////////////////////////
bool ConfigWidget::eventFilter(QObject *_obj, QEvent *_event)
{
  QAbstractSpinBox *spinBox = qobject_cast<QAbstractSpinBox *>(_obj);
  QComboBox *comboBox = qobject_cast<QComboBox *>(_obj);
  if (spinBox || comboBox)
  {
    QWidget *widget = qobject_cast<QWidget *>(_obj);
    if (_event->type() == QEvent::Wheel)
    {
      if (widget->focusPolicy() == Qt::WheelFocus)
      {
        _event->accept();
        return false;
      }
      else
      {
        _event->ignore();
        return true;
      }
    }
    else if (_event->type() == QEvent::FocusIn)
    {
      widget->setFocusPolicy(Qt::WheelFocus);
    }
    else if (_event->type() == QEvent::FocusOut)
    {
      widget->setFocusPolicy(Qt::StrongFocus);
    }
  }
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void ConfigWidget::OnDensityValueChanged(const double &_value)
{
  emit DensityValueChanged(_value);
}

/////////////////////////////////////////////////
void ConfigWidget::OnMassValueChanged(double _value)
{
  emit MassValueChanged(_value);
}


void ConfigWidget::OnGeometryChanged()
{
  emit GeometryChanged();
}

/////////////////////////////////////////////////
void GroupWidget::Toggle(bool _checked)
{
  if (!this->childWidget)
    return;

  this->childWidget->setVisible(_checked);
}

/////////////////////////////////////////////////
void GeometryConfigWidget::OnGeometryTypeChanged(const QString _text)
{
  QWidget *widget= qobject_cast<QWidget *>(QObject::sender());

  if (widget)
  {
    std::string textStr = _text.toStdString();
    bool isMesh = (textStr == "mesh");
    if (textStr == "box" || isMesh)
    {
      this->geomDimensionWidget->show();
      this->geomDimensionWidget->setCurrentIndex(0);
    }
    else if (textStr == "cylinder")
    {
      this->geomDimensionWidget->show();
      this->geomDimensionWidget->setCurrentIndex(1);
      this->geomLengthSpinBox->show();
      this->geomLengthLabel->show();
      this->geomLengthUnitLabel->show();
    }
    else if (textStr == "sphere")
    {
      this->geomDimensionWidget->show();
      this->geomDimensionWidget->setCurrentIndex(1);
      this->geomLengthSpinBox->hide();
      this->geomLengthLabel->hide();
      this->geomLengthUnitLabel->hide();
    }
    else if (textStr == "polyline")
    {
      this->geomDimensionWidget->hide();
    }

    this->geomFilenameLabel->setVisible(isMesh);
    this->geomFilenameLineEdit->setVisible(isMesh);
    this->geomFilenameButton->setVisible(isMesh);
  }
  emit GeometryChanged();
}

void GeometryConfigWidget::OnGeometrySizeChanged(double /*_value*/)
{
  emit GeometryChanged();
}

/////////////////////////////////////////////////
void GeometryConfigWidget::OnSelectFile()
{
  QWidget *widget= qobject_cast<QWidget *>(QObject::sender());

  if (widget)
  {
    QFileDialog fd(this, tr("Select mesh file"), QDir::homePath(),
      tr("Mesh files (*.dae *.stl)"));
    fd.setFilter(QDir::AllDirs | QDir::Hidden);
    fd.setFileMode(QFileDialog::ExistingFile);
    fd.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
        Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);
    if (fd.exec())
    {
      if (!fd.selectedFiles().isEmpty())
      {
        QString file = fd.selectedFiles().at(0);
        if (!file.isEmpty())
        {
          dynamic_cast<QLineEdit *>(this->geomFilenameLineEdit)->setText(file);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void DensityConfigWidget::OnComboBoxChanged(const QString /*_text*/)
{
  QVariant variant = this->comboBox->itemData(this->comboBox->currentIndex());
  this->SetDensity(variant.toDouble());
}

/////////////////////////////////////////////////
void DensityConfigWidget::OnSpinBoxChanged(const QString /*_text*/)
{
  this->SetDensity(this->spinBox->value());
}

/////////////////////////////////////////////////
void DensityConfigWidget::SetDensity(double _density)
{
  bool comboSigState = this->comboBox->blockSignals(true);
  bool spinSigState = this->spinBox->blockSignals(true);
  {
    const DensityEntry *entry =
        DensityModel::Instance()->EntryByValue(_density);

    if (entry)
      this->comboBox->setCurrentIndex(
          this->comboBox->findText(tr(entry->desc.c_str())));
    else
      this->comboBox->setCurrentIndex(
          this->comboBox->count()-1);

    this->spinBox->setValue(_density);
    this->density = _density;
  }
  this->comboBox->blockSignals(comboSigState);
  this->spinBox->blockSignals(spinSigState);

  emit DensityValueChanged(this->density);
}

/////////////////////////////////////////////////
double DensityConfigWidget::Density() const
{
  return this->density;
}

/////////////////////////////////////////////////
void EnumConfigWidget::EnumChanged(const QString &_value)
{
  emit EnumValueChanged(_value);
}

/////////////////////////////////////////////////
bool ConfigWidget::ClearEnumWidget(const std::string &_name)
{
  // Find widget
  auto iter = this->configWidgets.find(_name);

  if (iter == this->configWidgets.end())
    return false;

  EnumConfigWidget *enumWidget = dynamic_cast<EnumConfigWidget *>(iter->second);

  if (enumWidget->widgets.size() != 1u)
  {
    gzerr << "Enum config widget has wrong number of widgets." << std::endl;
    return false;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(enumWidget->widgets[0]);
  if (!valueComboBox)
  {
    gzerr << "Enum config widget doesn't have a QComboBox." << std::endl;
    return false;
  }

  // Clear
  valueComboBox->blockSignals(true);
  valueComboBox->clear();
  valueComboBox->blockSignals(false);
  return true;
}

/////////////////////////////////////////////////
bool ConfigWidget::AddItemEnumWidget(const std::string &_name,
    const std::string &_itemText)
{
  // Find widget
  auto iter = this->configWidgets.find(_name);

  if (iter == this->configWidgets.end())
    return false;

  EnumConfigWidget *enumWidget = dynamic_cast<EnumConfigWidget *>(iter->second);

  if (enumWidget->widgets.size() != 1u)
  {
    gzerr << "Enum config widget has wrong number of widgets." << std::endl;
    return false;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(enumWidget->widgets[0]);
  if (!valueComboBox)
  {
    gzerr << "Enum config widget doesn't have a QComboBox." << std::endl;
    return false;
  }

  // Add item
  valueComboBox->blockSignals(true);
  valueComboBox->addItem(QString::fromStdString(_itemText));
  valueComboBox->blockSignals(false);

  return true;
}

/////////////////////////////////////////////////
bool ConfigWidget::RemoveItemEnumWidget(const std::string &_name,
    const std::string &_itemText)
{
  // Find widget
  auto iter = this->configWidgets.find(_name);

  if (iter == this->configWidgets.end())
    return false;

  EnumConfigWidget *enumWidget = dynamic_cast<EnumConfigWidget *>(iter->second);

  if (enumWidget->widgets.size() != 1u)
  {
    gzerr << "Enum config widget has wrong number of widgets." << std::endl;
    return false;
  }

  QComboBox *valueComboBox = qobject_cast<QComboBox *>(enumWidget->widgets[0]);
  if (!valueComboBox)
  {
    gzerr << "Enum config widget doesn't have a QComboBox." << std::endl;
    return false;
  }

  // Remove item if exists, otherwise return false
  int index = valueComboBox->findText(QString::fromStdString(
      _itemText));
  if (index < 0)
    return false;

  valueComboBox->blockSignals(true);
  valueComboBox->removeItem(index);
  valueComboBox->blockSignals(false);

  return true;
}

/////////////////////////////////////////////////
void ConfigWidget::InsertLayout(QLayout *_layout, int _pos)
{
  QGroupBox *box = qobject_cast<QGroupBox *>(
      this->layout()->itemAt(0)->widget());
  if (!box)
    return;

  QVBoxLayout *boxLayout = qobject_cast<QVBoxLayout *>(box->layout());
  if (!boxLayout)
    return;

  boxLayout->insertLayout(_pos, _layout);
}

/////////////////////////////////////////////////
QString ConfigWidget::StyleSheet(const std::string &_type, const int _level)
{
  if (_type == "normal")
  {
    return "QWidget\
        {\
          background-color: " + ConfigWidget::bgColors[_level] + ";\
          color: #4c4c4c;\
        }\
        QLabel\
        {\
          color: #d0d0d0;\
        }\
        QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
        {\
          background-color: " + ConfigWidget::widgetColors[_level] +
        "}";
  }
  else if (_type == "warning")
  {
    return "QWidget\
      {\
        background-color: " + ConfigWidget::bgColors[_level] + ";\
        color: " + ConfigWidget::redColor + ";\
      }\
      QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox\
      {\
        background-color: " + ConfigWidget::widgetColors[_level] +
      "}";
  }
  gzwarn << "Requested unknown style sheet type [" << _type << "]" << std::endl;
  return "";
}
