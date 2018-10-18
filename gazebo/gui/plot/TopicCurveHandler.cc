/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <google/protobuf/message.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/TopicCurveHandler.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the TopicCurveHandler class.
    class TopicCurveHandlerPrivate
    {
      /// \brief Topic name
      public: std::string topic;

      /// \brief Message type for the topic.
      public: std::string msgType;

      /// \brief A map of param names to plot curves.
      public: std::map<std::string, CurveVariableSet> curves;

      /// \brief Subscriber to specified topic
      public: transport::SubscriberPtr subscriber;

      /// \brief Node for communications.
      public: transport::NodePtr node;

      /// \brief Mutex to protect the topic data updates.
      public: std::mutex mutex;
    };
  }
}

/////////////////////////////////////////////////
TopicCurveHandler::TopicCurveHandler()
  : dataPtr(new TopicCurveHandlerPrivate())
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
}

/////////////////////////////////////////////////
TopicCurveHandler::~TopicCurveHandler()
{
}

/////////////////////////////////////////////////
void TopicCurveHandler::SetTopic(const std::string &_topic)
{
  this->dataPtr->topic = _topic;

  this->dataPtr->msgType = transport::getTopicMsgType(this->dataPtr->topic);
  if (this->dataPtr->msgType == "")
  {
    gzwarn << "Couldn't find message type for topic [" <<
        this->dataPtr->topic << "]" << std::endl;
    return;
  }

  this->dataPtr->subscriber = this->dataPtr->node->Subscribe(
      this->dataPtr->topic, &TopicCurveHandler::OnTopicData, this);
}

/////////////////////////////////////////////////
void TopicCurveHandler::AddCurve(const std::string &_param,
    PlotCurveWeakPtr _curve)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->curves.find(_param);
  if (it == this->dataPtr->curves.end())
  {
    CurveVariableSet curveSet;
    curveSet.insert(_curve);
    this->dataPtr->curves[_param] = curveSet;
  }
  else
  {
    auto cIt = it->second.find(_curve);
    if (cIt == it->second.end())
    {
      it->second.insert(_curve);
    }
  }
}

/////////////////////////////////////////////////
void TopicCurveHandler::RemoveCurve(PlotCurveWeakPtr _curve)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (auto it = this->dataPtr->curves.begin();
      it != this->dataPtr->curves.end(); ++it)
  {
    auto cIt = it->second.find(_curve);
    if (cIt != it->second.end())
    {
      it->second.erase(cIt);
      if (it->second.empty())
        this->dataPtr->curves.erase(it);
      return;
    }
  }
}

/////////////////////////////////////////////////
bool TopicCurveHandler::HasCurve(PlotCurveWeakPtr _curve) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  for (const auto &it : this->dataPtr->curves)
  {
    if (it.second.find(_curve) != it.second.end())
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
unsigned int TopicCurveHandler::CurveCount() const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  unsigned int count = 0;
  for (const auto &it : this->dataPtr->curves)
    count += it.second.size();
  return count;
}

/////////////////////////////////////////////////
void TopicCurveHandler::OnTopicData(const std::string &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->curves.empty())
    return;

  auto msg = msgs::MsgFactory::NewMsg(this->dataPtr->msgType);
  msg->ParseFromString(_msg);

  this->UpdateCurve(msg.get());
}

/////////////////////////////////////////////////
void TopicCurveHandler::UpdateCurve(google::protobuf::Message *_msg,
    const int _index)
{
  auto ref = _msg->GetReflection();
  if (!ref)
    return;

  auto descriptor = _msg->GetDescriptor();
  if (!descriptor)
    return;

  for (int i = 0; i < descriptor->field_count(); ++i)
  {
    auto field = descriptor->field(i);
    if (!field)
      continue;

    std::string name = field->name();

    // TODO look only at subset of curves instead of everything?
    for (const auto it : this->dataPtr->curves)
    {
      std::string param = it.first;
      // TODO parse param to get the attribute at _index
      std::string attribute;
      if (attribute == name)
      {
        switch (field->type())
        {
          case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
          {
            double value = ref->GetDouble(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_FLOAT:
          {
            float value = ref->GetFloat(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_INT64:
          {
            int64_t value = ref->GetInt64(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_UINT64:
          {
            uint64_t value = ref->GetUInt64(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_INT32:
          {
            int32_t value = ref->GetInt32(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_UINT32:
          {
            uint32_t value = ref->GetUInt32(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_BOOL:
          {
            bool value = ref->GetBool(*_msg, field);
            break;
          }
          case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
          {
            auto valueMsg = ref->MutableMessage(_msg, field);

            if (field->message_type()->name() == "Time")
            {
              auto valueDescriptor = valueMsg->GetDescriptor();

              auto secValueField = valueDescriptor->field(0);
              auto nsecValueField = valueDescriptor->field(1);
              double sec = valueMsg->GetReflection()->GetDouble(
                    *valueMsg, secValueField);
              double nsec = valueMsg->GetReflection()->GetDouble(
                    *valueMsg, nsecValueField);

              double value = sec + nsec*1e9;
            }
            else if (field->message_type()->name() == "Vector3d")
            {
              auto valueDescriptor = valueMsg->GetDescriptor();

              // TODO parse attribute to get x, y, or z at leaf of uri
              std::string p;

              auto valueField = valueDescriptor->FindFieldByName(p);
              double value =
                  valueMsg->GetReflection()->GetDouble(*valueMsg, valueField);
            }
            else if (field->message_type()->name() ==
                "Quaternion")
            {
              auto valueDescriptor = valueMsg->GetDescriptor();

              // TODO parse attribute to get roll, pitch, or yaw at leaf of uri
              std::string p;

              std::vector<double> quatValues;
              for (unsigned int j = 0; j < 4; ++j)
              {
                auto quatValueField = valueDescriptor->field(j);
                quatValues.push_back(valueMsg->GetReflection()->GetDouble(
                    *valueMsg, quatValueField));
              }
              ignition::math::Quaterniond quat(quatValues[3], quatValues[0],
                  quatValues[1], quatValues[2]);

              ignition::math::Vector3d rpy = quat.Euler();
              double value = 0;
              if (p == "roll")
                value = rpy.X();
              else if (p == "pitch")
                value = rpy.Y();
              else if (p == "yaw")
                value = rpy.Z();
            }
            else
            {
              this->UpdateCurve(valueMsg, _index + 1);
            }
            break;
          }
          default:
            break;
        }
      }
    }
  }
}
