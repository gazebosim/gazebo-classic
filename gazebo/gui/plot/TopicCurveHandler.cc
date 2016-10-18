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

#include <mutex>

#include <google/protobuf/message.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/common/URI.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SingletonT.hh"

#include "gazebo/gui/plot/PlottingTypes.hh"
#include "gazebo/gui/plot/PlotCurve.hh"
#include "gazebo/gui/plot/TopicCurveHandler.hh"

using namespace gazebo;
using namespace gui;

namespace gazebo
{
  namespace gui
  {
    /// \brief Time data for topic messages
    /// Gazebo messages are not timestamped so as a workaround this class
    /// tries to match the messages against the last sim time received.
    /// WARNING: This produces plots with inaccurate x values!
    /// A better solution is to properly timestamp the messages.
    class TopicTime : public SingletonT<TopicTime>
    {
      /// \brief Initialize the topic time helper class.
      public: void Init();

      /// \brief Get last sim time.
      /// \return last sim time received.
      public: common::Time LastSimTime();

      /// \brief Callback when world stats message is receieved.
      /// \param[in] _msg WorldStats msg.
      public: void OnStats(ConstWorldStatisticsPtr &_msg);

      /// \brief Node for communications.
      private: transport::NodePtr node;

      /// \brief Mutex to protect the sime time updates.
      private: std::mutex mutex;

      /// \brief Subscriber to specified topic
      private: transport::SubscriberPtr subscriber;

      /// \brief Last sim time received.
      private: common::Time lastSimTime;
    };

    /// \brief Helper class to update curves associated with a single topic
    class TopicCurve
    {
      /// \def CurveVariableMapIt
      /// \brief Curve variable map iterator
      public: using CurveVariableMapIt =
          std::map<std::string, CurveVariableSet>::iterator;

      /// \brief Constructor.
      public: TopicCurve(const std::string &_topic);

      /// \brief Destructor.
      public: ~TopicCurve();

      /// \brief Add a curve to be updated
      /// \param[in] _query URI query string containing the param the curve is
      /// associated with.
      /// \param[in] _curve Pointer to the plot curve to add.
      public: bool AddCurve(const std::string &_query, PlotCurveWeakPtr _curve);

      /// \brief Remove a curve from the topic data handler
      /// \param[in] _curve Pointer to the plot curve to remove.
      public: bool RemoveCurve(PlotCurveWeakPtr _curve);

      /// \brief Get whether the topic curve has the specified plot curve.
      /// \param[in] _curve Pointer to the plot curve
      /// \return True if curve exists
      public: bool HasCurve(PlotCurveWeakPtr _curve) const;

      /// \brief Get the number of curves managed by this topic curve class.
      /// \return Number of curves
      public: unsigned int CurveCount() const;

      /// \brief Topic data callback
      /// \param[in] _msg Message data
      public: void OnTopicData(const std::string &_msg);

      /// \brief Update the plot curve based on message
      /// \param[in] _msg Message containing data to be added to the curve
      /// \param[in] _index Index of token in the param path string
      /// \param[in] _x X value. Only used if the topic data is not timestamped
      /// \param[in] _curvesUpdates A list of curves and values to update
      public: void UpdateCurve(google::protobuf::Message *_msg,
                  const int _index, const double _x,
                  std::vector<std::pair<TopicCurve::CurveVariableMapIt,
                      ignition::math::Vector2d> > &_curvesUpdates);

      /// \brief Topic name
      private: std::string topic;

      /// \brief Subscriber to specified topic
      private: transport::SubscriberPtr subscriber;

      /// \brief Node for communications.
      private: transport::NodePtr node;

      /// \brief Mutex to protect the topic data updates.
      private: mutable std::mutex mutex;

      /// \brief Topic message type.
      private: std::string msgType;

      /// \brief A map of param names to plot curves.
      private: std::map<std::string, CurveVariableSet> curves;
    };

    /// \brief Private data for the TopicCurveHandler class.
    class TopicCurveHandlerPrivate
    {
      /// \brief A map of unique topics to topic curves.
      public: std::map<std::string, TopicCurve *> topics;
    };
  }
}

/////////////////////////////////////////////////
void TopicTime::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->subscriber = this->node->Subscribe("~/world_stats",
      &TopicTime::OnStats, this);
}

/////////////////////////////////////////////////
void TopicTime::OnStats(ConstWorldStatisticsPtr &_msg)
{
  if (!_msg)
  {
    GZ_ASSERT(_msg, "_msg pointer in OnStats method should not be null");
    return;
  }

  msgs::Time t = _msg->sim_time();

  std::lock_guard<std::mutex> lock(this->mutex);
  this->lastSimTime = msgs::Convert(t);
}

/////////////////////////////////////////////////
common::Time TopicTime::LastSimTime()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  return this->lastSimTime;
}

/////////////////////////////////////////////////
TopicCurve::TopicCurve(const std::string &_topic)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->topic = _topic;

  this->msgType = transport::getTopicMsgType(this->topic);
  if (this->msgType == "")
  {
    gzwarn << "Couldn't find message type for topic [" <<
        this->topic << "]" << std::endl;
    return;
  }

  this->subscriber = this->node->Subscribe(
      this->topic, &TopicCurve::OnTopicData, this);
}

/////////////////////////////////////////////////
TopicCurve::~TopicCurve()
{
  this->subscriber.reset();
  this->node.reset();
}

/////////////////////////////////////////////////
bool TopicCurve::AddCurve(const std::string &_name, PlotCurveWeakPtr _curve)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  auto c = _curve.lock();
  if (!c)
    return false;

  common::URI topicURI(_name);
  if (!topicURI.Valid())
  {
    gzwarn << "topicURI '" << topicURI.Str() <<
              "' is invalid" << std::endl;
    return false;
  }

  common::URIQuery topicQuery = topicURI.Query();
  std::string topicQueryStr = topicQuery.Str();

  auto it = this->curves.find(topicQueryStr);
  if (it == this->curves.end())
  {
    // create entry in map
    CurveVariableSet curveSet;
    curveSet.insert(_curve);
    this->curves[topicQueryStr] = curveSet;
  }
  else
  {
    auto cIt = it->second.find(_curve);
    if (cIt == it->second.end())
    {
      it->second.insert(_curve);
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool TopicCurve::RemoveCurve(PlotCurveWeakPtr _curve)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  auto c = _curve.lock();
  if (!c)
    return false;

  for (auto it = this->curves.begin(); it != this->curves.end(); ++it)
  {
    auto cIt = it->second.find(_curve);
    if (cIt != it->second.end())
    {
      it->second.erase(cIt);
      if (it->second.empty())
      {
        this->curves.erase(it);
      }
      return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
bool TopicCurve::HasCurve(PlotCurveWeakPtr _curve) const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &it : this->curves)
  {
    if (it.second.find(_curve) != it.second.end())
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
unsigned int TopicCurve::CurveCount() const
{
  std::lock_guard<std::mutex> lock(this->mutex);
  unsigned int count = 0;
  for (const auto &it : this->curves)
    count += it.second.size();
  return count;
}

/////////////////////////////////////////////////
void TopicCurve::OnTopicData(const std::string &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  if (this->curves.empty())
    return;

  auto msg = msgs::MsgFactory::NewMsg(this->msgType);
  msg->ParseFromString(_msg);

  // stores a list of curve iterators and their new values
  std::vector<std::pair<TopicCurve::CurveVariableMapIt,
      ignition::math::Vector2d> > curvesUpdates;

  // nearest sim time - use this x value if the message is not timestamped
  double x = TopicTime::Instance()->LastSimTime().Double();

  // collect updates
  this->UpdateCurve(msg.get(), 0, x, curvesUpdates);

  // update curves!
  for (auto &curveUpdate : curvesUpdates)
  {
    for (auto &cIt : curveUpdate.first->second)
    {
      auto curve = cIt.lock();
      if (!curve)
        continue;

      curve->AddPoint(curveUpdate.second);
    }
  }
}

/////////////////////////////////////////////////
void TopicCurve::UpdateCurve(google::protobuf::Message *_msg,
    const int _index, const double x,
    std::vector<std::pair<TopicCurve::CurveVariableMapIt,
    ignition::math::Vector2d> > &_curvesUpdates)
{
  if (!_msg)
  {
    GZ_ASSERT(_msg,
        "_msg pointer in TopicCurve::UpdateCurve should not be null");
  }

  auto ref = _msg->GetReflection();
  if (!ref)
    return;

  auto descriptor = _msg->GetDescriptor();
  if (!descriptor)
    return;

  // x axis data
  double xData = x;

  for (int i = 0; i < descriptor->field_count(); ++i)
  {
    auto field = descriptor->field(i);
    if (!field)
      continue;

    std::string fieldName = field->name();

    // Check if message has timestamp and use it if it exists and is
    // a top level msg field.
    // TODO x axis is hardcoded to be the sim time for now. Once it is
    // configurable, remove this logic for setting the x value
    if (_index == 0 && (fieldName == "stamp" || fieldName == "time") &&
        field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE &&
        !field->is_repeated())
    {
      auto valueMsg = ref->MutableMessage(_msg, field);
      if (field->message_type()->name() == "Time")
      {
        msgs::Time *msg = dynamic_cast<msgs::Time *>(valueMsg);
        if (msg)
        {
          common::Time time = msgs::Convert(*msg);
          xData = time.Double();
        }
      }
    }

    // loop through all the topic param name + curve pairs
    for (auto cIt = this->curves.begin(); cIt != this->curves.end(); ++cIt)
    {
      // parse param to get field at current index
      std::string topicField;

      std::string query = cIt->first;

      // tokenize query, e.g. ?p=sim_time -> [?p, sim_time]
      std::vector<std::string> queryTokens = common::split(query, "=/");

      // skip ?p
      unsigned int queryIndex = _index + 1;

      if (queryTokens.size() < 2 || queryTokens.size() <= queryIndex)
        continue;

      topicField = queryTokens[queryIndex];

      double data = 0;
      bool addData = true;
      if (topicField != fieldName)
        continue;

      switch (field->type())
      {
        case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
        {
          data = ref->GetDouble(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_FLOAT:
        {
          data = ref->GetFloat(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT64:
        {
          data = ref->GetInt64(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT64:
        {
          data = ref->GetUInt64(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_INT32:
        {
          data = ref->GetInt32(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_UINT32:
        {
          data = ref->GetUInt32(*_msg, field);
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_BOOL:
        {
          data = static_cast<int>(ref->GetBool(*_msg, field));
          break;
        }
        case google::protobuf::FieldDescriptor::TYPE_MESSAGE:
        {
          auto valueMsg = ref->MutableMessage(_msg, field);

          if (field->message_type()->name() == "Time")
          {
            msgs::Time *msg = dynamic_cast<msgs::Time *>(valueMsg);

            if (!msg)
              continue;

             common::Time time = msgs::Convert(*msg);
             data = time.Double();
          }
          else if (field->message_type()->name() == "Vector3d")
          {
            msgs::Vector3d *msg = dynamic_cast<msgs::Vector3d *>(valueMsg);

            if (!msg)
              continue;

            ignition::math::Vector3d vec = msgs::ConvertIgn(*msg);

            // parse param to get x, y, or z at leaf of query
            std::string elem = query.substr(query.size()-1);
            if (elem == "x")
            {
              data = vec.X();
            }
            else if (elem == "y")
            {
              data = vec.Y();
            }
            else if (elem == "z")
            {
              data = vec.Z();
            }
            else
              continue;
          }
          else if (field->message_type()->name() == "Quaternion")
          {
            msgs::Quaternion *msg =
                dynamic_cast<msgs::Quaternion *>(valueMsg);

            if (!msg)
              continue;

            ignition::math::Quaterniond quat = msgs::ConvertIgn(*msg);

            // parse query to get roll, pitch, or yaw at leaf of uri
            ignition::math::Vector3d rpy = quat.Euler();
            if (query.find("roll") != std::string::npos)
              data = rpy.X();
            else if (query.find("pitch") != std::string::npos)
              data = rpy.Y();
            else if (query.find("yaw") != std::string::npos)
              data = rpy.Z();
            else
              continue;
          }
          else
          {
            this->UpdateCurve(valueMsg, _index + 1, xData, _curvesUpdates);
            addData = false;
          }
          break;
        }
        default:
        {
          continue;
        }
      // end switch
      }

      // push to tmp list and update later
      if (addData)
      {
        _curvesUpdates.push_back(
            std::make_pair(cIt, ignition::math::Vector2d(xData, data)));
      }
    }
  }
}

/////////////////////////////////////////////////
TopicCurveHandler::TopicCurveHandler()
  : dataPtr(new TopicCurveHandlerPrivate())
{
  TopicTime::Instance()->Init();
}

/////////////////////////////////////////////////
TopicCurveHandler::~TopicCurveHandler()
{
  while (!this->dataPtr->topics.empty())
  {
    auto it = this->dataPtr->topics.begin();
    delete it->second;
    this->dataPtr->topics.erase(it);
  }

  this->dataPtr->topics.clear();
}

/////////////////////////////////////////////////
void TopicCurveHandler::AddCurve(const std::string &_name,
    PlotCurveWeakPtr _curve)
{
  // append scheme to make it a valid uri so we can parse the string
  std::string uriName = "topic://" + _name;

  common::URI topicURI(uriName);
  if (!topicURI.Valid())
  {
    gzwarn << "topicURI '" << topicURI.Str().c_str() <<
              "' is invalid" << std::endl;
    return;
  }

  common::URIPath topicPath = topicURI.Path();
  // append '/' to make it a valid topic name
  std::string topicPathStr = "/" + topicPath.Str();

  auto topicCurveIt = this->dataPtr->topics.find(topicPathStr);
  if (topicCurveIt == this->dataPtr->topics.end())
  {
    TopicCurve *topicCurve = new TopicCurve(topicPathStr);
    bool result = topicCurve->AddCurve(uriName, _curve);
    if (result)
      this->dataPtr->topics[topicPathStr] = topicCurve;
    else
      delete topicCurve;
  }
  else
  {
    topicCurveIt->second->AddCurve(uriName, _curve);
  }
}

/////////////////////////////////////////////////
void TopicCurveHandler::RemoveCurve(PlotCurveWeakPtr _curve)
{
  for (auto it = this->dataPtr->topics.begin();
      it !=this->dataPtr->topics.end(); ++it)
  {
    if (it->second->HasCurve(_curve))
    {
      it->second->RemoveCurve(_curve);
      if (it->second->CurveCount() == 0)
      {
        delete it->second;
        this->dataPtr->topics.erase(it);
      }
      break;
    }
  }
}

/////////////////////////////////////////////////
bool TopicCurveHandler::HasCurve(PlotCurveWeakPtr _curve) const
{
  for (const auto &it : this->dataPtr->topics)
  {
    if (it.second->HasCurve(_curve))
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
unsigned int TopicCurveHandler::CurveCount() const
{
  unsigned int count = 0;
  for (const auto &it : this->dataPtr->topics)
    count += it.second->CurveCount();
  return count;
}
