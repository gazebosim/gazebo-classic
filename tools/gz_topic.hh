/*
 * Copyright 2015 Open Source Robotics Foundation
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
#ifndef _GZ_TOPIC_HH_
#define _GZ_TOPIC_HH_

#include <string>
#include <vector>

#include "gz.hh"

namespace gazebo
{
  /// \brief Topic command
  class TopicCommand : public Command
  {
    /// \brief Constructor
    public: TopicCommand();

    // Documentation inherited
    public: virtual void HelpDetailed();

    // Documentation inherited
    protected: virtual bool RunImpl();

    /// \brief Output the topic list.
    private: void List();

    /// \brief Output information about a topic.
    /// \param[in] _topic Topic to print info about.
    private: void Info(const std::string &_topic);

    /// \brief Output messages from a topic.
    /// \param[in] _topic Topic to print messages from.
    private: void Echo(const std::string &_topic);

    /// \brief Get a TopicInfo message from a topic.
    /// \param[in] _topic Topic to get info about.
    /// \return The TopicInfo message about the _topic.
    private: msgs::TopicInfo GetInfo(const std::string &_topic);

    /// \brief Callback used by Echo() to receive topic messages.
    /// \param[in] _data Data message from a topic.
    private: void EchoCB(const std::string &_data);

    /// \brief Callback used by Hz() to receive topic messages.
    /// \param[in] _data Data message from a topic (unused).
    private: void HzCB(const std::string &_data);

    /// \brief Output Hz rate for a topic.
    /// \param[in] _topic Topic name.
    private: void Hz(const std::string &_topic);

    /// \brief Subscription callback used by Bw().
    /// \param[in] _data Message data.
    private: void BwCB(const std::string &_data);

    /// \brief Output bandwidth data for a topic.
    /// \param[in] _topic Topic name.
    private: void Bw(const std::string &_topic);

    /// \brief View topic information using QT.
    /// \param[in] _topic Name of the topic to view. Empty will bring up
    /// a topic selector.
    private: void View(const std::string &_topic);

    /// \brief Message used to hold data received from EchoCB().
    private: boost::shared_ptr<google::protobuf::Message> echoMsg;

    /// \brief Node pointer.
    private: transport::NodePtr node;

    /// \brief Prev time a message was received.
    private: common::Time prevMsgTime;

    /// \brief Buffer of message sizes, used by Bw().
    private: std::vector<int> bwBytes;

    /// \brief Buffer of message publish times, used by Bw().
    private: std::vector<common::Time> bwTime;
  };
}
#endif
