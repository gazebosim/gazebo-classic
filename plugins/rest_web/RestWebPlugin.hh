/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_REST_WEB_PLUGIN_HH_
#define _GAZEBO_REST_WEB_PLUGIN_HH_

#include <list>
#include <vector>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "RestApi.hh"

namespace gazebo
{
  /// \class RestWebPlugin RestWebPlugin.hh RestWebPlugin.hh
  /// \brief REST web plugin
  class GAZEBO_VISIBLE RestWebPlugin : public SystemPlugin
  {
    /// \brief Constructor
    public: RestWebPlugin();

    /// \brief Destructor
    public: virtual ~RestWebPlugin();

    /// \brief Plugin Load
    /// \param[in] _argc Argument count
    /// \param[in] _argv Argument vector
    public: virtual void Load(int _argc, char **_argv);

    /// \brief Called everytime a login message is received.
    /// \param[in] _msg The login message
    public: void OnRestLoginRequest(ConstRestLoginPtr &_msg);

    /// \brief Called everytime a logout message is received.
    /// \param[in] _msg The login message
    public: void OnRestLogoutRequest(ConstRestLogoutPtr &_msg);

    /// \brief Called everytime a REST POST event message is received
    /// \param[in] _msg The post message
    public: void OnEventRestPost(ConstRestPostPtr &_msg);

    /// \brief Called everytime a SimEvent message is received
    /// \param[in] The SimEvent message
    public: void OnSimEvent(ConstSimEventPtr &_msg);

    /// \brief Plugin initialization
    private: virtual void Init();

    /// \brief Entry point for the web requests processing thread
    private: void RunRequestQ();

    /// \brief Process a RestRequest message from the requestThread
    /// \param[in] The message to process
    private: void ProcessLoginRequest(ConstRestLoginPtr _msg);

    /// \brief Gazebo pub/sub node
    private: gazebo::transport::NodePtr node;

    /// \brief Gazebo subscriber for login requests
    private: gazebo::transport::SubscriberPtr subLogin;

    /// \brief Gazebo subscriber for logout requests
    private: gazebo::transport::SubscriberPtr subLogout;

    /// \brief Gazebo subscriber for POST events
    private: gazebo::transport::SubscriberPtr subEvent;

    /// \brief Gazebo subscriber for sim events
    private: gazebo::transport::SubscriberPtr subSimEvent;

    /// \brief Gazebo publisher
    private: gazebo::transport::PublisherPtr pub;

    /// \brief Gazebo events
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief REST calls
    private: RestApi restApi;

    /// \brief A flag to interrupt message processing
    private: bool stopMsgProcessing;

    /// \brief A list to accumulate pending requests
    private: std::list<boost::shared_ptr<const gazebo::msgs::RestLogin>>
        msgLoginQ;

    /// \brief A thread to process requests without stopping the simulation
    private: boost::thread *requestQThread;

    /// \brief A mutex to ensure integrity of the request list
    private: boost::mutex requestQMutex;

    /// \brief A session string to keep track of exercises
    private: std::string session;
  };
}

#endif


