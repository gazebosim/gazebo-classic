/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _REST_SERVICE_PLUGIN_HH_
#define _REST_SERVICE_PLUGIN_HH_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "RestApi.hh"


namespace gazebo
{

  class GAZEBO_VISIBLE SimpleMOOCPlugin : public SystemPlugin
  {
    /// \brief ctor
    public: SimpleMOOCPlugin();

    /// \brief dtor
    public: virtual ~SimpleMOOCPlugin();

    /// \brief Plugin Load
    public: virtual void Load(int /*_argc*/, char ** /*_argv*/);

    /// \brief  called everytime a login message is received.
    public: void OnRestLoginRequest(ConstRestLoginPtr &_msg);
   
    /// \brief called everytime a REST POST event message is received
    public: void OnEventRestPost(ConstRestPostPtr &_msg);
 
    /// \brief Plugin initialization
    private: virtual void Init();
    
    /// \brief Entry point for the web requests processing thread
    private: void RunRequestQ();

    /// \brief Process a RestRequest message from the requestThread
    private: void ProcessLoginRequest(ConstRestLoginPtr _msg);

    /// \brief Process a MOOCEvent messsage from the requestThread
    private: void ProcessMOOCEvent(ConstRestPostPtr _msg);

    /// \brief Gazebo pub/sub node
    private: gazebo::transport::NodePtr node;
    
    /// \brief Gazebo subscriber for login requests
    private: gazebo::transport::SubscriberPtr subRequest;
    
    /// \brief Gazebo subscriber for MOOC events (ex: scoring)
    private: gazebo::transport::SubscriberPtr subEvent;

    /// \brief Gazebo publisher
    private: gazebo::transport::PublisherPtr pub;

    /// \brief Gazebo events
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief REST calls
    private: RestApi restApi;

    /// \brief a flag to interrupt message processing
    private: bool stopMsgProcessing;
 
    /// \brief a list to accumulate pending request    
    private: std::list< boost::shared_ptr<const gazebo::msgs::RestLogin> > msgLoginQ;

    /// \brief a list to accumulate pending request    
    private: std::list< boost::shared_ptr<const gazebo::msgs::RestPost> > msgEventQ;
 
    /// \brief a thread to process requests without stopping the simulation
    private: boost::thread *requestQThread;

    /// \brief a mutex to ensure integrity of the request list
    private: boost::mutex requestQMutex;

    /// \brief a session string to keep track of exercises
    private: std::string session;
 };

}

#endif


