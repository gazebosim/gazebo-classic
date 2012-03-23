/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef SERVER_HH
#define SERVER_HH

#include <string>
#include <vector>
#include <list>
#include <map>

#include <boost/thread.hpp>

#include "transport/TransportTypes.hh"
#include "common/CommonTypes.hh"
#include "physics/PhysicsTypes.hh"
#include "physics/World.hh"

namespace boost
{
  class mutex;
}

namespace gazebo
{
  class Master;

  class Server
  {
    public: Server();
    public: virtual ~Server();

    public: void LoadPlugin(const std::string &_filename);
    public: bool Load(const std::string &filename);
    public: void Init();
    public: void Run();
    public: void Stop();
    public: void Fini();

    public: void SetParams(const common::StrStr_M &params);

    public: bool GetInitialized() const;

    private: void OnControl(ConstServerControlPtr &_msg);

    private: bool OpenWorld(const std::string &_filename);

    private: void ProcessControlMsgs();

    private: bool stop;

    private: Master *master;
    private: boost::thread *masterThread;
    private: std::vector<gazebo::SystemPluginPtr> plugins;
    private: transport::NodePtr node;
    private: transport::SubscriberPtr serverSub;
    private: transport::PublisherPtr worldModPub;

    private: boost::mutex *receiveMutex;
    private: std::list<msgs::ServerControl> controlMsgs;
    private: std::map<std::string, std::string> worldFilenames;
  };
}

#endif


