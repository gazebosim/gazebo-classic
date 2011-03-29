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
#include <list>

#include "transport/ConnectionManager.hh"
#include "Transport.hh"

using namespace gazebo;

void transport::init(const std::string &master_host, unsigned short master_port)
{
  transport::ConnectionManager::Instance()->Init( master_host, master_port );
}

/// Set the global topic namespace
void transport::set_topic_namespace(const std::string &space)
{
  transport::TopicManager::Instance()->SetTopicNamespace( space );
}
