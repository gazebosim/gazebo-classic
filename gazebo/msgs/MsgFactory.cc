/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/msgs/MsgFactory.hh"

using namespace gazebo;
using namespace msgs;

std::map<std::string, MsgFactoryFn> *MsgFactory::msgMap = NULL;

/////////////////////////////////////////////////
void MsgFactory::RegisterMsg(const std::string &_msgType,
                             MsgFactoryFn _factoryfn)
{
  // Create the msgMap if it's null
  if (!msgMap)
    msgMap = new std::map<std::string, MsgFactoryFn>;

  (*msgMap)[_msgType] = _factoryfn;
}

/////////////////////////////////////////////////
boost::shared_ptr<google::protobuf::Message> MsgFactory::NewMsg(
    const std::string &_msgType)
{
  boost::shared_ptr<google::protobuf::Message> msg;

  // Create a new message if a MsgFactoryFn has been assigned to the message
  // type

  if (msgMap->find(_msgType) != msgMap->end())
    msg = ((*msgMap)[_msgType]) ();

  return msg;
}

/////////////////////////////////////////////////
void MsgFactory::GetMsgTypes(std::vector<std::string> &_types)
{
  _types.clear();

  // Return the list of all known message types.
  std::map<std::string, MsgFactoryFn>::const_iterator iter;
  for (iter = msgMap->begin(); iter != msgMap->end(); ++iter)
  {
    _types.push_back(iter->first);
  }
}
