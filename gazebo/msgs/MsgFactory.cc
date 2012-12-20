/*
 * Copyright 2012 Open Source Robotics Foundation
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

std::map<std::string, MsgFactoryFn> MsgFactory::msgMap;

/////////////////////////////////////////////////
void MsgFactory::RegisterAll()
{
}

/////////////////////////////////////////////////
void MsgFactory::RegisterMsg(const std::string &_msgType,
                             MsgFactoryFn _factoryfn)
{
  msgMap[_msgType] = _factoryfn;
}

/////////////////////////////////////////////////
google::protobuf::Message *MsgFactory::NewMsg(const std::string &_msgType)
{
  google::protobuf::Message *msg = NULL;

  if (msgMap[_msgType])
    msg = (msgMap[_msgType]) ();

  return msg;
}

/////////////////////////////////////////////////
void MsgFactory::GetMsgTypes(std::vector<std::string> &_types)
{
  _types.clear();

  std::map<std::string, MsgFactoryFn>::const_iterator iter;
  for (iter = msgMap.begin(); iter != msgMap.end(); ++iter)
  {
    _types.push_back(iter->first);
  }
}
