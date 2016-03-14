/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <google/protobuf/compiler/plugin.h>
#include "GazeboGenerator.hh"

int main(int _argc, char *_argv[])
{
#ifdef _MSC_VER
  // Don't print a silly message or stick a modal dialog box in my face,
  // please.
  _set_abort_behavior(0, ~0);
#endif  // !_MSC_VER

  google::protobuf::compiler::cpp::GazeboGenerator generator("gazebo_plugin");
  return google::protobuf::compiler::PluginMain(_argc, _argv, &generator);
}


