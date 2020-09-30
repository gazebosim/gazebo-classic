/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <memory>
#include "gazebo/common/Exception.hh"
#include "gazebo/util/LogRecord.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/Server.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
#ifndef _WIN32
  ::setenv("RMT_PORT", "1500", true);
#endif
  std::unique_ptr<gazebo::Server> server;

  try
  {
    // Initialize the informational logger. This will log warnings, and
    // errors.
    gzLogInit("server-", "gzserver.log");

    // Initialize the data logger. This will log state information.
    gazebo::util::LogRecord::Instance()->Init("gzserver");

    server.reset(new gazebo::Server());
    if (!server->ParseArgs(argc, argv))
      return -1;

    server->Run();
    server->Fini();
  }
  catch(gazebo::common::Exception &_e)
  {
    _e.Print();

    server->Fini();
    return -1;
  }
  catch(Ogre::Exception &_e)
  {
    gzerr << "Ogre Error:" << _e.getFullDescription() << "\n";

    server->Fini();
    return -1;
  }

  return 0;
}
