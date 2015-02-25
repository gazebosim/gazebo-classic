/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <signal.h>
#include <boost/lexical_cast.hpp>

#include "gazebo/common/Exception.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/Master.hh"
#include "gazebo/gazebo_config.h"

gazebo::Master *master = NULL;

//////////////////////////////////////////////////
void PrintVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

//////////////////////////////////////////////////
void SignalHandler(int /*dummy*/)
{
  master->Stop();
  return;
}

int main(int /*argc*/, char ** /*argv*/)
{
  try
  {
    PrintVersion();

    if (signal(SIGINT, SignalHandler) == SIG_ERR)
    {
      std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
      return -1;
    }

    std::string host = "";
    unsigned int port = 0;

    if (!gazebo::transport::get_master_uri(host, port))

      master = new gazebo::Master();
    master->Init(port);
    master->Run();
    master->Fini();

    delete master;
    master = NULL;
  }
  catch(gazebo::common::Exception &_e)
  {
    _e.Print();
  }

  return 1;
}

