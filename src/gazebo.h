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
#ifndef GAZEBO_HH
#define GAZEBO_HH

#include "transport/transport.h"
#include "common/common.h"
#include "math/gzmath.h"
#include "gazebo_config.h"

namespace gazebo
{
  bool load()
  {
    // Start the transport system by connecting to the master.
    return gazebo::transport::init();
  }

  bool init()
  {
    return true;
  }

  void run()
  {
    // Run transport loop. Starts a thread
    gazebo::transport::run();
  }

  void stop()
  {
    gazebo::transport::stop();
  }

  void fini()
  {
    gazebo::transport::fini();
  }
}
#endif

