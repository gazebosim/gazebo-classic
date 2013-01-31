/*
 * Copyright 2011 Nate Koenig
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

#include <boost/thread.hpp>
#include "gazebo_config.h"
#include "test_config.h"
#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/Server.hh"

#include "gazebo/gui/qt.h"

class TestFramework : public QObject
{
  Q_OBJECT

  private slots: void initTestCase();
  private slots: void cleanupTestCase();

  private: void RunServer();

  protected: gazebo::Server *server;
  protected: boost::thread *serverThread;
};
