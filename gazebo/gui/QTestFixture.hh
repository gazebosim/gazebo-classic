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

#ifndef _QTESTFIXTURE_HH_
#define _QTESTFIXTURE_HH_

#include <boost/thread.hpp>

#include "gazebo/physics/physics.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/Server.hh"
#include "gazebo/gui/qt.h"

#include "gazebo_config.h"
#include "test_config.h"

/// \brief Base class for all Gazebo GUI unit tests.
class QTestFixture : public QObject
{
  Q_OBJECT

  /// \brief QT slot that is called automatically when a test begins
  private slots: void initTestCase();

  /// \brief QT slot that is called automatically when a test ends
  private slots: void cleanupTestCase();

  /// \brief Run the Gazebo server in a thread.
  private: void RunServer();

  /// \brief The Gazebo server, which is run in a thread.
  protected: gazebo::Server *server;

  /// \brief Thread to run the Gazebo server.
  protected: boost::thread *serverThread;
};

#endif
