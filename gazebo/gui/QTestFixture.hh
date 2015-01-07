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

#ifndef _QTESTFIXTURE_HH_
#define _QTESTFIXTURE_HH_

#include <string>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <boost/thread.hpp>
# include "gazebo/Server.hh"
# include "gazebo/physics/physics.hh"
# include "gazebo/rendering/rendering.hh"
#endif

#include "gazebo/gui/qt.h"
#include "gazebo/gui/qt_test.h"

#include "gazebo/gazebo_config.h"
#include "test_config.h"

/// \brief Base class for all Gazebo GUI unit tests.
class QTestFixture : public QObject
{
  Q_OBJECT

  public: QTestFixture();

  /// \brief Load a world.
  /// \param[in] _worldFilename Name of the world to load.
  /// \param[in] _paused True to start the world paused.
  /// \param[in] _serverScene True to create a scene on the server
  /// \param[in] _clientScene True to create a scene on the client
  protected: void Load(const std::string &_worldFilename, bool _paused = false,
                 bool _serverScene = true, bool _clientScene = false);

  /// \brief Pause or unpause the world.
  /// \param[in] _pause True to pause the world
  protected: void SetPause(bool _pause);

  /// \brief Get memory information about the current process.
  /// \param[out] _resident Resident size, in Kb.
  /// \param[out] _share Shared memory, in Kb.
  protected: void GetMemInfo(double &_resident, double &_share);

  /// \brief QT slot that is called automatically when the whole test case
  /// begins
  private slots: void initTestCase();

  /// \brief QT slot that is called automatically when each test begins
  private slots: void init();

  /// \brief QT slot that is called automatically when each test ends
  private slots: void cleanup();

  /// \brief QT slot that is called automatically when the whole test case ends
  private slots: void cleanupTestCase();

  /// \brief Run the Gazebo server in a thread.
  /// \param[in] _worldFilename World file to load.
  /// \param[in] _paused True to start the world paused.
  /// \param[in] _createScene True to create a scene.
  private: void RunServer(const std::string &_worldFilename, bool _paused,
                bool _createScene);

  /// \brief The Gazebo server, which is run in a thread.
  protected: gazebo::Server *server;

  /// \brief Thread to run the Gazebo server.
  protected: boost::thread *serverThread;

  /// \brief Maximum allowed percent change in resident memory usage.
  protected: double resMaxPercentChange;

  /// \brief Maximum allowed percent change in shared memory usage.
  protected: double shareMaxPercentChange;

  /// \brief Amount of resident memory at start.
  private: double residentStart;

  /// \brief Amount of shared memory at start.
  private: double shareStart;
};
#endif
