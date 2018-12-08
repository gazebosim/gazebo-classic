/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef _GAZEBO_PLUGIN_INTERFACE_TEST_HH_
#define _GAZEBO_PLUGIN_INTERFACE_TEST_HH_

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
  /// \brief Helper class to allow "bubbling up" GTest failures from the
  ///        thread running the plugin to the main test thread.
  class GtestPartResultListener : public ::testing::EmptyTestEventListener
  {
    public: std::vector<::testing::TestPartResult> failedTests;

    public: virtual void OnTestPartResult(
      const ::testing::TestPartResult &result);
  };

  class PluginInterfaceTest : public ModelPlugin
  {
    /// \brief Constructor
    public: PluginInterfaceTest();

    /// \brief Destructor
    public: virtual ~PluginInterfaceTest();

    public: virtual void Load(physics::ModelPtr _mode, sdf::ElementPtr _sdf);
    public: virtual void Init();

    private: void TestLoadParam(sdf::ElementPtr &_sdf) const;

    public: GtestPartResultListener* partResultListener;
    private: physics::ModelPtr model;
  };
}
#endif
