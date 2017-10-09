/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "plugins/PluginInterfaceTest.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

using namespace gazebo;

PluginInterfaceTest::PluginInterfaceTest() : ModelPlugin()
{
  // HACK Part of the hack described in PluginInterfaceTest.hh
  this->partResultListener = new GtestPartResultListener();

  ::testing::UnitTest::GetInstance()->listeners().Append(
    this->partResultListener);
}

PluginInterfaceTest::~PluginInterfaceTest()
{
}

void PluginInterfaceTest::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  this->model = _model;

  ASSERT_NE(_sdf, nullptr);
  ASSERT_NE(_model, nullptr);
  ASSERT_EQ(_model->GetName(), "box");

  TestLoadParam(_sdf);

  EXPECT_EQ(this->type, PluginType::MODEL_PLUGIN);
  EXPECT_EQ(this->GetType(), PluginType::MODEL_PLUGIN);

  EXPECT_EQ(this->filename, "libPluginInterfaceTest.so");
  EXPECT_EQ(this->GetFilename(), "libPluginInterfaceTest.so");

  EXPECT_EQ(this->handleName, "pluginInterfaceTest");
  EXPECT_EQ(this->GetHandle(), "pluginInterfaceTest");
}

void PluginInterfaceTest::TestLoadParam(sdf::ElementPtr &_sdf) const
{
  {
    bool b = false;

    LoadParam(_sdf, "bool", b, false);
    EXPECT_EQ(b, true);

    LoadParam(_sdf, "bool_nonexistent", b, false);
    EXPECT_EQ(b, false);
  }

  {
    int i = 0;

    LoadParam(_sdf, "int", i, 0);
    EXPECT_EQ(i, 42);

    LoadParam(_sdf, "int_nonexistent", i, 5);
    EXPECT_EQ(i, 5);
  }

  {
    float f = 3.14f;

    LoadParam(_sdf, "float", f, 0.0f);
    EXPECT_NEAR(f, 3.14f, 0.0f);

    LoadParam(_sdf, "float_nonexistent", f, 1.0f);
    EXPECT_NEAR(f, 1.0f, 0.0f);
  }

  {
    double d = 3.14;

    LoadParam(_sdf, "float", d, 0.0);
    EXPECT_NEAR(d, 3.14, 0.0);

    LoadParam(_sdf, "float_nonexistent", d, 1.0);
    EXPECT_NEAR(d, 1.0, 0.0);
  }

  {
    std::string s = "";

    LoadParam(_sdf, "string", s, "");
    EXPECT_EQ(s, std::string("plugin_test"));

    LoadParam(_sdf, "string_nonexistent", s, "foo");
    EXPECT_EQ(s, std::string("foo"));

    LoadParam(_sdf, "string_nonexistent2", s, std::string("bar"));
    EXPECT_EQ(s, std::string("bar"));
  }
}

void PluginInterfaceTest::Init()
{
  // HACK Part of the hack described in PluginInterfaceTest.hh
  if (!this->partResultListener->failedTests.empty())
  {
    this->model->SetName("failure");
  } else {
    this->model->SetName("success");
  }
}

// HACK Part of the hack described in PluginInterfaceTest.hh
void GtestPartResultListener::OnTestPartResult(
  const ::testing::TestPartResult &result)
{
  if (result.failed())
  {
    this->failedTests.push_back(result);
  }
}

GZ_REGISTER_MODEL_PLUGIN(PluginInterfaceTest)