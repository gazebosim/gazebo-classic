/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "test_config.h"

#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/rendering.hh"

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class VisualShininess : public RenderingFixture
{
};

/////////////////////////////////////////////////
void CheckShininessService(const std::string &_scopedName,
    double _expectedShininess)
{
  std::string serviceName = "/shininess";
  ignition::transport::Node ignNode;

  {
    std::vector<ignition::transport::ServicePublisher> publishers;
    EXPECT_TRUE(ignNode.ServiceInfo(serviceName, publishers));
  }

  ignition::msgs::StringMsg request;
  gazebo::msgs::Any reply;
  const unsigned int timeout = 3000;
  bool result = false;
  request.set_data(_scopedName);
  EXPECT_TRUE(ignNode.Request(serviceName, request, timeout, reply, result));
  EXPECT_TRUE(result);
  EXPECT_EQ(msgs::Any_ValueType_DOUBLE, reply.type());
  EXPECT_DOUBLE_EQ(_expectedShininess, reply.double_value()) << _scopedName;
}

/////////////////////////////////////////////////
TEST_F(VisualShininess, ShapesShininessServices)
{
  Load("worlds/shapes_shininess.world", true);

  CheckShininessService("ground_plane::link::visual", 0.0);
  CheckShininessService("box::link::visual", 1.0);
  CheckShininessService("sphere::link::visual", 5.0);
  CheckShininessService("cylinder::link::visual", 10.0);
}

/////////////////////////////////////////////////
TEST_F(VisualShininess, ShapesShininess)
{
  Load("worlds/shapes_shininess.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run shininess test"
          << std::endl;
    return;
  }
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(nullptr, scene);

  // Wait until all models are inserted
  unsigned int sleep = 0;
  unsigned int maxSleep = 30;
  rendering::VisualPtr box, sphere, cylinder;
  while ((!box || !sphere || !cylinder) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");

    common::Time::MSleep(100);
    sleep++;
  }
  ASSERT_NE(nullptr, box);
  ASSERT_NE(nullptr, cylinder);
  ASSERT_NE(nullptr, sphere);
  EXPECT_TRUE(scene->Initialized());

  std::unordered_map<std::string, double> nameToShininess;
  nameToShininess["box::link::visual"] = 1.0;
  nameToShininess["sphere::link::visual"] = 5.0;
  nameToShininess["cylinder::link::visual"] = 10.0;

  for (const auto &nameShininess : nameToShininess)
  {
    rendering::VisualPtr visual = scene->GetVisual(nameShininess.first);
    ASSERT_NE(nullptr, visual);

    // check shininess value
    EXPECT_DOUBLE_EQ(nameShininess.second, visual->Shininess())
        << nameShininess.first;
  }
}
