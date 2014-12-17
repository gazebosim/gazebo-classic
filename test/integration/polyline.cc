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
#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "helper_physics_generator.hh"

class PolylineTest : public ServerFixture,
                     public testing::WithParamInterface<const char*>
{
  public: void PolylineWorld(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
// Test polyline instantiation and polyline collision
void PolylineTest::PolylineWorld(const std::string &_physicsEngine)
{
  // Load the sample world
  Load("worlds/polyline.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr triangleModel = world->GetModel("triangle");
  EXPECT_TRUE(triangleModel != NULL);

  physics::LinkPtr triangleLink = triangleModel->GetLink("link");
  EXPECT_TRUE(triangleLink != NULL);

  physics::CollisionPtr triangleColl = triangleLink->GetCollision("collision");
  EXPECT_TRUE(triangleColl != NULL);

  physics::ShapePtr shape = triangleColl->GetShape();
  EXPECT_TRUE(shape != NULL);
  EXPECT_TRUE(shape->HasType(physics::Base::POLYLINE_SHAPE));

  physics::PolylineShapePtr polyShape =
    boost::dynamic_pointer_cast<physics::PolylineShape>(shape);
  EXPECT_TRUE(polyShape != NULL);

  EXPECT_DOUBLE_EQ(polyShape->GetHeight(), 1.0);

  std::vector<math::Vector2d> vertices = polyShape->GetVertices();
  EXPECT_EQ(vertices[0], math::Vector2d(-0.5, -0.5));
  EXPECT_EQ(vertices[1], math::Vector2d(-0.5, 0.5));
  EXPECT_EQ(vertices[2], math::Vector2d(0.5, 0.5));
  EXPECT_EQ(vertices[3], math::Vector2d(0.0, 0.0));
  EXPECT_EQ(vertices[4], math::Vector2d(0.5, -0.5));

  // Check the FillMsg function
  {
    msgs::Geometry msg;
    polyShape->FillMsg(msg);
    EXPECT_EQ(msg.type(), msgs::Geometry::POLYLINE);
    EXPECT_DOUBLE_EQ(msg.polyline().height(), 1);
    EXPECT_DOUBLE_EQ(msg.polyline().point(0).x(), -0.5);
    EXPECT_DOUBLE_EQ(msg.polyline().point(0).y(), -0.5);

    EXPECT_DOUBLE_EQ(msg.polyline().point(1).x(), -0.5);
    EXPECT_DOUBLE_EQ(msg.polyline().point(1).y(), 0.5);

    EXPECT_DOUBLE_EQ(msg.polyline().point(2).x(), 0.5);
    EXPECT_DOUBLE_EQ(msg.polyline().point(2).y(), 0.5);

    EXPECT_DOUBLE_EQ(msg.polyline().point(3).x(), 0.0);
    EXPECT_DOUBLE_EQ(msg.polyline().point(3).y(), 0.0);

    EXPECT_DOUBLE_EQ(msg.polyline().point(4).x(), 0.5);
    EXPECT_DOUBLE_EQ(msg.polyline().point(4).y(), -0.5);
  }

  // Spawn a sphere over the polyline shape, and check that it doesn't pass
  // through the polyline
  {
    SpawnSphere("test_sphere", math::Vector3(0, 0, 1.5),
        math::Vector3(0, 0, 0));
    physics::ModelPtr sphere = GetModel("test_sphere");

    common::Time::MSleep(1000);
    EXPECT_NEAR(sphere->GetWorldPose().pos.z, 1.5, 1e-2);
  }
}

/////////////////////////////////////////////////
TEST_P(PolylineTest, PolylineWorld)
{
  if (GetParam() == std::string("simbody"))
    gzwarn << "Polyline not supported in simbody\n";
  else
    PolylineWorld(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PolylineTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
