/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "test/util.hh"
#include "gazebo/physics/SphereShape.hh"

using namespace gazebo;

class SphereShapeTest : public gazebo::testing::AutoLogFixture { };

TEST_F(SphereShapeTest, Scale)
{
  std::ostringstream sphereStr;
  sphereStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<collision name ='collision'>"
    <<     "<geometry>"
    <<       "<sphere>"
    <<         "<radius>0.5</radius>"
    <<       "</sphere>"
    <<     "</geometry>"
    <<   "</collision>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sphereSDF(new sdf::SDF);
  sphereSDF->SetFromString(sphereStr.str());

  physics::SphereShapePtr sphere(
      new physics::SphereShape(physics::CollisionPtr()));
  sdf::ElementPtr elem = sphereSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("collision");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("geometry");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("sphere");
  ASSERT_TRUE(elem != NULL);
  sphere->Load(elem);

  // Test scaling with unit size
  double radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 0.5);

  sphere->SetScale(math::Vector3(1.5, 1.5, 1.5));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 0.75);

  sphere->SetScale(math::Vector3(2.0, 2.0, 2.0));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 1.0);

  // reset scale
  sphere->SetScale(math::Vector3(1.0, 1.0, 1.0));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 0.5);

  // Test scaling with non-unit size
  sphere->SetRadius(2.5);
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 2.5);

  sphere->SetScale(math::Vector3(2.0, 2.0, 2.0));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 5.0);

  sphere->SetScale(math::Vector3(100.0, 100.0, 100.0));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 250.0);

  sphere->SetScale(math::Vector3(0.1, 0.1, 0.1));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 0.25);

  // reset scale
  sphere->SetScale(math::Vector3(1.0, 1.0, 1.0));
  sphere->SetRadius(0.5);
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 0.5);

  // Test scaling with different x, y and z components
  sphere->SetScale(math::Vector3(0.5, 1.0, 2.5));
  radius = sphere->GetRadius();
  // radius should be multiplied by max of (0.5, 1.0, 2.5)
  EXPECT_DOUBLE_EQ(radius, 1.25);

  // Test scaling with negative components
  // This should fail and radius should remain the same as before
  sphere->SetScale(math::Vector3(-1.0, -2.0, -3.0));
  radius = sphere->GetRadius();
  EXPECT_DOUBLE_EQ(radius, 1.25);
}

TEST_F(SphereShapeTest, Volume)
{
  std::ostringstream sphereStr;
  sphereStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<collision name ='collision'>"
    <<     "<geometry>"
    <<       "<sphere>"
    <<         "<radius>1.0</radius>"
    <<       "</sphere>"
    <<     "</geometry>"
    <<   "</collision>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr sphereSDF(new sdf::SDF);
  sphereSDF->SetFromString(sphereStr.str());

  physics::SphereShapePtr sphere(
      new physics::SphereShape(physics::CollisionPtr()));
  sdf::ElementPtr elem = sphereSDF->Root();
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("collision");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("geometry");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("sphere");
  ASSERT_TRUE(elem != NULL);
  sphere->Load(elem);

  EXPECT_DOUBLE_EQ(sphere->ComputeVolume(), 4*M_PI/3);

  sphere->SetRadius(2);

  EXPECT_DOUBLE_EQ(sphere->ComputeVolume(), 4*M_PI*8/3);

  // The bounding box approximation should be 0 because the Shape has no
  // Collision parent
  EXPECT_DOUBLE_EQ(sphere->Shape::ComputeVolume(), 0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
