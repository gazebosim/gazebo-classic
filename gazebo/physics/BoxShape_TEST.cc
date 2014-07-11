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

#include "gazebo/physics/BoxShape.hh"
#include "test/util.hh"

using namespace gazebo;

class BoxShapeTest : public gazebo::testing::AutoLogFixture { };

TEST_F(BoxShapeTest, Scale)
{
  std::ostringstream boxStr;
  boxStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='model'>"
    << "<link name ='link'>"
    <<   "<collision name ='collision'>"
    <<     "<geometry>"
    <<       "<box>"
    <<         "<size>1.0 1.0 1.0</size>"
    <<       "</box>"
    <<     "</geometry>"
    <<   "</collision>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  sdf::SDFPtr boxSDF(new sdf::SDF);
  boxSDF->SetFromString(boxStr.str());

  physics::BoxShapePtr box(new physics::BoxShape(physics::CollisionPtr()));
  sdf::ElementPtr elem = boxSDF->root;
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("model");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("link");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("collision");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("geometry");
  ASSERT_TRUE(elem != NULL);
  elem = elem->GetElement("box");
  ASSERT_TRUE(elem != NULL);
  box->Load(elem);

  // Test scaling with unit size
  ignition::math::Vector3d size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 1.0);
  EXPECT_DOUBLE_EQ(size.Y(), 1.0);
  EXPECT_DOUBLE_EQ(size.Z(), 1.0);

  box->SetScale(ignition::math::Vector3d(1.5, 1.5, 1.5));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 1.5);
  EXPECT_DOUBLE_EQ(size.Y(), 1.5);
  EXPECT_DOUBLE_EQ(size.Z(), 1.5);

  box->SetScale(ignition::math::Vector3d(2.0, 2.0, 2.0));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 2.0);
  EXPECT_DOUBLE_EQ(size.Y(), 2.0);
  EXPECT_DOUBLE_EQ(size.Z(), 2.0);

  // reset scale
  box->SetScale(ignition::math::Vector3d(1.0, 1.0, 1.0));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 1.0);
  EXPECT_DOUBLE_EQ(size.Y(), 1.0);
  EXPECT_DOUBLE_EQ(size.Z(), 1.0);

  // Test scaling with non-unit size
  box->SetSize(ignition::math::Vector3d(0.5, 0.5, 0.5));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 0.5);
  EXPECT_DOUBLE_EQ(size.Y(), 0.5);
  EXPECT_DOUBLE_EQ(size.Z(), 0.5);

  box->SetScale(ignition::math::Vector3d(2.0, 2.0, 2.0));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 1.0);
  EXPECT_DOUBLE_EQ(size.Y(), 1.0);
  EXPECT_DOUBLE_EQ(size.Z(), 1.0);

  box->SetScale(ignition::math::Vector3d(100.0, 100.0, 100.0));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 50.0);
  EXPECT_DOUBLE_EQ(size.Y(), 50.0);
  EXPECT_DOUBLE_EQ(size.Z(), 50.0);

  box->SetScale(ignition::math::Vector3d(0.1, 0.1, 0.1));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 0.05);
  EXPECT_DOUBLE_EQ(size.Y(), 0.05);
  EXPECT_DOUBLE_EQ(size.Z(), 0.05);

  // reset scale
  box->SetScale(ignition::math::Vector3d(1.0, 1.0, 1.0));
  box->SetSize(ignition::math::Vector3d(1.0, 1.0, 1.0));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 1.0);
  EXPECT_DOUBLE_EQ(size.Y(), 1.0);
  EXPECT_DOUBLE_EQ(size.Z(), 1.0);

  // Test scaling with different x, y and z components
  box->SetScale(ignition::math::Vector3d(0.5, 1.0, 2.5));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 0.5);
  EXPECT_DOUBLE_EQ(size.Y(), 1.0);
  EXPECT_DOUBLE_EQ(size.Z(), 2.5);

  // Test scaling with negative components
  // This should fail and size should remain the same as before
  box->SetScale(ignition::math::Vector3d(-1.0, -2.0, -3.0));
  size = box->GetSize();
  EXPECT_DOUBLE_EQ(size.X(), 0.5);
  EXPECT_DOUBLE_EQ(size.Y(), 1.0);
  EXPECT_DOUBLE_EQ(size.Z(), 2.5);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
