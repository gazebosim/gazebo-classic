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
#include <string.h>
#include "ServerFixture.hh"
#include "images_cmp.h"

using namespace gazebo;
class HeightmapTest : public ServerFixture
{
};

TEST_F(HeightmapTest, Heights)
{
  Load("worlds/heightmap.world");
  ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");
  EXPECT_EQ(collision->GetShapeType(), physics::Base::HEIGHTMAP_SHAPE);

  physics::HeightmapShapePtr shape = boost::shared_dynamic_cast<HeightmapShape>(
      collision->GetShape());

  EXPECT_TRUE(shape->GetOrigin() == math::Vector3(0, 0, 0));
  EXPECT_TRUE(shape->GetSize() == math::Vector3(129, 129, 10));

}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
