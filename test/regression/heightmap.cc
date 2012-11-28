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

#include "gazebo/rendering/Rendering.hh"
#include "gazebo/rendering/Scene.hh"
#include "ServerFixture.hh"
#include "images_cmp.h"
#include "heights_cmp.h"

using namespace gazebo;
class HeightmapTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(HeightmapTest, Heights)
{
  Load("worlds/heightmap.world");
  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model);

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzthrow("Unable to use rendering engine.");
    return;
  }

  // Make sure we can get a valid pointer to the scene.
  rendering::ScenePtr scene = rendering::get_scene("default");
  if (!scene)
  {
    gzthrow("Unable to get scene.");
    return;
  }

  // Wait for the heightmap to get loaded by the scene.
  {
    int i = 0;
    while (i < 10 && scene->GetHeightmap() == NULL)
    {
      common::Time::MSleep(100);
      i++;
    }

    if (i >= 10)
      gzthrow("Unable to get heightmap");
  }

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::shared_dynamic_cast<physics::HeightmapShape>(collision->GetShape());

  EXPECT_TRUE(shape);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  EXPECT_TRUE(shape->GetPos() == math::Vector3(0, 0, 0));
  EXPECT_TRUE(shape->GetSize() == math::Vector3(129, 129, 10));

  float diffMax, diffSum, diffAvg;
  std::vector<float> test;
  std::vector<float> renderTest;

  int x, y;

  for (y = 0; y < 129; ++y)
  {
    for (x = 0; x < 129; ++x)
    {
      test.push_back(shape->GetHeight(x * 4, y * 4));

      renderTest.push_back( scene->GetHeightmap()->GetHeight(
            x - shape->GetSize().x*0.5,
            floor(shape->GetSize().y*0.5) - y));

      if (fabs(test.back() - renderTest.back()) >= 0.2)
        std::cout << "XY[" << x << " " << y << "] P["
          << test.back() << "] R[" << renderTest.back() << "]\n";

      EXPECT_TRUE(math::equal(test.back(), renderTest.back(), 0.2f));
    }
  }

  FloatCompare(heights, &test[0], test.size(), diffMax, diffSum, diffAvg);
  printf("Max[%f] Sim[%f] Avg[%f]\n", diffMax, diffSum, diffAvg);

  //std::cout << "Height[" << scene->GetHeightmap()->GetHeight(pt.x, pt.y) << "]\n";
  // This will print the heights
  /*printf("static float __heights[] = {");
  unsigned int i=0;
  for (y = 0; y < shape->GetVertexCount().y; ++y)
  {
    for (x = 0; x < shape->GetVertexCount().x; ++x)
    {
      if (y == shape->GetVertexCount().y && x == shape->GetVertexCount().x)
        break;

      if (i % 7 == 0)
        printf("\n");
      else
        printf(" ");
      printf("%f,", shape->GetHeight(x, y));
      i++;
    }
  }
  printf(" %f};\nstatic float *heights = __heights;\n", shape->GetHeight(x,y));
  */
}
/*
/////////////////////////////////////////////////
TEST_F(HeightmapTest, WhiteAlpha)
{
  Load("worlds/white_alpha_heightmap.world");
  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::shared_dynamic_cast<physics::HeightmapShape>(collision->GetShape());

  EXPECT_TRUE(shape);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  int x, y;
  for (y = 0; y < shape->GetVertexCount().y; ++y)
  {
    for (x = 0; x < shape->GetVertexCount().x; ++x)
    {
      EXPECT_EQ(shape->GetHeight(x, y), 10.0);
    }
  }
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, WhiteNoAlpha)
{
  Load("worlds/white_no_alpha_heightmap.world");
  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::shared_dynamic_cast<physics::HeightmapShape>(collision->GetShape());

  EXPECT_TRUE(shape);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  int x, y;
  for (y = 0; y < shape->GetVertexCount().y; ++y)
  {
    for (x = 0; x < shape->GetVertexCount().x; ++x)
    {
      EXPECT_EQ(shape->GetHeight(x, y), 10.0);
    }
  }
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, NotSquareImage)
{
  common::SystemPaths::Instance()->AddGazeboPaths(
      TEST_REGRESSION_PATH);

  this->server = new Server();
  EXPECT_THROW(this->server->LoadFile("worlds/not_square_heightmap.world"),
               common::Exception);

  this->server->Fini();
  delete this->server;
}*/

/////////////////////////////////////////////////
/*TEST_F(HeightmapTest, InvalidSizeImage)
{
  common::SystemPaths::Instance()->AddGazeboPaths(
      TEST_REGRESSION_PATH);

  this->server = new Server();
  EXPECT_THROW(this->server->Load("worlds/invalid_size_heightmap.world"),
               common::Exception);

  this->server->Fini();
  delete this->server;
}*/


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
