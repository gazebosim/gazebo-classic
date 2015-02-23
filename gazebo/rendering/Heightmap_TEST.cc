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

#include <gtest/gtest.h>
#include <boost/assign/list_of.hpp>
#include "test/ServerFixture.hh"
#include "gazebo/rendering/rendering.hh"

using namespace gazebo;
class Heightmap_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
/// \brief Test Split a terrain in a number of subterrains
TEST_F(Heightmap_TEST, splitTerrain)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  scene = gazebo::rendering::create_scene("default", false);

  // Make sure that the scene is created
  ASSERT_TRUE(scene != NULL);

  gazebo::rendering::Heightmap *heightmap =
      new gazebo::rendering::Heightmap(scene);

  // Check that the heightmap is created
  EXPECT_TRUE(heightmap != NULL);

  std::vector<float> heights;
  std::vector<std::vector<float> > heightsSplit;
  std::vector<std::vector<float> > slices;

  // Initialize a 9 x 9 terrain with known values
  int N = 9;
  heights.resize(N * N);
  for (int i = 0; i < N * N; ++i)
  {
    heights[i] = i + 1;
  }

  heightmap->SplitHeights(heights, heightmap->GetTerrainSubdivisionCount(),
      heightsSplit);

  ASSERT_TRUE(heightsSplit.size() == heightmap->GetTerrainSubdivisionCount());

  // Precomputed subterrains for a known 9 x 9 terrain starting from 1 and with
  // consecutive values
  slices.resize(16);
  slices[0] = boost::assign::list_of(1)(2)(2)(10)(11)(11)(10)(11)(11);
  slices[1] = boost::assign::list_of(3)(4)(4)(12)(13)(13)(12)(13)(13);
  slices[2] = boost::assign::list_of(5)(6)(6)(14)(15)(15)(14)(15)(15);
  slices[3] = boost::assign::list_of(7)(8)(8)(16)(17)(17)(16)(17)(17);
  slices[4] = boost::assign::list_of(19)(20)(20)(28)(29)(29)(28)(29)(29);
  slices[5] = boost::assign::list_of(21)(22)(22)(30)(31)(31)(30)(31)(31);
  slices[6] = boost::assign::list_of(23)(24)(24)(32)(33)(33)(32)(33)(33);
  slices[7] = boost::assign::list_of(25)(26)(26)(34)(35)(35)(34)(35)(35);
  slices[8] = boost::assign::list_of(37)(38)(38)(46)(47)(47)(46)(47)(47);
  slices[9] = boost::assign::list_of(39)(40)(40)(48)(49)(49)(48)(49)(49);
  slices[10] = boost::assign::list_of(41)(42)(42)(50)(51)(51)(50)(51)(51);
  slices[11] = boost::assign::list_of(43)(44)(44)(52)(53)(53)(52)(53)(53);
  slices[12] = boost::assign::list_of(55)(56)(56)(64)(65)(65)(64)(65)(65);
  slices[13] = boost::assign::list_of(57)(58)(58)(66)(67)(67)(66)(67)(67);
  slices[14] = boost::assign::list_of(59)(60)(60)(68)(69)(69)(68)(69)(69);
  slices[15] = boost::assign::list_of(61)(62)(62)(70)(71)(71)(70)(71)(71);

  // Make sure that the subterrain heights matches the precomputed slices
  for (unsigned int i = 0; i < heightmap->GetTerrainSubdivisionCount(); ++i)
  {
    EXPECT_TRUE(std::equal(heightsSplit[i].begin(), heightsSplit[i].end(),
          slices[i].begin()));
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
