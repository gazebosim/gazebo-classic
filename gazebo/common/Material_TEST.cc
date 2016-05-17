/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/Material.hh"
#include "test/util.hh"

using namespace gazebo;

class MaterialTest : public gazebo::testing::AutoLogFixture { };

TEST_F(MaterialTest, Material)
{
  common::Material mat(common::Color(1.0, 0.5, 0.2, 1.0));
  EXPECT_TRUE(mat.GetAmbient() == common::Color(1.0, 0.5, 0.2, 1.0));
  EXPECT_TRUE(mat.GetDiffuse() == common::Color(1.0, 0.5, 0.2, 1.0));
  EXPECT_STREQ("gazebo_material_0", mat.GetName().c_str());

  mat.SetTextureImage("texture_image");
  EXPECT_STREQ("texture_image", mat.GetTextureImage().c_str());

  mat.SetTextureImage("texture_image", "/path");
  EXPECT_STREQ("/path/../materials/textures/texture_image",
               mat.GetTextureImage().c_str());

  mat.SetAmbient(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetAmbient() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetDiffuse(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetDiffuse() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetSpecular(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetSpecular() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetEmissive(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetEmissive() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetTransparency(0.2);
  EXPECT_DOUBLE_EQ(0.2, mat.GetTransparency());

  mat.SetShininess(0.2);
  EXPECT_DOUBLE_EQ(0.2, mat.GetShininess());

  mat.SetBlendFactors(.1, .5);
  double a, b;
  mat.GetBlendFactors(a, b);
  EXPECT_DOUBLE_EQ(.1, a);
  EXPECT_DOUBLE_EQ(0.5, b);

  mat.SetBlendMode(common::Material::MODULATE);
  EXPECT_EQ(common::Material::MODULATE, mat.GetBlendMode());

  mat.SetShadeMode(common::Material::BLINN);
  EXPECT_EQ(common::Material::BLINN, mat.GetShadeMode());

  mat.SetPointSize(0.2);
  EXPECT_DOUBLE_EQ(0.2, mat.GetPointSize());

  mat.SetDepthWrite(false);
  EXPECT_FALSE(mat.GetDepthWrite());

  mat.SetLighting(true);
  EXPECT_TRUE(mat.GetLighting());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
