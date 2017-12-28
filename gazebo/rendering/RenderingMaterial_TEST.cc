/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include <ignition/math/Color.hh>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Material.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/test/ServerFixture.hh"

#include "test_config.h"

using namespace gazebo;
class Material_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(Material_TEST, Update)
{
  {
    // create an opaque common::Material
    ignition::math::Color ambient(0.1, 0.2, 0.3, 1.0);
    ignition::math::Color diffuse(0.4, 0.3, 0.2, 1.0);
    ignition::math::Color specular(0.2, 0.8, 0.0, 1.0);
    ignition::math::Color emissive(0.7, 0.5, 0.3, 1.0);
    float transparency = 0.0;
    float shininess = 0.5;
    bool depthWrite = true;
    bool lighting = true;

    common::Material mat;
    std::string path = TEST_PATH;
    path += "/data/cordless_drill/materials/textures";
    std::string texName = "cordless_drill.png";
    mat.SetTextureImage(texName, path);
    mat.SetAmbient(ambient);
    mat.SetDiffuse(diffuse);
    mat.SetSpecular(specular);
    mat.SetEmissive(emissive);
    mat.SetTransparency(transparency);
    mat.SetShininess(shininess);
    mat.SetDepthWrite(depthWrite);
    mat.SetLighting(lighting);

    // invoke rendering material update to create the ogre material
    rendering::Material::Update(&mat);

    // verify ogre pass properties
    EXPECT_TRUE(
        Ogre::MaterialManager::getSingleton().resourceExists(mat.GetName()));
    Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().getByName(
        mat.GetName(), "General");
    EXPECT_FALSE(matPtr.isNull());
    EXPECT_EQ(matPtr->getNumTechniques(), 1);
    Ogre::Technique *technique = matPtr->getTechnique(0);
    EXPECT_TRUE(technique != nullptr);
    EXPECT_EQ(technique->getNumPasses(), 1);
    Ogre::Pass *pass = technique->getPass(0);
    EXPECT_TRUE(pass != nullptr);
    EXPECT_EQ(ambient,
        rendering::Conversions::Convert(pass->getAmbient()));
    EXPECT_EQ(diffuse,
        rendering::Conversions::Convert(pass->getDiffuse()));
    EXPECT_EQ(specular,
        rendering::Conversions::Convert(pass->getSpecular()));
    EXPECT_FLOAT_EQ(pass->getShininess(), shininess);
    EXPECT_EQ(pass->getDepthWriteEnabled(), depthWrite);
    EXPECT_EQ(pass->getLightingEnabled(), lighting);
    EXPECT_EQ(pass->getNumTextureUnitStates(), 1);
    Ogre::TextureUnitState *texState = pass->getTextureUnitState(0);
    EXPECT_TRUE(texState != nullptr);
    std::string pathToTex = path + "/" + texName;
    EXPECT_STREQ(texState->getTextureName().c_str(), pathToTex.c_str());
  }

  {
    // create a semi-transparent common::Material
    ignition::math::Color ambient(0.1, 0.2, 0.3, 1.0);
    ignition::math::Color diffuse(0.4, 0.3, 0.2, 0.3);
    ignition::math::Color specular(0.2, 0.8, 0.0, 1.0);
    ignition::math::Color emissive(0.7, 0.5, 0.3, 1.0);
    float transparency = 0.5;
    float shininess = 0.2;
    bool depthWrite = false;
    bool lighting = true;

    common::Material mat;
    mat.SetAmbient(ambient);
    mat.SetDiffuse(diffuse);
    mat.SetSpecular(specular);
    mat.SetEmissive(emissive);
    mat.SetTransparency(transparency);
    mat.SetShininess(shininess);
    mat.SetDepthWrite(depthWrite);
    mat.SetLighting(lighting);

    // invoke rendering material update to create the ogre material
    rendering::Material::Update(&mat);

    // verify ogre pass properties
    EXPECT_TRUE(
        Ogre::MaterialManager::getSingleton().resourceExists(mat.GetName()));
    Ogre::MaterialPtr matPtr = Ogre::MaterialManager::getSingleton().getByName(
        mat.GetName(), "General");
    EXPECT_FALSE(matPtr.isNull());
    EXPECT_EQ(matPtr->getNumTechniques(), 1);
    Ogre::Technique *technique = matPtr->getTechnique(0);
    EXPECT_TRUE(technique != nullptr);
    EXPECT_EQ(technique->getNumPasses(), 1);
    Ogre::Pass *pass = technique->getPass(0);
    EXPECT_TRUE(pass != nullptr);
    EXPECT_EQ(ambient,
        rendering::Conversions::Convert(pass->getAmbient()));
    // diffuse alpha value should be overwritten based on material transparency
    ignition::math::Color newDiffuse = diffuse;
    newDiffuse.A(1.0f-transparency);
    EXPECT_EQ(newDiffuse,
        rendering::Conversions::Convert(pass->getDiffuse()));
    EXPECT_EQ(rendering::Conversions::Convert(
          pass->getSpecular()), specular);
    EXPECT_FLOAT_EQ(pass->getShininess(), shininess);
    EXPECT_EQ(pass->getDepthWriteEnabled(), depthWrite);
    EXPECT_EQ(pass->getLightingEnabled(), lighting);
    EXPECT_EQ(pass->getNumTextureUnitStates(), 0);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
