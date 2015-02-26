/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include "gazebo/math/Rand.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "test/ServerFixture.hh"


using namespace gazebo;
class Visual_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
std::string GetVisualSDFString(const std::string &_name,
    const std::string &_geomType = "box",
    gazebo::math::Pose _pose = gazebo::math::Pose::Zero,
    double _transparency = 0, bool _castShadows = true,
    const std::string &_material = "Gazebo/Grey",
    bool _lighting = true,
    const common::Color &_ambient = common::Color::White,
    const common::Color &_diffuse = common::Color::White,
    const common::Color &_specular = common::Color::Black,
    const common::Color &_emissive = common::Color::Black)
{
  std::stringstream visualString;
  visualString
      << "<sdf version='1.5'>"
      << "  <visual name='" << _name << "'>"
      << "    <pose>" << _pose << "</pose>"
      << "    <geometry>";
  if (_geomType == "box")
  {
    visualString
      << "      <box>"
      << "        <size>1.0 1.0 1.0</size>"
      << "      </box>";
  }
  else if (_geomType == "sphere")
  {
    visualString
      << "      <sphere>"
      << "        <radius>0.5</radius>"
      << "      </sphere>";
  }
  else if (_geomType == "cylinder")
  {
    visualString
      << "      <cylinder>"
      << "        <radius>0.5</radius>"
      << "        <length>1.0</length>"
      << "      </cylinder>";
  }
  visualString
      << "    </geometry>"
      << "    <material>"
      << "      <script>"
      << "        <uri>file://media/materials/scripts/gazebo.material</uri>"
      << "        <name>" << _material << "</name>"
      << "      </script>"
      << "      <lighting>" << _lighting << "</lighting>"
      << "      <ambient>" << _ambient << "</ambient>"
      << "      <diffuse>" << _diffuse << "</diffuse>"
      << "      <specular>" << _specular << "</specular>"
      << "      <emissive>" << _emissive << "</emissive>"
      << "    </material>"
      << "    <transparency>" << _transparency << "</transparency>"
      << "    <cast_shadows>" << _castShadows << "</cast_shadows>"
      << "  </visual>"
      << "</sdf>";
  return visualString.str();
}

/////////////////////////////////////////////////
void CreateColorMaterial(const std::string &_materialName,
    const common::Color &_ambient, const common::Color &_diffuse,
    const common::Color &_specular, const common::Color &_emissive)
{
  // test setup - create a material for testing
  Ogre::MaterialPtr ogreMaterial =
      Ogre::MaterialManager::getSingleton().create(_materialName, "General");
  for (unsigned int i = 0; i < ogreMaterial->getNumTechniques(); ++i)
  {
    Ogre::Technique *technique = ogreMaterial->getTechnique(i);
    for (unsigned int j = 0; j < technique->getNumPasses(); ++j)
    {
      Ogre::Pass *pass = technique->getPass(j);
      pass->setAmbient(rendering::Conversions::Convert(_ambient));
      pass->setDiffuse(rendering::Conversions::Convert(_diffuse));
      pass->setSpecular(rendering::Conversions::Convert(_specular));
      pass->setSelfIllumination(rendering::Conversions::Convert(_emissive));
      EXPECT_EQ(rendering::Conversions::Convert(pass->getAmbient()), _ambient);
      EXPECT_EQ(rendering::Conversions::Convert(pass->getDiffuse()), _diffuse);
      EXPECT_EQ(rendering::Conversions::Convert(pass->getSpecular()),
          _specular);
      EXPECT_EQ(rendering::Conversions::Convert(pass->getSelfIllumination()),
          _emissive);
    }
  }
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, BoundingBox)
{
  Load("worlds/empty.world");

  // Spawn a box and check its bounding box dimensions.
  SpawnBox("box", math::Vector3(1, 1, 1), math::Vector3(10, 10, 1),
      math::Vector3(0, 0, 0));

  // FIXME need a camera otherwise test produces a gl vertex buffer error
  math::Pose cameraStartPose(0, 0, 0, 0, 0, 0);
  std::string cameraName = "test_camera";
  SpawnCamera("test_camera_model", cameraName,
      cameraStartPose.pos, cameraStartPose.rot.GetAsEuler());

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  int sleep = 0;
  int maxSleep = 50;
  rendering::VisualPtr visual;
  while (!visual && sleep < maxSleep)
  {
    visual = scene->GetVisual("box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_TRUE(visual != NULL);

  // verify initial bounding box
  math::Vector3 bboxMin(-0.5, -0.5, -0.5);
  math::Vector3 bboxMax(0.5, 0.5, 0.5);
  math::Box boundingBox = visual->GetBoundingBox();
  EXPECT_EQ(boundingBox.min, bboxMin);
  EXPECT_EQ(boundingBox.max, bboxMax);

  // verify scale
  math::Vector3 scale = visual->GetScale();
  EXPECT_EQ(scale, math::Vector3(1, 1, 1));

  // set new scale
  math::Vector3 scaleToSet(2, 3, 4);
  visual->SetScale(scaleToSet);

  // verify new scale
  math::Vector3 newScale = visual->GetScale();
  EXPECT_EQ(newScale, scaleToSet);
  EXPECT_EQ(newScale, math::Vector3(2, 3, 4));

  // verify local bounding box dimensions remain the same
  math::Box newBoundingBox = visual->GetBoundingBox();
  EXPECT_EQ(newBoundingBox.min, bboxMin);
  EXPECT_EQ(newBoundingBox.max, bboxMax);

  // verify local bounding box dimensions with scale applied
  EXPECT_EQ(newScale*newBoundingBox.min, newScale*bboxMin);
  EXPECT_EQ(newScale*newBoundingBox.max, newScale*bboxMax);
  EXPECT_EQ(newScale*newBoundingBox.min, math::Vector3(-1, -1.5, -2));
  EXPECT_EQ(newScale*newBoundingBox.max, math::Vector3(1, 1.5, 2));
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, GetGeometryType)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  // box geom
  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("visual_box", "box"), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", scene));
  boxVis->Load(boxSDF);
  EXPECT_EQ(boxVis->GetGeometryType(), "box");

  // sphere geom
  sdf::ElementPtr sphereSDF(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF);
  sdf::readString(GetVisualSDFString("visual_sphere", "sphere"), sphereSDF);
  gazebo::rendering::VisualPtr sphereVis(
      new gazebo::rendering::Visual("sphere_visual", scene));
  sphereVis->Load(sphereSDF);
  EXPECT_EQ(sphereVis->GetGeometryType(), "sphere");

  // cylinder geom
  sdf::ElementPtr cylinderSDF(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF);
  sdf::readString(GetVisualSDFString("visual_cylinder", "cylinder"),
      cylinderSDF);
  gazebo::rendering::VisualPtr cylinderVis(
      new gazebo::rendering::Visual("cylinder_visual", scene));
  cylinderVis->Load(cylinderSDF);
  EXPECT_EQ(cylinderVis->GetGeometryType(), "cylinder");
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, CastShadows)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  // load a model that casts shadows by default
  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("visual_box_has_shadows", "box",
      gazebo::math::Pose::Zero, 0, true), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", scene));
  boxVis->Load(boxSDF);
  EXPECT_TRUE(boxVis->GetCastShadows());

  // load another model that does not cast shadows by default
  sdf::ElementPtr boxSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF2);
  sdf::readString(GetVisualSDFString("visual_box_has_no_shadows", "box",
      gazebo::math::Pose::Zero, 0, false), boxSDF2);
  gazebo::rendering::VisualPtr boxVis2(
      new gazebo::rendering::Visual("box_visual2", scene));
  boxVis2->Load(boxSDF2);
  EXPECT_FALSE(boxVis2->GetCastShadows());

  // test changing cast shadow property
  boxVis->SetCastShadows(false);
  EXPECT_FALSE(boxVis->GetCastShadows());
  boxVis2->SetCastShadows(true);
  EXPECT_TRUE(boxVis2->GetCastShadows());
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Transparency)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  sdf::ElementPtr sphereSDF(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF);
  sdf::readString(GetVisualSDFString("visual_sphere_no_transparency", "sphere",
      gazebo::math::Pose::Zero, 0), sphereSDF);
  gazebo::rendering::VisualPtr sphereVis(
      new gazebo::rendering::Visual("sphere_visual", scene));
  sphereVis->Load(sphereSDF);
  EXPECT_DOUBLE_EQ(sphereVis->GetTransparency(), 0);

  sdf::ElementPtr sphereSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF2);
  sdf::readString(GetVisualSDFString("visual_sphere_semi_transparent", "sphere",
      gazebo::math::Pose::Zero, 0.5), sphereSDF2);
  gazebo::rendering::VisualPtr sphereVis2(
      new gazebo::rendering::Visual("sphere_visual2", scene));
  sphereVis2->Load(sphereSDF2);
  EXPECT_DOUBLE_EQ(sphereVis2->GetTransparency(), 0.5f);

  // test changing transparency property
  sphereVis->SetTransparency(0.3f);
  EXPECT_DOUBLE_EQ(sphereVis->GetTransparency(), 0.3f);
  sphereVis2->SetTransparency(1.0f);
  EXPECT_DOUBLE_EQ(sphereVis2->GetTransparency(), 1.0f);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Material)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("visual_box_red", "box",
      gazebo::math::Pose::Zero, 0, true, "Gazebo/Red"), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", scene));
  boxVis->Load(boxSDF);
  // visual generates a new material with unique name so the name is slightly
  // longer with a prefix of the visual name
  EXPECT_TRUE(
      boxVis->GetMaterialName().find("Gazebo/Red") != std::string::npos);

  sdf::ElementPtr boxSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF2);
  sdf::readString(GetVisualSDFString("visual_box_white", "box",
      gazebo::math::Pose::Zero, 0, true, "Gazebo/White", true), boxSDF2);
  gazebo::rendering::VisualPtr boxVis2(
      new gazebo::rendering::Visual("box_visual2", scene));
  boxVis2->Load(boxSDF2);
  EXPECT_TRUE(
      boxVis2->GetMaterialName().find("Gazebo/White") != std::string::npos);

  // test changing to use non-unique materials
  boxVis->SetMaterial("Gazebo/Yellow", false);
  EXPECT_EQ(boxVis->GetMaterialName(), "Gazebo/Yellow");
  boxVis2->SetMaterial("Gazebo/OrangeTransparent", false);
  EXPECT_EQ(boxVis2->GetMaterialName(), "Gazebo/OrangeTransparent");
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Lighting)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  sdf::ElementPtr cylinderSDF(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF);
  sdf::readString(GetVisualSDFString("visual_cylinder_lighting", "cylinder",
      gazebo::math::Pose::Zero, 0, true, "Gazebo/Grey", true), cylinderSDF);
  gazebo::rendering::VisualPtr cylinderVis(
      new gazebo::rendering::Visual("cylinder_visual", scene));
  cylinderVis->Load(cylinderSDF);
  EXPECT_TRUE(cylinderVis->GetLighting());

  sdf::ElementPtr cylinderSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF2);
  sdf::readString(GetVisualSDFString("visual_cylinder_no_lighting", "cylinder",
      gazebo::math::Pose::Zero, 0, true, "Gazebo/Grey", false), cylinderSDF2);
  gazebo::rendering::VisualPtr cylinderVis2(
      new gazebo::rendering::Visual("cylinder_visual2", scene));
  cylinderVis2->Load(cylinderSDF2);
  EXPECT_FALSE(cylinderVis2->GetLighting());

  // test changing lighting property
  cylinderVis->SetLighting(false);
  EXPECT_FALSE(cylinderVis->GetLighting());
  cylinderVis2->SetLighting(true);
  EXPECT_TRUE(cylinderVis2->GetLighting());
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Color)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  sdf::ElementPtr cylinderSDF(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF);
  sdf::readString(GetVisualSDFString("visual_cylinder_black", "cylinder",
      gazebo::math::Pose::Zero, 0, true, "Gazebo/Grey", true,
      common::Color::Black, common::Color::Black, common::Color::Black,
      common::Color::Black), cylinderSDF);
  gazebo::rendering::VisualPtr cylinderVis(
      new gazebo::rendering::Visual("cylinder_visual", scene));
  cylinderVis->Load(cylinderSDF);
  EXPECT_EQ(cylinderVis->GetAmbient(), common::Color::Black);
  EXPECT_EQ(cylinderVis->GetDiffuse(), common::Color::Black);
  EXPECT_EQ(cylinderVis->GetSpecular(), common::Color::Black);
  EXPECT_EQ(cylinderVis->GetEmissive(), common::Color::Black);

  sdf::ElementPtr cylinderSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF2);
  sdf::readString(GetVisualSDFString("visual_cylinder_color", "cylinder",
      gazebo::math::Pose::Zero, 0, true, "Gazebo/Grey", true,
      common::Color::Green, common::Color::Blue, common::Color::Red,
      common::Color::Yellow), cylinderSDF2);
  gazebo::rendering::VisualPtr cylinderVis2(
      new gazebo::rendering::Visual("cylinder_visual2", scene));
  cylinderVis2->Load(cylinderSDF2);
  EXPECT_EQ(cylinderVis2->GetAmbient(), common::Color::Green);
  EXPECT_EQ(cylinderVis2->GetDiffuse(), common::Color::Blue);
  EXPECT_EQ(cylinderVis2->GetSpecular(), common::Color::Red);
  EXPECT_EQ(cylinderVis2->GetEmissive(), common::Color::Yellow);

  // test changing ambient/diffuse/specular colors
  {
    common::Color color(0.1, 0.2, 0.3, 0.4);
    cylinderVis->SetAmbient(color);
    EXPECT_EQ(cylinderVis->GetAmbient(), color);
  }
  {
    common::Color color(1.0, 1.0, 1.0, 1.0);
    cylinderVis->SetDiffuse(color);
    EXPECT_EQ(cylinderVis->GetDiffuse(), color);
  }
  {
    common::Color color(0.5, 0.6, 0.4, 0.0);
    cylinderVis->SetSpecular(color);
    EXPECT_EQ(cylinderVis->GetSpecular(), color);
  }
  {
    common::Color color(0.9, 0.8, 0.7, 0.6);
    cylinderVis->SetEmissive(color);
    EXPECT_EQ(cylinderVis->GetEmissive(), color);
  }

  {
    common::Color color(0.0, 0.0, 0.0, 0.0);
    cylinderVis2->SetAmbient(color);
    EXPECT_EQ(cylinderVis2->GetAmbient(), color);
  }
  {
    common::Color color(1.0, 1.0, 1.0, 1.0);
    cylinderVis2->SetDiffuse(color);
    EXPECT_EQ(cylinderVis2->GetDiffuse(), color);
  }
  // test with color values that are out of range but should still work,
  // rendering engine should clamp the values internally
  {
    common::Color color(5.0, 5.0, 5.5, 5.1);
    cylinderVis2->SetSpecular(color);
    EXPECT_EQ(cylinderVis2->GetSpecular(), color);
  }
  {
    common::Color color(-5.0, -5.0, -5.5, -5.1);
    cylinderVis2->SetEmissive(color);
    EXPECT_EQ(cylinderVis2->GetEmissive(), color);
  }
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, ColorMaterial)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  std::string materialName = "Test/Grey";
  CreateColorMaterial(materialName, common::Color(0.3, 0.3, 0.3, 1.0),
      common::Color(0.7, 0.7, 0.7, 1.0), common::Color(0.01, 0.01, 0.01, 1.0),
      common::Color::Black);

  // test with a visual that only has a material name and no color components.
  std::string visualName = "boxMaterialColor";
  math::Pose visualPose = math::Pose::Zero;
  std::stringstream visualString;
  visualString
      << "<sdf version='" << SDF_VERSION << "'>"
      << "  <visual name='" << visualName << "'>"
      << "    <pose>" << visualPose << "</pose>"
      << "    <geometry>"
      << "      <box>"
      << "        <size>1.0 1.0 1.0</size>"
      << "      </box>"
      << "    </geometry>"
      << "    <material>"
      << "      <script>"
      << "        <uri>file://media/materials/scripts/gazebo.material</uri>"
      << "        <name>" << materialName << "</name>"
      << "      </script>"
      << "    </material>"
      << "  </visual>"
      << "</sdf>";

  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(visualString.str(), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", scene));
  boxVis->Load(boxSDF);

  EXPECT_TRUE(
      boxVis->GetMaterialName().find(materialName) != std::string::npos);

  // Verify the visual color components are the same as the ones specified in
  // the material script
  EXPECT_EQ(boxVis->GetAmbient(), common::Color(0.3, 0.3, 0.3, 1.0));
  EXPECT_EQ(boxVis->GetDiffuse(), common::Color(0.7, 0.7, 0.7, 1.0));
  EXPECT_EQ(boxVis->GetSpecular(), common::Color(0.01, 0.01, 0.01, 1.0));
  EXPECT_EQ(boxVis->GetEmissive(), common::Color::Black);

  // test changing diffuse colors and verify color again.
  common::Color redColor(1.0, 0.0, 0.0, 1.0);
  boxVis->SetDiffuse(redColor);
  EXPECT_EQ(boxVis->GetAmbient(), common::Color(0.3, 0.3, 0.3, 1.0));
  EXPECT_EQ(boxVis->GetDiffuse(), redColor);
  EXPECT_EQ(boxVis->GetSpecular(), common::Color(0.01, 0.01, 0.01, 1.0));
  EXPECT_EQ(boxVis->GetEmissive(), common::Color::Black);

  // test setting a different material name
  std::string greenMaterialName = "Test/Green";
  CreateColorMaterial(greenMaterialName, common::Color(0.0, 1.0, 0.0, 1.0),
      common::Color(0.0, 1.0, 0.0, 1.0), common::Color(0.1, 0.1, 0.1, 1.0),
      common::Color::Black);
  boxVis->SetMaterial(greenMaterialName);
  EXPECT_TRUE(
      boxVis->GetMaterialName().find(greenMaterialName) != std::string::npos);

  // Verify the visual color components are the same as the ones in the new
  // material script
  EXPECT_EQ(boxVis->GetAmbient(), common::Color(0.0, 1.0, 0.0, 1.0));
  EXPECT_EQ(boxVis->GetDiffuse(), common::Color(0.0, 1.0, 0.0, 1.0));
  EXPECT_EQ(boxVis->GetSpecular(), common::Color(0.1, 0.1, 0.1, 1.0));
  EXPECT_EQ(boxVis->GetEmissive(), common::Color::Black);

  // test setting back to original material color
  boxVis->SetMaterial(materialName);
  EXPECT_TRUE(
      boxVis->GetMaterialName().find(materialName) != std::string::npos);

  // Verify the visual color components are the same as the ones in the
  // original material script
  EXPECT_EQ(boxVis->GetAmbient(), common::Color(0.3, 0.3, 0.3, 1.0));
  EXPECT_EQ(boxVis->GetDiffuse(), common::Color(0.7, 0.7, 0.7, 1.0));
  EXPECT_EQ(boxVis->GetSpecular(), common::Color(0.01, 0.01, 0.01, 1.0));
  EXPECT_EQ(boxVis->GetEmissive(), common::Color::Black);

  // test with a semi-transparent color material
  std::string redTransparentMaterialName = "Test/RedTransparent";
  CreateColorMaterial(redTransparentMaterialName,
      common::Color(1.0, 0.0, 0.0, 0.2), common::Color(1.0, 0.0, 0.0, 0.4),
      common::Color(0.1, 0.1, 0.1, 0.6), common::Color(1.0, 0.0, 0.0, 0.8));
  boxVis->SetMaterial(redTransparentMaterialName);
  EXPECT_TRUE(boxVis->GetMaterialName().find(redTransparentMaterialName)
      != std::string::npos);

  // Verify the visual color components are the same as the ones in the new
  // material script
  EXPECT_EQ(boxVis->GetAmbient(), common::Color(1.0, 0.0, 0.0, 0.2));
  EXPECT_EQ(boxVis->GetDiffuse(), common::Color(1.0, 0.0, 0.0, 0.4));
  EXPECT_EQ(boxVis->GetSpecular(), common::Color(0.1, 0.1, 0.1, 0.6));
  EXPECT_EQ(boxVis->GetEmissive(), common::Color(1.0, 0.0, 0.0, 0.8));

  // update transparency and verify diffuse alpha value has changed
  boxVis->SetTransparency(0.5f);
  EXPECT_DOUBLE_EQ(boxVis->GetTransparency(), 0.5f);
  EXPECT_EQ(boxVis->GetAmbient(), common::Color(1.0, 0.0, 0.0, 0.2));
  EXPECT_EQ(boxVis->GetDiffuse(), common::Color(1.0, 0.0, 0.0, 0.5));
  EXPECT_EQ(boxVis->GetSpecular(), common::Color(0.1, 0.1, 0.1, 0.6));
  EXPECT_EQ(boxVis->GetEmissive(), common::Color(1.0, 0.0, 0.0, 0.8));
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, UpdateMeshFromMsg)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != NULL);

  sdf::ElementPtr meshUpdateSDF(new sdf::Element);
  sdf::initFile("visual.sdf", meshUpdateSDF);
  sdf::readString(GetVisualSDFString("visual_mesh_update"), meshUpdateSDF);
  gazebo::rendering::VisualPtr meshUpdateVis(
      new gazebo::rendering::Visual("visual_mesh_update_visual", scene));
  meshUpdateVis->Load(meshUpdateSDF);

  EXPECT_EQ(meshUpdateVis->GetMeshName(), "unit_box");
  EXPECT_EQ(meshUpdateVis->GetSubMeshName(), "");

  msgs::VisualPtr visualMsg(new msgs::Visual);
  msgs::Geometry *geomMsg = visualMsg->mutable_geometry();
  geomMsg->set_type(msgs::Geometry::MESH);
  msgs::MeshGeom *meshMsg = geomMsg->mutable_mesh();
  std::string meshFile = "polaris_ranger_ev/meshes/polaris.dae";
  meshMsg->set_filename("model://" + meshFile);
  meshMsg->set_submesh("Steering_Wheel");
  meshUpdateVis->UpdateFromMsg(visualMsg);

  // verify new mesh and submesh names
  EXPECT_TRUE(meshUpdateVis->GetMeshName().find(meshFile) != std::string::npos);
  EXPECT_EQ(meshUpdateVis->GetSubMeshName(), "Steering_Wheel");

  // verify updated sdf
  sdf::ElementPtr visualUpdateSDF = meshUpdateVis->GetSDF();
  sdf::ElementPtr geomSDF = visualUpdateSDF->GetElement("geometry");
  sdf::ElementPtr meshSDF = geomSDF->GetElement("mesh");
  EXPECT_EQ(meshSDF->Get<std::string>("uri"), "model://" + meshFile);
  sdf::ElementPtr submeshSDF = meshSDF->GetElement("submesh");
  EXPECT_EQ(submeshSDF->Get<std::string>("name"), "Steering_Wheel");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
