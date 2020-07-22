/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <ignition/math/Rand.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/test/ServerFixture.hh"


using namespace gazebo;
class Visual_TEST : public RenderingFixture
{
};

namespace ignmath = ignition::math;

/////////////////////////////////////////////////
std::string GetVisualSDFString(const std::string &_name,
    const std::string &_geomType = "box",
    const ignition::math::Vector3d &_size = ignition::math::Vector3d::One,
    ignition::math::Pose3d _pose = ignition::math::Pose3d::Zero,
    double _transparency = 0, bool _castShadows = true,
    const std::string &_material = "Gazebo/Grey",
    bool _lighting = true,
    const ignmath::Color &_ambient = ignmath::Color::White,
    const ignmath::Color &_diffuse = ignmath::Color::White,
    const ignmath::Color &_specular = ignmath::Color::Black,
    const ignmath::Color &_emissive = ignmath::Color::Black)
{
  std::stringstream visualString;
  visualString
      << "<sdf version='" << SDF_VERSION <<"'>"
      << "  <visual name='" << _name << "'>"
      << "    <pose>" << _pose << "</pose>"
      << "    <geometry>";
  if (_geomType == "box")
  {
    visualString
      << "      <box>"
      << "        <size>" << _size << "</size>"
      << "      </box>";
  }
  else if (_geomType == "sphere")
  {
    visualString
      << "      <sphere>"
      << "        <radius>" << _size.X() * 0.5 << "</radius>"
      << "      </sphere>";
  }
  else if (_geomType == "cylinder")
  {
    visualString
      << "      <cylinder>"
      << "        <radius>" << _size.X() * 0.5 << "</radius>"
      << "        <length>" << _size.Z() << "</length>"
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
    const ignmath::Color &_ambient, const ignmath::Color &_diffuse,
    const ignmath::Color &_specular, const ignmath::Color &_emissive)
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
  SpawnBox("box", ignition::math::Vector3d::One,
      ignition::math::Vector3d(10, 10, 1),
      ignition::math::Vector3d::Zero);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  int sleep = 0;
  int maxSleep = 50;
  rendering::VisualPtr visual;
  while (!visual && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    visual = scene->GetVisual("box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_NE(visual, nullptr);

  // verify initial bounding box
  ignition::math::Vector3d bboxMin(-0.5, -0.5, -0.5);
  ignition::math::Vector3d bboxMax(0.5, 0.5, 0.5);
  ignition::math::Box boundingBox = visual->BoundingBox();
  EXPECT_EQ(boundingBox.Min(), bboxMin);
  EXPECT_EQ(boundingBox.Max(), bboxMax);

  // verify scale
  ignition::math::Vector3d scale = visual->Scale();
  EXPECT_EQ(scale, ignition::math::Vector3d::One);

  // set new scale
  ignition::math::Vector3d scaleToSet(2, 3, 4);
  visual->SetScale(scaleToSet);

  // verify new scale
  ignition::math::Vector3d newScale = visual->Scale();
  EXPECT_EQ(newScale, scaleToSet);
  EXPECT_EQ(newScale, ignition::math::Vector3d(2, 3, 4));

  // verify local bounding box dimensions remain the same
  ignition::math::Box newBoundingBox = visual->BoundingBox();
  EXPECT_EQ(newBoundingBox.Min(), bboxMin);
  EXPECT_EQ(newBoundingBox.Max(), bboxMax);

  // verify local bounding box dimensions with scale applied
  EXPECT_EQ(newScale*newBoundingBox.Min(), newScale*bboxMin);
  EXPECT_EQ(newScale*newBoundingBox.Max(), newScale*bboxMax);
  EXPECT_EQ(newScale*newBoundingBox.Min(),
      ignition::math::Vector3d(-1, -1.5, -2));
  EXPECT_EQ(newScale*newBoundingBox.Max(),
      ignition::math::Vector3d(1, 1.5, 2));

  // create empty visual and check bounding box
  gazebo::rendering::VisualPtr emptyVis(
      new gazebo::rendering::Visual("empty_visual", scene));
  ignition::math::Box emptyBoundingBox = emptyVis->BoundingBox();
  EXPECT_EQ(ignition::math::Vector3d::Zero, emptyBoundingBox.Min());
  EXPECT_EQ(ignition::math::Vector3d::Zero, emptyBoundingBox.Max());

  // spawn box with link offset
  double zOffset = 10.0;
  std::ostringstream linkOffsetStream;
  ignition::math::Pose3d linkOffsetPose1(0, 0, zOffset, 0, 0, 0);
  ignition::math::Vector3d linkOffsetSize(1, 1, 1);
  linkOffsetStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='link_offset_box'>"
    << "<link name ='body'>"
    << "  <pose>" << linkOffsetPose1 << "</pose>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <box><size>" << linkOffsetSize << "</size></box>"
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << "      <box><size>" << linkOffsetSize << "</size></box>"
    << "    </geometry>"
    << "  </visual>"
    << "</link>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(linkOffsetStream.str());

  sleep = 0;
  rendering::VisualPtr visualOffset;
  while (!visualOffset && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    visualOffset = scene->GetVisual("link_offset_box");
    common::Time::MSleep(1000);
    sleep++;
  }
  ASSERT_NE(visualOffset, nullptr);

  // verify bounding box
  ignition::math::Vector3d bboxMinOffset(-0.5, -0.5, -0.5 + zOffset);
  ignition::math::Vector3d bboxMaxOffset(0.5, 0.5, 0.5 + zOffset);
  ignition::math::Box boundingBoxOffset = visualOffset->BoundingBox();
  EXPECT_EQ(boundingBoxOffset.Min(), bboxMinOffset);
  EXPECT_EQ(boundingBoxOffset.Max(), bboxMaxOffset);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Geometry)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // check geometry with unit scale
  {
    // box geom
    sdf::ElementPtr boxSDF(new sdf::Element);
    sdf::initFile("visual.sdf", boxSDF);
    sdf::readString(GetVisualSDFString("visual_box", "box"), boxSDF);
    gazebo::rendering::VisualPtr boxVis(
        new gazebo::rendering::Visual("box_visual", scene));
    boxVis->Load(boxSDF);
    EXPECT_EQ(boxVis->GetGeometryType(), "box");
    EXPECT_EQ(boxVis->GetGeometrySize(), ignition::math::Vector3d::One);

    // sphere geom
    sdf::ElementPtr sphereSDF(new sdf::Element);
    sdf::initFile("visual.sdf", sphereSDF);
    sdf::readString(GetVisualSDFString("visual_sphere", "sphere"), sphereSDF);
    gazebo::rendering::VisualPtr sphereVis(
        new gazebo::rendering::Visual("sphere_visual", scene));
    sphereVis->Load(sphereSDF);
    EXPECT_EQ(sphereVis->GetGeometryType(), "sphere");
    EXPECT_EQ(sphereVis->GetGeometrySize(), ignition::math::Vector3d::One);

    // cylinder geom
    sdf::ElementPtr cylinderSDF(new sdf::Element);
    sdf::initFile("visual.sdf", cylinderSDF);
    sdf::readString(GetVisualSDFString("visual_cylinder", "cylinder"),
        cylinderSDF);
    gazebo::rendering::VisualPtr cylinderVis(
        new gazebo::rendering::Visual("cylinder_visual", scene));
    cylinderVis->Load(cylinderSDF);
    EXPECT_EQ(cylinderVis->GetGeometryType(), "cylinder");
    EXPECT_EQ(cylinderVis->GetGeometrySize(), ignition::math::Vector3d::One);
  }

  // non-unit scale
  {
    // box
    ignition::math::Vector3d boxSize(0.2, 0.4, 2.1);
    sdf::ElementPtr boxSDF(new sdf::Element);
    sdf::initFile("visual.sdf", boxSDF);
    sdf::readString(GetVisualSDFString("visual_box", "box", boxSize), boxSDF);
    gazebo::rendering::VisualPtr boxVis(
        new gazebo::rendering::Visual("box_visual", scene));
    boxVis->Load(boxSDF);
    EXPECT_EQ(boxVis->GetGeometrySize(), boxSize);
    EXPECT_EQ(boxVis->Scale(), boxSize);

    ignition::math::Vector3d boxScale(1.5, 2.4, 3.0);
    boxVis->SetScale(boxScale);
    EXPECT_EQ(boxVis->GetGeometrySize(), boxScale);
    EXPECT_EQ(boxVis->Scale(), boxScale);

    sdf::ElementPtr newBoxSDF = boxVis->GetSDF();
    sdf::ElementPtr newBoxGeomElem = newBoxSDF->GetElement("geometry");
    EXPECT_TRUE(newBoxGeomElem->HasElement("box"));
    sdf::ElementPtr newBoxGeomBoxElem = newBoxGeomElem->GetElement("box");
    EXPECT_EQ(
        newBoxGeomBoxElem->GetElement("size")->Get<ignition::math::Vector3d>(),
        boxScale);

    // sphere
    ignition::math::Vector3d sphereSize(3.2, 3.2, 3.2);
    sdf::ElementPtr sphereSDF(new sdf::Element);
    sdf::initFile("visual.sdf", sphereSDF);
    sdf::readString(GetVisualSDFString("visual_sphere", "sphere", sphereSize),
    sphereSDF);
    gazebo::rendering::VisualPtr sphereVis(
        new gazebo::rendering::Visual("sphere_visual", scene));
    sphereVis->Load(sphereSDF);
    EXPECT_EQ(sphereVis->GetGeometrySize(), sphereSize);
    EXPECT_EQ(sphereVis->Scale(), sphereSize);

    ignition::math::Vector3d sphereScale(3.5, 3.5, 3.5);
    sphereVis->SetScale(sphereScale);
    EXPECT_EQ(sphereVis->GetGeometrySize(), sphereScale);
    EXPECT_EQ(sphereVis->Scale(), sphereScale);

    sdf::ElementPtr newSphereSDF = sphereVis->GetSDF();
    sdf::ElementPtr newSphereGeomElem = newSphereSDF->GetElement("geometry");
    EXPECT_TRUE(newSphereGeomElem->HasElement("sphere"));
    sdf::ElementPtr newSphereGeomSphereElem =
        newSphereGeomElem->GetElement("sphere");
    EXPECT_DOUBLE_EQ(
        newSphereGeomSphereElem->GetElement("radius")->Get<double>(),
        sphereScale.X()*0.5);

    // cylinder
    ignition::math::Vector3d cylinderSize(52, 52, 0.6);
    sdf::ElementPtr cylinderSDF(new sdf::Element);
    sdf::initFile("visual.sdf", cylinderSDF);
    sdf::readString(GetVisualSDFString("visual_cylinder", "cylinder",
        cylinderSize), cylinderSDF);
    gazebo::rendering::VisualPtr cylinderVis(
        new gazebo::rendering::Visual("cylinder_visual", scene));
    cylinderVis->Load(cylinderSDF);
    EXPECT_EQ(cylinderVis->GetGeometrySize(), cylinderSize);
    EXPECT_EQ(cylinderVis->Scale(), cylinderSize);

    ignition::math::Vector3d cylinderScale(0.1, 0.1, 3.0);
    cylinderVis->SetScale(cylinderScale);
    EXPECT_EQ(cylinderVis->GetGeometrySize(), cylinderScale);
    EXPECT_EQ(cylinderVis->Scale(), cylinderScale);

    sdf::ElementPtr newCylinderSDF = cylinderVis->GetSDF();
    sdf::ElementPtr newCylinderGeomElem =
        newCylinderSDF->GetElement("geometry");
    EXPECT_TRUE(newCylinderGeomElem->HasElement("cylinder"));
    sdf::ElementPtr newCylinderGeomCylinderElem =
        newCylinderGeomElem->GetElement("cylinder");
    EXPECT_DOUBLE_EQ(
        newCylinderGeomCylinderElem->GetElement("radius")->Get<double>(),
        cylinderScale.X()*0.5);
    EXPECT_DOUBLE_EQ(
        newCylinderGeomCylinderElem->GetElement("length")->Get<double>(),
        cylinderScale.Z());
  }
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, CastShadows)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // load a model that casts shadows by default
  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("visual_box_has_shadows", "box",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true),
      boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", scene));
  boxVis->Load(boxSDF);
  EXPECT_TRUE(boxVis->GetCastShadows());

  // load another model that does not cast shadows by default
  sdf::ElementPtr boxSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF2);
  sdf::readString(GetVisualSDFString("visual_box_has_no_shadows", "box",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, false),
      boxSDF2);
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
  ASSERT_NE(scene, nullptr);

  sdf::ElementPtr sphereSDF(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF);
  sdf::readString(GetVisualSDFString("visual_sphere_no_transparency", "sphere",
      ignition::math::Vector3d::One,
      ignition::math::Pose3d::Zero, 0), sphereSDF);
  gazebo::rendering::VisualPtr sphereVis(
      new gazebo::rendering::Visual("sphere_visual", scene));
  sphereVis->Load(sphereSDF);
  EXPECT_DOUBLE_EQ(sphereVis->GetTransparency(), 0);

  sdf::ElementPtr sphereSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF2);
  sdf::readString(GetVisualSDFString("visual_sphere_semi_transparent", "sphere",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0.5),
      sphereSDF2);
  gazebo::rendering::VisualPtr sphereVis2(
      new gazebo::rendering::Visual("sphere_visual2", scene));
  sphereVis2->Load(sphereSDF2);
  EXPECT_DOUBLE_EQ(sphereVis2->GetTransparency(), 0.5f);

  // test changing transparency property
  float trans = 0.3f;
  sphereVis->SetTransparency(trans);
  EXPECT_DOUBLE_EQ(sphereVis->GetTransparency(), trans);
  sphereVis2->SetTransparency(1.0f);
  EXPECT_DOUBLE_EQ(sphereVis2->GetTransparency(), 1.0f);

  // Create another visual with geom as child of sphere_visual
  // sphere_visual is already semi-transparency so boxVis should inherit
  // this transparency
  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("box_visual", "box"), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("visual", sphereVis));
  boxVis->Load(boxSDF);
  EXPECT_FLOAT_EQ(boxVis->DerivedTransparency(), trans);
  // check diffuse to verify they have the correct alpha value
  EXPECT_FLOAT_EQ(boxVis->Diffuse().A(), 1.0f-trans);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, ChildTransparency)
{
  Load("worlds/empty.world");

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Create a visual as child of the world visual
  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();

  // Create a visual as child of vis1
  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis2", vis1));
  vis2->Load();

  // Check default transparency
  EXPECT_FLOAT_EQ(vis1->GetTransparency(), 0.0);
  EXPECT_FLOAT_EQ(vis2->GetTransparency(), 0.0);

  // Set vis1's transparency
  float trans = 0.1;
  vis1->SetTransparency(trans);
  EXPECT_FLOAT_EQ(vis1->GetTransparency(), trans);
  EXPECT_FLOAT_EQ(vis2->GetTransparency(), 0.0f);

  // Set vis1's transparency again
  float trans2 = 0.2;
  vis1->SetTransparency(trans2);
  EXPECT_FLOAT_EQ(vis1->GetTransparency(), trans2);
  EXPECT_FLOAT_EQ(vis2->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(vis2->DerivedTransparency(), trans2);

  // Set vis2's transparency
  float vis2Transparency = 0.4;
  vis2->SetTransparency(vis2Transparency);
  EXPECT_FLOAT_EQ(vis1->GetTransparency(), trans2);
  EXPECT_FLOAT_EQ(vis2->GetTransparency(), vis2Transparency);
  EXPECT_FLOAT_EQ(vis2->DerivedTransparency(),
      1.0 - ((1.0 - trans2) * (1.0 - vis2Transparency)));
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, DerivedTransparency)
{
  // Load a world containing 3 simple shapes
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder;
  while ((!box || !sphere || !cylinder) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the model, link, and visuals were properly added
  // and verify initial transparency

  // box
  ASSERT_NE(box, nullptr);
  rendering::VisualPtr boxLink;
  for (unsigned int i = 0; i < box->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = box->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      boxLink = vis;
  }
  ASSERT_NE(boxLink, nullptr);

  rendering::VisualPtr boxVisual;
  for (unsigned int i = 0; i < boxLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = boxLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      boxVisual = vis;
  }
  ASSERT_NE(boxVisual, nullptr);

  EXPECT_FLOAT_EQ(box->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(boxLink->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(boxVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(box->DerivedTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(boxLink->DerivedTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(boxVisual->DerivedTransparency(), 0.0f);

  // sphere
  ASSERT_NE(sphere, nullptr);
  rendering::VisualPtr sphereLink;
  for (unsigned int i = 0; i < sphere->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = sphere->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      sphereLink = vis;
  }
  ASSERT_NE(sphereLink, nullptr);

  rendering::VisualPtr sphereVisual;
  for (unsigned int i = 0; i < sphereLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = sphereLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      sphereVisual = vis;
  }
  ASSERT_NE(sphereVisual, nullptr);

  EXPECT_FLOAT_EQ(sphere->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(sphereLink->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(sphereVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(sphere->DerivedTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(sphereLink->DerivedTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(sphereVisual->DerivedTransparency(), 0.0f);

  // cylinder
  ASSERT_NE(cylinder, nullptr);
  rendering::VisualPtr cylinderLink;
  for (unsigned int i = 0; i < cylinder->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = cylinder->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      cylinderLink = vis;
  }
  ASSERT_NE(cylinderLink, nullptr);

  rendering::VisualPtr cylinderVisual;
  for (unsigned int i = 0; i < cylinderLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = cylinderLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      cylinderVisual = vis;
  }
  ASSERT_NE(cylinderVisual, nullptr);

  EXPECT_FLOAT_EQ(cylinder->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(cylinderLink->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(cylinderVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(cylinder->DerivedTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(cylinderLink->DerivedTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(cylinderVisual->DerivedTransparency(), 0.0f);

  // update model transparency and verify derived transparency
  float newBoxTransparency = 0.4f;
  box->SetTransparency(newBoxTransparency);
  EXPECT_FLOAT_EQ(box->GetTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxLink->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(boxVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(box->DerivedTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxLink->DerivedTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxVisual->DerivedTransparency(), newBoxTransparency);

  float newSphereTransparency = 0.3f;
  sphere->SetTransparency(newSphereTransparency);
  EXPECT_FLOAT_EQ(sphere->GetTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereLink->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(sphereVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(sphere->DerivedTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereLink->DerivedTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereVisual->DerivedTransparency(), newSphereTransparency);

  float newCylinderTransparency = 0.25f;
  cylinder->SetTransparency(newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinder->GetTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->GetTransparency(), 0.0f);
  EXPECT_FLOAT_EQ(cylinderVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(cylinder->DerivedTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->DerivedTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderVisual->DerivedTransparency(),
      newCylinderTransparency);

  // update link transparency and verify derived transparency
  float newBoxLinkTransparency = 0.2f;
  boxLink->SetTransparency(newBoxLinkTransparency);
  EXPECT_FLOAT_EQ(box->GetTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxLink->GetTransparency(), newBoxLinkTransparency);
  EXPECT_FLOAT_EQ(boxVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(box->DerivedTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxLink->DerivedTransparency(),
      1.0 - ((1.0 - newBoxTransparency) * (1.0 - newBoxLinkTransparency)));
  EXPECT_FLOAT_EQ(boxVisual->DerivedTransparency(),
      1.0 -((1.0 - newBoxTransparency) * (1.0 - newBoxLinkTransparency)));

  float newSphereLinkTransparency = 0.9f;
  sphereLink->SetTransparency(newSphereLinkTransparency);
  EXPECT_FLOAT_EQ(sphere->GetTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereLink->GetTransparency(), newSphereLinkTransparency);
  EXPECT_FLOAT_EQ(sphereVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(sphere->DerivedTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereLink->DerivedTransparency(),
      1.0 - ((1.0 - newSphereTransparency) *
      (1.0 - newSphereLinkTransparency)));
  EXPECT_FLOAT_EQ(sphereVisual->DerivedTransparency(),
      1.0 - ((1.0 - newSphereTransparency) *
      (1.0 - newSphereLinkTransparency)));

  float newCylinderLinkTransparency = 0.02f;
  cylinderLink->SetTransparency(newCylinderLinkTransparency);
  EXPECT_FLOAT_EQ(cylinder->GetTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->GetTransparency(), newCylinderLinkTransparency);
  EXPECT_FLOAT_EQ(cylinderVisual->GetTransparency(), 0.0f);

  EXPECT_FLOAT_EQ(cylinder->DerivedTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->DerivedTransparency(),
      1.0 - ((1.0 - newCylinderTransparency) *
      (1.0 - newCylinderLinkTransparency)));
  EXPECT_FLOAT_EQ(cylinderVisual->DerivedTransparency(),
      1.0 - ((1.0 - newCylinderTransparency) *
      (1.0 - newCylinderLinkTransparency)));

  // update visual transparency and verify derived transparency
  float newBoxVisualTransparency = 0.6f;
  boxVisual->SetTransparency(newBoxVisualTransparency);
  EXPECT_FLOAT_EQ(box->GetTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxLink->GetTransparency(), newBoxLinkTransparency);
  EXPECT_FLOAT_EQ(boxVisual->GetTransparency(), newBoxVisualTransparency);

  EXPECT_FLOAT_EQ(box->DerivedTransparency(), newBoxTransparency);
  EXPECT_FLOAT_EQ(boxLink->DerivedTransparency(),
      1.0 - ((1.0 - newBoxTransparency) * (1.0 - newBoxLinkTransparency)));
  EXPECT_FLOAT_EQ(boxVisual->DerivedTransparency(),
      1.0 - ((1.0 - newBoxTransparency) * (1.0 - newBoxLinkTransparency) *
      (1.0 - newBoxVisualTransparency)));


  float newSphereVisualTransparency = 0.08f;
  sphereVisual->SetTransparency(newSphereVisualTransparency);
  EXPECT_FLOAT_EQ(sphere->GetTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereLink->GetTransparency(), newSphereLinkTransparency);
  EXPECT_FLOAT_EQ(sphereVisual->GetTransparency(), newSphereVisualTransparency);

  EXPECT_FLOAT_EQ(sphere->DerivedTransparency(), newSphereTransparency);
  EXPECT_FLOAT_EQ(sphereLink->DerivedTransparency(),
      1.0 - ((1.0 - newSphereTransparency) *
      (1.0 - newSphereLinkTransparency)));
  EXPECT_FLOAT_EQ(sphereVisual->DerivedTransparency(),
      1.0 - ((1.0 - newSphereTransparency) *
      (1.0 - newSphereLinkTransparency) *
      (1.0 - newSphereVisualTransparency)));

  float newCylinderVisualTransparency = 1.0f;
  cylinderVisual->SetTransparency(newCylinderVisualTransparency);
  EXPECT_FLOAT_EQ(cylinder->GetTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->GetTransparency(), newCylinderLinkTransparency);
  EXPECT_FLOAT_EQ(cylinderVisual->GetTransparency(),
      newCylinderVisualTransparency);

  EXPECT_FLOAT_EQ(cylinder->DerivedTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->DerivedTransparency(),
      1.0 - ((1.0 - newCylinderTransparency) *
      (1.0 - newCylinderLinkTransparency)));
  EXPECT_FLOAT_EQ(cylinderVisual->DerivedTransparency(),
      1.0 - ((1.0 - newCylinderTransparency) *
      (1.0 - newCylinderLinkTransparency) *
      (1.0 - newCylinderVisualTransparency)));

  // clone visual and verify transparency
  rendering::VisualPtr boxClone = box->Clone("boxClone",
      box->GetParent());
  rendering::VisualPtr sphereClone = sphere->Clone("sphereClone",
      sphere->GetParent());
  rendering::VisualPtr cylinderClone = cylinder->Clone("cylinderClone",
      cylinder->GetParent());

  std::queue<std::pair<rendering::VisualPtr, rendering::VisualPtr> >
      cloneVisuals;
  cloneVisuals.push(std::make_pair(box, boxClone));
  cloneVisuals.push(std::make_pair(sphere, sphereClone));
  cloneVisuals.push(std::make_pair(cylinder, cylinderClone));
  while (!cloneVisuals.empty())
  {
    auto visualPair = cloneVisuals.front();
    cloneVisuals.pop();
    EXPECT_FLOAT_EQ(visualPair.first->GetTransparency(),
        visualPair.second->GetTransparency());
    EXPECT_FLOAT_EQ(visualPair.first->DerivedTransparency(),
        visualPair.second->DerivedTransparency());

    EXPECT_FLOAT_EQ(visualPair.first->GetChildCount(),
        visualPair.second->GetChildCount());
    for (unsigned int i  = 0; i < visualPair.first->GetChildCount(); ++i)
    {
      // check transparency for model, link, and visual types. Other visuals
      // such as gui only visualizations have their own preset transparency
      // value
      rendering::Visual::VisualType type =
          visualPair.first->GetChild(i)->GetType();
      if (type == rendering::Visual::VT_MODEL ||
          type == rendering::Visual::VT_LINK ||
          type == rendering::Visual::VT_VISUAL)
      {
        cloneVisuals.push(std::make_pair(
            visualPair.first->GetChild(i), visualPair.second->GetChild(i)));
      }
    }
  }

  // verify inherit transparency
  EXPECT_TRUE(cylinder->InheritTransparency());
  EXPECT_TRUE(cylinderLink->InheritTransparency());
  EXPECT_TRUE(cylinderVisual->InheritTransparency());

  cylinderLink->SetInheritTransparency(false);
  EXPECT_TRUE(cylinder->InheritTransparency());
  EXPECT_FALSE(cylinderLink->InheritTransparency());
  EXPECT_TRUE(cylinderVisual->InheritTransparency());

  EXPECT_FLOAT_EQ(cylinder->DerivedTransparency(), newCylinderTransparency);
  EXPECT_FLOAT_EQ(cylinderLink->DerivedTransparency(),
      newCylinderLinkTransparency);
  EXPECT_FLOAT_EQ(cylinderVisual->DerivedTransparency(),
      1.0 - ((1.0 - newCylinderLinkTransparency) *
      (1.0 - newCylinderVisualTransparency)));
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Wireframe)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  sdf::ElementPtr sphereSDF(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF);
  sdf::readString(GetVisualSDFString("sphere_visual", "sphere",
      ignition::math::Vector3d::One,
      ignition::math::Pose3d::Zero, 0), sphereSDF);
  gazebo::rendering::VisualPtr sphereVis(
      new gazebo::rendering::Visual("sphere_visual", scene));
  sphereVis->Load(sphereSDF);

  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("box_visual", "box",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", sphereVis));
  boxVis->Load(boxSDF);

  // wireframe should be disabled by default
  EXPECT_FALSE(sphereVis->Wireframe());
  EXPECT_FALSE(boxVis->Wireframe());

  // enable wireframe for box visual
  boxVis->SetWireframe(true);
  EXPECT_FALSE(sphereVis->Wireframe());
  EXPECT_TRUE(boxVis->Wireframe());

  // disable wireframe for box visual
  boxVis->SetWireframe(false);
  EXPECT_FALSE(sphereVis->Wireframe());
  EXPECT_FALSE(boxVis->Wireframe());

  // enable wireframe for sphere visual, it should cascade down to box visual
  sphereVis->SetWireframe(true);
  EXPECT_TRUE(sphereVis->Wireframe());
  EXPECT_TRUE(boxVis->Wireframe());

  // reset everything
  sphereVis->SetWireframe(false);
  EXPECT_FALSE(sphereVis->Wireframe());
  EXPECT_FALSE(boxVis->Wireframe());

  // check that certain visual types won't be affected by wireframe mode
  boxVis->SetType(rendering::Visual::VT_GUI);
  boxVis->SetWireframe(true);
  EXPECT_FALSE(sphereVis->Wireframe());
  EXPECT_FALSE(boxVis->Wireframe());

  sphereVis->SetWireframe(true);
  EXPECT_TRUE(sphereVis->Wireframe());
  EXPECT_FALSE(boxVis->Wireframe());
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Material)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("visual_box_red", "box",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true,
      "Gazebo/Red"), boxSDF);
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
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true,
      "Gazebo/White", true), boxSDF2);
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
TEST_F(Visual_TEST, ChildMaterial)
{
  Load("worlds/empty.world");

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Create a visual as child of the world visual
  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();

  // Create a visual as child of vis1
  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis2", vis1));
  vis2->Load();

  // Check material is empty
  EXPECT_TRUE(vis1->GetMaterialName().empty());
  EXPECT_TRUE(vis2->GetMaterialName().empty());

  // Set vis1's material with default cascade
  std::string defaultCascade = "Gazebo/Grey";
  vis1->SetMaterial(defaultCascade);
  EXPECT_TRUE(vis1->GetMaterialName().find(defaultCascade) !=
      std::string::npos);
  EXPECT_TRUE(vis2->GetMaterialName().find(defaultCascade) !=
      std::string::npos);

  // Set vis1's material with explicit cascade
  std::string explicitCascade = "Gazebo/Red";
  vis1->SetMaterial(explicitCascade, true, true);
  EXPECT_TRUE(vis1->GetMaterialName().find(explicitCascade) !=
      std::string::npos);
  EXPECT_TRUE(vis2->GetMaterialName().find(explicitCascade) !=
      std::string::npos);

  // Set vis1's material with no cascade
  std::string noCascade = "Gazebo/Green";
  vis1->SetMaterial(noCascade, true, false);
  EXPECT_TRUE(vis1->GetMaterialName().find(noCascade) !=
      std::string::npos);
  EXPECT_TRUE(vis2->GetMaterialName().find(explicitCascade) !=
      std::string::npos);

  // Set vis2's material
  std::string vis2Material = "Gazebo/Blue";
  vis2->SetMaterial(vis2Material);
  EXPECT_TRUE(vis1->GetMaterialName().find(noCascade) !=
      std::string::npos);
  EXPECT_TRUE(vis2->GetMaterialName().find(vis2Material) !=
      std::string::npos);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Lighting)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  sdf::ElementPtr cylinderSDF(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF);
  sdf::readString(GetVisualSDFString("visual_cylinder_lighting", "cylinder",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true,
      "Gazebo/Grey", true), cylinderSDF);
  gazebo::rendering::VisualPtr cylinderVis(
      new gazebo::rendering::Visual("cylinder_visual", scene));
  cylinderVis->Load(cylinderSDF);
  EXPECT_TRUE(cylinderVis->GetLighting());

  sdf::ElementPtr cylinderSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF2);
  sdf::readString(GetVisualSDFString("visual_cylinder_no_lighting", "cylinder",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true,
      "Gazebo/Grey", false), cylinderSDF2);
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
  ASSERT_NE(scene, nullptr);

  sdf::ElementPtr cylinderSDF(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF);
  sdf::readString(GetVisualSDFString("visual_cylinder_black", "cylinder",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true,
      "Gazebo/Grey", true, ignmath::Color::Black, ignmath::Color::Black,
      ignmath::Color::Black, ignmath::Color::Black), cylinderSDF);
  gazebo::rendering::VisualPtr cylinderVis(
      new gazebo::rendering::Visual("cylinder_visual", scene));
  cylinderVis->Load(cylinderSDF);
  EXPECT_EQ(cylinderVis->Ambient(), ignmath::Color::Black);
  EXPECT_EQ(cylinderVis->Diffuse(), ignmath::Color::Black);
  EXPECT_EQ(cylinderVis->Specular(), ignmath::Color::Black);
  EXPECT_EQ(cylinderVis->Emissive(), ignmath::Color::Black);

  sdf::ElementPtr cylinderSDF2(new sdf::Element);
  sdf::initFile("visual.sdf", cylinderSDF2);
  sdf::readString(GetVisualSDFString("visual_cylinder_color", "cylinder",
      ignition::math::Vector3d::One, ignition::math::Pose3d::Zero, 0, true,
      "Gazebo/Grey", true, ignmath::Color::Green, ignmath::Color::Blue,
      ignmath::Color::Red, ignmath::Color::Yellow), cylinderSDF2);
  gazebo::rendering::VisualPtr cylinderVis2(
      new gazebo::rendering::Visual("cylinder_visual2", scene));
  cylinderVis2->Load(cylinderSDF2);
  EXPECT_EQ(cylinderVis2->Ambient(), ignmath::Color::Green);
  EXPECT_EQ(cylinderVis2->Diffuse(), ignmath::Color::Blue);
  EXPECT_EQ(cylinderVis2->Specular(), ignmath::Color::Red);
  EXPECT_EQ(cylinderVis2->Emissive(), ignmath::Color::Yellow);

  // test changing ambient/diffuse/specular colors
  {
    ignmath::Color color(0.1, 0.2, 0.3, 0.4);
    cylinderVis->SetAmbient(color);
    EXPECT_EQ(cylinderVis->Ambient(), color);
  }
  {
    ignmath::Color color(1.0, 1.0, 1.0, 1.0);
    cylinderVis->SetDiffuse(color);
    EXPECT_EQ(cylinderVis->Diffuse(), color);
  }
  {
    ignmath::Color color(0.5, 0.6, 0.4, 0.0);
    cylinderVis->SetSpecular(color);
    EXPECT_EQ(cylinderVis->Specular(), color);
  }
  {
    ignmath::Color color(0.9, 0.8, 0.7, 0.6);
    cylinderVis->SetEmissive(color);
    EXPECT_EQ(cylinderVis->Emissive(), color);
  }

  {
    ignmath::Color color(0.0, 0.0, 0.0, 0.0);
    cylinderVis2->SetAmbient(color);
    EXPECT_EQ(cylinderVis2->Ambient(), color);
  }
  {
    ignmath::Color color(1.0, 1.0, 1.0, 1.0);
    cylinderVis2->SetDiffuse(color);
    EXPECT_EQ(cylinderVis2->Diffuse(), color);
  }
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, ChildColor)
{
  Load("worlds/empty.world");

  // Get scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Create a visual as child of the world visual
  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();

  // Create a visual as child of vis1
  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis2", vis1));
  vis2->Load();

  // Check default colors
  EXPECT_EQ(vis1->Ambient(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis1->Emissive(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis1->Specular(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis1->Diffuse(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis2->Ambient(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis2->Emissive(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis2->Specular(), ignmath::Color(0, 0, 0, 0));
  EXPECT_EQ(vis2->Diffuse(), ignmath::Color(0, 0, 0, 0));

  // Set vis1's color with default cascade
  ignmath::Color defaultCascadeAmbient(0.1, 0, 0, 1);
  ignmath::Color defaultCascadeEmissive(0.2, 0, 0, 1);
  ignmath::Color defaultCascadeSpecular(0.3, 0, 0, 1);
  ignmath::Color defaultCascadeDiffuse(0.4, 0, 0, 1);
  vis1->SetAmbient(defaultCascadeAmbient);
  vis1->SetEmissive(defaultCascadeEmissive);
  vis1->SetSpecular(defaultCascadeSpecular);
  vis1->SetDiffuse(defaultCascadeDiffuse);
  EXPECT_EQ(vis1->Ambient(), defaultCascadeAmbient);
  EXPECT_EQ(vis1->Emissive(), defaultCascadeEmissive);
  EXPECT_EQ(vis1->Specular(), defaultCascadeSpecular);
  EXPECT_EQ(vis1->Diffuse(), defaultCascadeDiffuse);
  EXPECT_EQ(vis2->Ambient(), defaultCascadeAmbient);
  EXPECT_EQ(vis2->Emissive(), defaultCascadeEmissive);
  EXPECT_EQ(vis2->Specular(), defaultCascadeSpecular);
  EXPECT_EQ(vis2->Diffuse(), defaultCascadeDiffuse);

  // Set vis1's color with explicit cascade
  ignmath::Color explicitCascadeAmbient(0, 0.1, 0, 1);
  ignmath::Color explicitCascadeEmissive(0, 0.2, 0, 1);
  ignmath::Color explicitCascadeSpecular(0, 0.3, 0, 1);
  ignmath::Color explicitCascadeDiffuse(0, 0.4, 0, 1);
  vis1->SetAmbient(explicitCascadeAmbient, true);
  vis1->SetEmissive(explicitCascadeEmissive, true);
  vis1->SetSpecular(explicitCascadeSpecular, true);
  vis1->SetDiffuse(explicitCascadeDiffuse, true);
  EXPECT_EQ(vis1->Ambient(), explicitCascadeAmbient);
  EXPECT_EQ(vis1->Emissive(), explicitCascadeEmissive);
  EXPECT_EQ(vis1->Specular(), explicitCascadeSpecular);
  EXPECT_EQ(vis1->Diffuse(), explicitCascadeDiffuse);
  EXPECT_EQ(vis2->Ambient(), explicitCascadeAmbient);
  EXPECT_EQ(vis2->Emissive(), explicitCascadeEmissive);
  EXPECT_EQ(vis2->Specular(), explicitCascadeSpecular);
  EXPECT_EQ(vis2->Diffuse(), explicitCascadeDiffuse);

  // Set vis1's color with no cascade
  ignmath::Color noCascadeAmbient(0, 0, 0.1, 1);
  ignmath::Color noCascadeEmissive(0, 0, 0.2, 1);
  ignmath::Color noCascadeSpecular(0, 0, 0.3, 1);
  ignmath::Color noCascadeDiffuse(0, 0, 0.4, 1);
  vis1->SetAmbient(noCascadeAmbient, false);
  vis1->SetEmissive(noCascadeEmissive, false);
  vis1->SetSpecular(noCascadeSpecular, false);
  vis1->SetDiffuse(noCascadeDiffuse, false);
  EXPECT_EQ(vis1->Ambient(), noCascadeAmbient);
  EXPECT_EQ(vis1->Emissive(), noCascadeEmissive);
  EXPECT_EQ(vis1->Specular(), noCascadeSpecular);
  EXPECT_EQ(vis1->Diffuse(), noCascadeDiffuse);
  EXPECT_EQ(vis2->Ambient(), explicitCascadeAmbient);
  EXPECT_EQ(vis2->Emissive(), explicitCascadeEmissive);
  EXPECT_EQ(vis2->Specular(), explicitCascadeSpecular);
  EXPECT_EQ(vis2->Diffuse(), explicitCascadeDiffuse);

  // Set vis2's color
  ignmath::Color vis2Ambient(0.1, 0.1, 0.1, 1);
  ignmath::Color vis2Emissive(0.1, 0.2, 0.2, 1);
  ignmath::Color vis2Specular(0.1, 0.3, 0.3, 1);
  ignmath::Color vis2Diffuse(0.1, 0.4, 0.4, 1);
  vis2->SetAmbient(vis2Ambient);
  vis2->SetEmissive(vis2Emissive);
  vis2->SetSpecular(vis2Specular);
  vis2->SetDiffuse(vis2Diffuse);
  EXPECT_EQ(vis1->Ambient(), noCascadeAmbient);
  EXPECT_EQ(vis1->Emissive(), noCascadeEmissive);
  EXPECT_EQ(vis1->Specular(), noCascadeSpecular);
  EXPECT_EQ(vis1->Diffuse(), noCascadeDiffuse);
  EXPECT_EQ(vis2->Ambient(), vis2Ambient);
  EXPECT_EQ(vis2->Emissive(), vis2Emissive);
  EXPECT_EQ(vis2->Specular(), vis2Specular);
  EXPECT_EQ(vis2->Diffuse(), vis2Diffuse);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, ColorMaterial)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  std::string materialName = "Test/Grey";
  CreateColorMaterial(materialName, ignmath::Color(0.3, 0.3, 0.3, 1.0),
      ignmath::Color(0.7, 0.7, 0.7, 1.0), ignmath::Color(0.01, 0.01, 0.01, 1.0),
      ignmath::Color::Black);

  // test with a visual that only has a material name and no color components.
  std::string visualName = "boxMaterialColor";
  ignition::math::Pose3d visualPose = ignition::math::Pose3d::Zero;
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
  EXPECT_EQ(boxVis->Ambient(), ignmath::Color(0.3, 0.3, 0.3, 1.0));
  EXPECT_EQ(boxVis->Diffuse(), ignmath::Color(0.7, 0.7, 0.7, 1.0));
  EXPECT_EQ(boxVis->Specular(), ignmath::Color(0.01, 0.01, 0.01, 1.0));
  EXPECT_EQ(boxVis->Emissive(), ignmath::Color::Black);

  // test changing diffuse colors and verify color again.
  ignmath::Color redColor(1.0, 0.0, 0.0, 1.0);
  boxVis->SetDiffuse(redColor);
  EXPECT_EQ(boxVis->Ambient(), ignmath::Color(0.3, 0.3, 0.3, 1.0));
  EXPECT_EQ(boxVis->Diffuse(), redColor);
  EXPECT_EQ(boxVis->Specular(), ignmath::Color(0.01, 0.01, 0.01, 1.0));
  EXPECT_EQ(boxVis->Emissive(), ignmath::Color::Black);

  // test setting a different material name
  std::string greenMaterialName = "Test/Green";
  CreateColorMaterial(greenMaterialName, ignmath::Color(0.0, 1.0, 0.0, 1.0),
      ignmath::Color(0.0, 1.0, 0.0, 1.0), ignmath::Color(0.1, 0.1, 0.1, 1.0),
      ignmath::Color::Black);
  boxVis->SetMaterial(greenMaterialName);
  EXPECT_TRUE(
      boxVis->GetMaterialName().find(greenMaterialName) != std::string::npos);

  // Verify the visual color components are the same as the ones in the new
  // material script
  EXPECT_EQ(boxVis->Ambient(), ignmath::Color(0.0, 1.0, 0.0, 1.0));
  EXPECT_EQ(boxVis->Diffuse(), ignmath::Color(0.0, 1.0, 0.0, 1.0));
  EXPECT_EQ(boxVis->Specular(), ignmath::Color(0.1, 0.1, 0.1, 1.0));
  EXPECT_EQ(boxVis->Emissive(), ignmath::Color::Black);

  // test setting back to original material color
  boxVis->SetMaterial(materialName);
  EXPECT_TRUE(
      boxVis->GetMaterialName().find(materialName) != std::string::npos);

  // Verify the visual color components are the same as the ones in the
  // original material script
  EXPECT_EQ(boxVis->Ambient(), ignmath::Color(0.3, 0.3, 0.3, 1.0));
  EXPECT_EQ(boxVis->Diffuse(), ignmath::Color(0.7, 0.7, 0.7, 1.0));
  EXPECT_EQ(boxVis->Specular(), ignmath::Color(0.01, 0.01, 0.01, 1.0));
  EXPECT_EQ(boxVis->Emissive(), ignmath::Color::Black);

  // test with a semi-transparent color material
  std::string redTransparentMaterialName = "Test/RedTransparent";
  CreateColorMaterial(redTransparentMaterialName,
      ignmath::Color(1.0, 0.0, 0.0, 0.2), ignmath::Color(1.0, 0.0, 0.0, 0.4),
      ignmath::Color(0.1, 0.1, 0.1, 0.6), ignmath::Color(1.0, 0.0, 0.0, 0.8));
  boxVis->SetMaterial(redTransparentMaterialName);
  EXPECT_TRUE(boxVis->GetMaterialName().find(redTransparentMaterialName)
      != std::string::npos);

  // Verify the visual color components are the same as the ones in the new
  // material script
  EXPECT_EQ(boxVis->Ambient(), ignmath::Color(1.0, 0.0, 0.0, 0.2));
  EXPECT_EQ(boxVis->Diffuse(), ignmath::Color(1.0, 0.0, 0.0, 0.4));
  EXPECT_EQ(boxVis->Specular(), ignmath::Color(0.1, 0.1, 0.1, 0.6));
  EXPECT_EQ(boxVis->Emissive(), ignmath::Color(1.0, 0.0, 0.0, 0.8));

  // update transparency and verify diffuse alpha value has changed
  boxVis->SetTransparency(0.5f);
  EXPECT_DOUBLE_EQ(boxVis->GetTransparency(), 0.5f);
  EXPECT_EQ(boxVis->Ambient(), ignmath::Color(1.0, 0.0, 0.0, 0.2));
  EXPECT_EQ(boxVis->Diffuse(), ignmath::Color(1.0, 0.0, 0.0, 0.5));
  EXPECT_EQ(boxVis->Specular(), ignmath::Color(0.1, 0.1, 0.1, 0.6));
  EXPECT_EQ(boxVis->Emissive(), ignmath::Color(1.0, 0.0, 0.0, 0.8));
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, UpdateMeshFromMsg)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

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
TEST_F(Visual_TEST, GetAncestors)
{
  Load("worlds/blank.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Get world visual
  gazebo::rendering::VisualPtr world = scene->WorldVisual();
  ASSERT_NE(world, nullptr);

  // Create a visual as child of the world visual
  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();

  // Create a visual as child of vis1
  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis2", vis1));
  vis2->Load();

  // Create a visual as child of vis2
  gazebo::rendering::VisualPtr vis3_1;
  vis3_1.reset(new gazebo::rendering::Visual("vis3_1", vis2));
  vis3_1->Load();

  // Create one more visual as child of vis2
  gazebo::rendering::VisualPtr vis3_2;
  vis3_2.reset(new gazebo::rendering::Visual("vis3_2", vis2));
  vis3_2->Load();

  // Create a visual as child of vis3_1
  gazebo::rendering::VisualPtr vis4;
  vis4.reset(new gazebo::rendering::Visual("vis4", vis3_1));
  vis4->Load();

  // Check depths
  EXPECT_EQ(world->GetDepth(), 0u);
  EXPECT_EQ(vis1->GetDepth(), 1u);
  EXPECT_EQ(vis2->GetDepth(), 2u);
  EXPECT_EQ(vis3_1->GetDepth(), 3u);
  EXPECT_EQ(vis3_2->GetDepth(), 3u);
  EXPECT_EQ(vis4->GetDepth(), 4u);

  // Check parents
  EXPECT_EQ(world->GetParent(), nullptr);
  EXPECT_EQ(vis1->GetParent(), world);
  EXPECT_EQ(vis2->GetParent(), vis1);
  EXPECT_EQ(vis3_1->GetParent(), vis2);
  EXPECT_EQ(vis3_2->GetParent(), vis2);
  EXPECT_EQ(vis4->GetParent(), vis3_1);

  // Check that world is its own root
  EXPECT_EQ(world->GetRootVisual(), world);
  // Check that vis1 is the root for all others
  EXPECT_EQ(vis1->GetRootVisual(), vis1);
  EXPECT_EQ(vis2->GetRootVisual(), vis1);
  EXPECT_EQ(vis3_1->GetRootVisual(), vis1);
  EXPECT_EQ(vis3_2->GetRootVisual(), vis1);
  EXPECT_EQ(vis4->GetRootVisual(), vis1);

  // Check that world is 0th ancestor for all of them
  EXPECT_EQ(world->GetNthAncestor(0), world);
  EXPECT_EQ(vis1->GetNthAncestor(0), world);
  EXPECT_EQ(vis2->GetNthAncestor(0), world);
  EXPECT_EQ(vis3_1->GetNthAncestor(0), world);
  EXPECT_EQ(vis3_2->GetNthAncestor(0), world);
  EXPECT_EQ(vis4->GetNthAncestor(0), world);

  // Check that the 1st ancestor is the root visual
  EXPECT_EQ(world->GetNthAncestor(1), nullptr);
  EXPECT_EQ(vis1->GetNthAncestor(1), vis1->GetRootVisual());
  EXPECT_EQ(vis2->GetNthAncestor(1), vis2->GetRootVisual());
  EXPECT_EQ(vis3_1->GetNthAncestor(1), vis3_1->GetRootVisual());
  EXPECT_EQ(vis3_2->GetNthAncestor(1), vis3_2->GetRootVisual());
  EXPECT_EQ(vis4->GetNthAncestor(1), vis4->GetRootVisual());

  // Check 2nd ancestor
  EXPECT_EQ(world->GetNthAncestor(2), nullptr);
  EXPECT_EQ(vis1->GetNthAncestor(2), nullptr);
  EXPECT_EQ(vis2->GetNthAncestor(2), vis2);
  EXPECT_EQ(vis3_1->GetNthAncestor(2), vis2);
  EXPECT_EQ(vis3_2->GetNthAncestor(2), vis2);
  EXPECT_EQ(vis4->GetNthAncestor(2), vis2);

  // Check 3rd ancestor
  EXPECT_EQ(world->GetNthAncestor(3), nullptr);
  EXPECT_EQ(vis1->GetNthAncestor(3), nullptr);
  EXPECT_EQ(vis2->GetNthAncestor(3), nullptr);
  EXPECT_EQ(vis3_1->GetNthAncestor(3), vis3_1);
  EXPECT_EQ(vis3_2->GetNthAncestor(3), vis3_2);
  EXPECT_EQ(vis4->GetNthAncestor(3), vis3_1);

  // Check 4th ancestor
  EXPECT_EQ(world->GetNthAncestor(4), nullptr);
  EXPECT_EQ(vis1->GetNthAncestor(4), nullptr);
  EXPECT_EQ(vis2->GetNthAncestor(4), nullptr);
  EXPECT_EQ(vis3_1->GetNthAncestor(4), nullptr);
  EXPECT_EQ(vis3_2->GetNthAncestor(4), nullptr);
  EXPECT_EQ(vis4->GetNthAncestor(4), vis4);

  // Check if it is ancestor / descendant

  // world
  EXPECT_FALSE(world->IsAncestorOf(world));
  EXPECT_TRUE(world->IsAncestorOf(vis1));
  EXPECT_TRUE(world->IsAncestorOf(vis2));
  EXPECT_TRUE(world->IsAncestorOf(vis3_1));
  EXPECT_TRUE(world->IsAncestorOf(vis3_2));
  EXPECT_TRUE(world->IsAncestorOf(vis4));

  EXPECT_FALSE(world->IsDescendantOf(world));
  EXPECT_FALSE(world->IsDescendantOf(vis1));
  EXPECT_FALSE(world->IsDescendantOf(vis2));
  EXPECT_FALSE(world->IsDescendantOf(vis3_1));
  EXPECT_FALSE(world->IsDescendantOf(vis3_2));
  EXPECT_FALSE(world->IsDescendantOf(vis4));

  // vis1
  EXPECT_FALSE(vis1->IsAncestorOf(world));
  EXPECT_FALSE(vis1->IsAncestorOf(vis1));
  EXPECT_TRUE(vis1->IsAncestorOf(vis2));
  EXPECT_TRUE(vis1->IsAncestorOf(vis3_1));
  EXPECT_TRUE(vis1->IsAncestorOf(vis3_2));
  EXPECT_TRUE(vis1->IsAncestorOf(vis4));

  EXPECT_TRUE(vis1->IsDescendantOf(world));
  EXPECT_FALSE(vis1->IsDescendantOf(vis1));
  EXPECT_FALSE(vis1->IsDescendantOf(vis2));
  EXPECT_FALSE(vis1->IsDescendantOf(vis3_1));
  EXPECT_FALSE(vis1->IsDescendantOf(vis3_2));
  EXPECT_FALSE(vis1->IsDescendantOf(vis4));

  // vis2
  EXPECT_FALSE(vis2->IsAncestorOf(world));
  EXPECT_FALSE(vis2->IsAncestorOf(vis1));
  EXPECT_FALSE(vis2->IsAncestorOf(vis2));
  EXPECT_TRUE(vis2->IsAncestorOf(vis3_1));
  EXPECT_TRUE(vis2->IsAncestorOf(vis3_2));
  EXPECT_TRUE(vis2->IsAncestorOf(vis4));

  EXPECT_TRUE(vis2->IsDescendantOf(world));
  EXPECT_TRUE(vis2->IsDescendantOf(vis1));
  EXPECT_FALSE(vis2->IsDescendantOf(vis2));
  EXPECT_FALSE(vis2->IsDescendantOf(vis3_1));
  EXPECT_FALSE(vis2->IsDescendantOf(vis3_2));
  EXPECT_FALSE(vis2->IsDescendantOf(vis4));

  // vis3_1
  EXPECT_FALSE(vis3_1->IsAncestorOf(world));
  EXPECT_FALSE(vis3_1->IsAncestorOf(vis1));
  EXPECT_FALSE(vis3_1->IsAncestorOf(vis2));
  EXPECT_FALSE(vis3_1->IsAncestorOf(vis3_1));
  EXPECT_FALSE(vis3_1->IsAncestorOf(vis3_2));
  EXPECT_TRUE(vis3_1->IsAncestorOf(vis4));

  EXPECT_TRUE(vis3_1->IsDescendantOf(world));
  EXPECT_TRUE(vis3_1->IsDescendantOf(vis1));
  EXPECT_TRUE(vis3_1->IsDescendantOf(vis2));
  EXPECT_FALSE(vis3_1->IsDescendantOf(vis3_1));
  EXPECT_FALSE(vis3_1->IsDescendantOf(vis3_2));
  EXPECT_FALSE(vis3_1->IsDescendantOf(vis4));

  // vis3_2
  EXPECT_FALSE(vis3_2->IsAncestorOf(world));
  EXPECT_FALSE(vis3_2->IsAncestorOf(vis1));
  EXPECT_FALSE(vis3_2->IsAncestorOf(vis2));
  EXPECT_FALSE(vis3_2->IsAncestorOf(vis3_1));
  EXPECT_FALSE(vis3_2->IsAncestorOf(vis3_2));
  EXPECT_FALSE(vis3_2->IsAncestorOf(vis4));

  EXPECT_TRUE(vis3_2->IsDescendantOf(world));
  EXPECT_TRUE(vis3_2->IsDescendantOf(vis1));
  EXPECT_TRUE(vis3_2->IsDescendantOf(vis2));
  EXPECT_FALSE(vis3_2->IsDescendantOf(vis3_1));
  EXPECT_FALSE(vis3_2->IsDescendantOf(vis3_2));
  EXPECT_FALSE(vis3_2->IsDescendantOf(vis4));

  // vis4
  EXPECT_FALSE(vis4->IsAncestorOf(world));
  EXPECT_FALSE(vis4->IsAncestorOf(vis1));
  EXPECT_FALSE(vis4->IsAncestorOf(vis2));
  EXPECT_FALSE(vis4->IsAncestorOf(vis3_1));
  EXPECT_FALSE(vis4->IsAncestorOf(vis3_2));
  EXPECT_FALSE(vis4->IsAncestorOf(vis4));

  EXPECT_TRUE(vis4->IsDescendantOf(world));
  EXPECT_TRUE(vis4->IsDescendantOf(vis1));
  EXPECT_TRUE(vis4->IsDescendantOf(vis2));
  EXPECT_TRUE(vis4->IsDescendantOf(vis3_1));
  EXPECT_FALSE(vis4->IsDescendantOf(vis3_2));
  EXPECT_FALSE(vis4->IsDescendantOf(vis4));

  // null
  EXPECT_FALSE(world->IsAncestorOf(nullptr));
  EXPECT_FALSE(world->IsDescendantOf(nullptr));
  EXPECT_FALSE(vis4->IsAncestorOf(nullptr));
  EXPECT_FALSE(vis4->IsDescendantOf(nullptr));
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, EntityDepths)
{
  this->Load("worlds/shapes.world");

  auto scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 30;
  rendering::VisualPtr box, sphere, cylinder, sun;
  while ((!box || !sphere || !cylinder || !sun) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    sun = scene->GetVisual("sun");

    common::Time::MSleep(100);
    sleep++;
  }

  // World
  auto world = scene->WorldVisual();
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(0u, world->GetDepth());

  // Models
  ASSERT_NE(nullptr, box);
  EXPECT_EQ(1u, box->GetDepth());

  ASSERT_NE(nullptr, cylinder);
  EXPECT_EQ(1u, cylinder->GetDepth());

  ASSERT_NE(nullptr, sphere);
  EXPECT_EQ(1u, sphere->GetDepth());

  // Lights
  ASSERT_NE(nullptr, sun);
  EXPECT_EQ(1u, sun->GetDepth());
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, ConvertVisualType)
{
  // convert from msgs::Visual::Type to Visual::VisualType
  EXPECT_EQ(msgs::Visual::ENTITY,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_ENTITY));
  EXPECT_EQ(msgs::Visual::MODEL,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_MODEL));
  EXPECT_EQ(msgs::Visual::LINK,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_LINK));
  EXPECT_EQ(msgs::Visual::VISUAL,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_VISUAL));
  EXPECT_EQ(msgs::Visual::COLLISION,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_COLLISION));
  EXPECT_EQ(msgs::Visual::SENSOR,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_SENSOR));
  EXPECT_EQ(msgs::Visual::GUI,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_GUI));
  EXPECT_EQ(msgs::Visual::PHYSICS,
      rendering::Visual::ConvertVisualType(rendering::Visual::VT_PHYSICS));

  // convert from Visual::VisualType to msgs::Visual::Type
  EXPECT_EQ(rendering::Visual::VT_ENTITY,
      rendering::Visual::ConvertVisualType(msgs::Visual::ENTITY));
  EXPECT_EQ(rendering::Visual::VT_MODEL,
      rendering::Visual::ConvertVisualType(msgs::Visual::MODEL));
  EXPECT_EQ(rendering::Visual::VT_LINK,
      rendering::Visual::ConvertVisualType(msgs::Visual::LINK));
  EXPECT_EQ(rendering::Visual::VT_VISUAL,
      rendering::Visual::ConvertVisualType(msgs::Visual::VISUAL));
  EXPECT_EQ(rendering::Visual::VT_COLLISION,
      rendering::Visual::ConvertVisualType(msgs::Visual::COLLISION));
  EXPECT_EQ(rendering::Visual::VT_SENSOR,
      rendering::Visual::ConvertVisualType(msgs::Visual::SENSOR));
  EXPECT_EQ(rendering::Visual::VT_GUI,
      rendering::Visual::ConvertVisualType(msgs::Visual::GUI));
  EXPECT_EQ(rendering::Visual::VT_PHYSICS,
      rendering::Visual::ConvertVisualType(msgs::Visual::PHYSICS));
}

TEST_F(Visual_TEST, CollisionZero)
{
  // This test checks that there isn't a segmentation fault when inserting
  // zero collision geometries.
  // Load a world containing 3 simple shapes with collision geometry equals
  // to zero.
  Load("worlds/collision_zero.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder;
  while ((!box || !sphere || !cylinder) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    common::Time::MSleep(1000);
    sleep++;
  }
  scene->ShowCollisions(true);

  // box
  ASSERT_NE(box, nullptr);
  // cylinder
  ASSERT_NE(cylinder, nullptr);
  // sphere
  ASSERT_NE(sphere, nullptr);
}
/////////////////////////////////////////////////
TEST_F(Visual_TEST, Scale)
{
  // Load a world containing 3 simple shapes
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr box, sphere, cylinder;
  while ((!box || !sphere || !cylinder) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    box = scene->GetVisual("box");
    cylinder = scene->GetVisual("cylinder");
    sphere = scene->GetVisual("sphere");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the model, link, and visuals were properly added
  // and verify initial scale

  // box
  ASSERT_NE(box, nullptr);
  rendering::VisualPtr boxLink;
  for (unsigned int i = 0; i < box->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = box->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      boxLink = vis;
  }
  ASSERT_NE(boxLink, nullptr);

  rendering::VisualPtr boxVisual;
  for (unsigned int i = 0; i < boxLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = boxLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      boxVisual = vis;
  }
  ASSERT_NE(boxVisual, nullptr);

  EXPECT_EQ(box->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxLink->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxVisual->Scale(), ignition::math::Vector3d::One);

  EXPECT_EQ(box->DerivedScale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxLink->DerivedScale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxVisual->DerivedScale(), ignition::math::Vector3d::One);

  // sphere
  ASSERT_NE(sphere, nullptr);
  rendering::VisualPtr sphereLink;
  for (unsigned int i = 0; i < sphere->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = sphere->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      sphereLink = vis;
  }
  ASSERT_NE(sphereLink, nullptr);

  rendering::VisualPtr sphereVisual;
  for (unsigned int i = 0; i < sphereLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = sphereLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      sphereVisual = vis;
  }
  ASSERT_NE(sphereVisual, nullptr);

  EXPECT_EQ(sphere->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereLink->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereVisual->Scale(), ignition::math::Vector3d::One);

  EXPECT_EQ(sphere->DerivedScale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereLink->DerivedScale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereVisual->DerivedScale(), ignition::math::Vector3d::One);

  // cylinder
  ASSERT_NE(cylinder, nullptr);
  rendering::VisualPtr cylinderLink;
  for (unsigned int i = 0; i < cylinder->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = cylinder->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      cylinderLink = vis;
  }
  ASSERT_NE(cylinderLink, nullptr);

  rendering::VisualPtr cylinderVisual;
  for (unsigned int i = 0; i < cylinderLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = cylinderLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      cylinderVisual = vis;
  }
  ASSERT_NE(cylinderVisual, nullptr);

  EXPECT_EQ(cylinder->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderLink->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderVisual->Scale(), ignition::math::Vector3d::One);

  EXPECT_EQ(cylinder->DerivedScale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderLink->DerivedScale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderVisual->DerivedScale(), ignition::math::Vector3d::One);

  // update model scale and verify derived scale and geom size
  ignition::math::Vector3d newBoxScale(0.4, 0.5, 0.6);
  box->SetScale(newBoxScale);
  EXPECT_EQ(box->Scale(), newBoxScale);
  EXPECT_EQ(boxLink->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxVisual->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxVisual->GetGeometrySize(), newBoxScale);

  ignition::math::Vector3d newSphereScale(0.3, 0.3, 0.3);
  sphere->SetScale(newSphereScale);
  EXPECT_EQ(sphere->Scale(), newSphereScale);
  EXPECT_EQ(sphereLink->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereVisual->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereVisual->GetGeometrySize(), newSphereScale);

  ignition::math::Vector3d newCylinderScale(0.2, 0.2, 0.5);
  cylinder->SetScale(newCylinderScale);
  EXPECT_EQ(cylinder->Scale(), newCylinderScale);
  EXPECT_EQ(cylinderLink->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderVisual->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderVisual->GetGeometrySize(), newCylinderScale);

  // update link scale and verify derived scale and geom size
  ignition::math::Vector3d newBoxLinkScale(0.2, 0.1, 3);
  boxLink->SetScale(newBoxLinkScale);
  EXPECT_EQ(box->Scale(), newBoxScale);
  EXPECT_EQ(boxLink->Scale(), newBoxLinkScale);
  EXPECT_EQ(boxVisual->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(boxVisual->GetGeometrySize(), newBoxScale * newBoxLinkScale);

  EXPECT_EQ(box->DerivedScale(), newBoxScale);
  EXPECT_EQ(boxLink->DerivedScale(), newBoxScale * newBoxLinkScale);
  EXPECT_EQ(boxVisual->DerivedScale(), newBoxScale * newBoxLinkScale);

  ignition::math::Vector3d newSphereLinkScale(2, 2, 2);
  sphereLink->SetScale(newSphereLinkScale);
  EXPECT_EQ(sphere->Scale(), newSphereScale);
  EXPECT_EQ(sphereLink->Scale(), newSphereLinkScale);
  EXPECT_EQ(sphereVisual->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(sphereVisual->GetGeometrySize(),
      newSphereScale * newSphereLinkScale);

  EXPECT_EQ(sphere->DerivedScale(), newSphereScale);
  EXPECT_EQ(sphereLink->DerivedScale(), newSphereScale * newSphereLinkScale);
  EXPECT_EQ(sphereVisual->DerivedScale(), newSphereScale * newSphereLinkScale);

  ignition::math::Vector3d newCylinderLinkScale(4, 4, 0.5);
  cylinderLink->SetScale(newCylinderLinkScale);
  EXPECT_EQ(cylinder->Scale(), newCylinderScale);
  EXPECT_EQ(cylinderLink->Scale(), newCylinderLinkScale);
  EXPECT_EQ(cylinderVisual->Scale(), ignition::math::Vector3d::One);
  EXPECT_EQ(cylinderVisual->GetGeometrySize(),
      newCylinderScale * newCylinderLinkScale);

  EXPECT_EQ(cylinder->DerivedScale(), newCylinderScale);
  EXPECT_EQ(cylinderLink->DerivedScale(),
      newCylinderScale * newCylinderLinkScale);
  EXPECT_EQ(cylinderVisual->DerivedScale(),
      newCylinderScale * newCylinderLinkScale);

  // update visual scale and verify derived scale and geom size
  ignition::math::Vector3d newBoxVisualScale(1.2, 1, 50);
  boxVisual->SetScale(newBoxVisualScale);
  EXPECT_EQ(box->Scale(), newBoxScale);
  EXPECT_EQ(boxLink->Scale(), newBoxLinkScale);
  EXPECT_EQ(boxVisual->Scale(), newBoxVisualScale);
  EXPECT_EQ(boxVisual->GetGeometrySize(),
      newBoxScale * newBoxLinkScale * newBoxVisualScale);

  EXPECT_EQ(box->DerivedScale(), newBoxScale);
  EXPECT_EQ(boxLink->DerivedScale(), newBoxScale * newBoxLinkScale);
  EXPECT_EQ(boxVisual->DerivedScale(),
      newBoxScale * newBoxLinkScale * newBoxVisualScale);

  ignition::math::Vector3d newSphereVisualScale(0.08, 0.08, 0.08);
  sphereVisual->SetScale(newSphereVisualScale);
  EXPECT_EQ(sphere->Scale(), newSphereScale);
  EXPECT_EQ(sphereLink->Scale(), newSphereLinkScale);
  EXPECT_EQ(sphereVisual->Scale(), newSphereVisualScale);
  EXPECT_EQ(sphereVisual->GetGeometrySize(),
      newSphereScale * newSphereLinkScale * newSphereVisualScale);

  EXPECT_EQ(sphere->DerivedScale(), newSphereScale);
  EXPECT_EQ(sphereLink->DerivedScale(), newSphereScale * newSphereLinkScale);
  EXPECT_EQ(sphereVisual->DerivedScale(),
      newSphereScale * newSphereLinkScale * newSphereVisualScale);

  ignition::math::Vector3d newCylinderVisualScale(3, 3, 0.25);
  cylinderVisual->SetScale(newCylinderVisualScale);
  EXPECT_EQ(cylinder->Scale(), newCylinderScale);
  EXPECT_EQ(cylinderLink->Scale(), newCylinderLinkScale);
  EXPECT_EQ(cylinderVisual->Scale(), newCylinderVisualScale);
  EXPECT_EQ(cylinderVisual->GetGeometrySize(),
      newCylinderScale * newCylinderLinkScale * newCylinderVisualScale);

  EXPECT_EQ(cylinder->DerivedScale(), newCylinderScale);
  EXPECT_EQ(cylinderLink->DerivedScale(),
      newCylinderScale * newCylinderLinkScale);
  EXPECT_EQ(cylinderVisual->DerivedScale(),
      newCylinderScale * newCylinderLinkScale * newCylinderVisualScale);

  // clone visual and verify scale
  rendering::VisualPtr boxClone = box->Clone("boxClone",
      box->GetParent());
  rendering::VisualPtr sphereClone = sphere->Clone("sphereClone",
      sphere->GetParent());
  rendering::VisualPtr cylinderClone = cylinder->Clone("cylinderClone",
      cylinder->GetParent());

  std::queue<std::pair<rendering::VisualPtr, rendering::VisualPtr> >
      cloneVisuals;
  cloneVisuals.push(std::make_pair(box, boxClone));
  cloneVisuals.push(std::make_pair(sphere, sphereClone));
  cloneVisuals.push(std::make_pair(cylinder, cylinderClone));
  while (!cloneVisuals.empty())
  {
    auto visualPair = cloneVisuals.front();
    cloneVisuals.pop();
    EXPECT_EQ(visualPair.first->Scale(), visualPair.second->Scale());
    EXPECT_EQ(visualPair.first->DerivedScale(),
        visualPair.second->DerivedScale());
    EXPECT_EQ(visualPair.first->GetGeometrySize(),
        visualPair.second->GetGeometrySize());
    EXPECT_EQ(visualPair.first->GetChildCount(),
        visualPair.second->GetChildCount());
    for (unsigned int i  = 0; i < visualPair.first->GetChildCount(); ++i)
    {
      cloneVisuals.push(std::make_pair(
          visualPair.first->GetChild(i), visualPair.second->GetChild(i)));
    }
  }
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Clone)
{
  Load("worlds/blank.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Get world visual
  gazebo::rendering::VisualPtr world = scene->WorldVisual();
  ASSERT_NE(world, nullptr);

  // Create a visual as child of the world visual
  gazebo::rendering::VisualPtr vis1;
  vis1.reset(new gazebo::rendering::Visual("vis1", scene->WorldVisual()));
  vis1->Load();

  // Create a visual as child of vis1
  gazebo::rendering::VisualPtr vis2;
  vis2.reset(new gazebo::rendering::Visual("vis1::vis2", vis1));
  vis2->Load();

  // Create a visual as child of vis2
  gazebo::rendering::VisualPtr vis3;
  vis3.reset(new gazebo::rendering::Visual("vis1::vis2::vis3", vis2));
  vis3->Load();

  gazebo::rendering::VisualPtr vis1Clone = vis1->Clone("vis1_clone",
      vis1->GetParent());
  ASSERT_NE(vis1Clone, nullptr);
  EXPECT_EQ(vis1Clone->Name(), "vis1_clone");

  EXPECT_EQ(vis1Clone->GetChildCount(), vis1->GetChildCount());
  EXPECT_EQ(vis1Clone->GetChildCount(), 1u);

  gazebo::rendering::VisualPtr vis2Clone = vis1Clone->GetChild(0);
  ASSERT_NE(vis2Clone, nullptr);
  EXPECT_EQ(vis2Clone->Name(), "vis1_clone::vis2");

  EXPECT_EQ(vis2Clone->GetChildCount(), vis2->GetChildCount());
  EXPECT_EQ(vis2Clone->GetChildCount(), 1u);

  gazebo::rendering::VisualPtr vis3Clone = vis2Clone->GetChild(0);
  ASSERT_NE(vis3Clone, nullptr);
  EXPECT_EQ(vis3Clone->Name(), "vis1_clone::vis2::vis3");

  EXPECT_EQ(vis3Clone->GetChildCount(), vis3->GetChildCount());
  EXPECT_EQ(vis3Clone->GetChildCount(), 0u);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, VisibilityFlags)
{
  // Load a world containing 3 simple shapes
  Load("worlds/shapes.world");

  // Get the scene
  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Wait until all models are inserted
  int sleep = 0;
  int maxSleep = 10;
  rendering::VisualPtr cylinder;
  while ((!cylinder) && sleep < maxSleep)
  {
    event::Events::preRender();
    event::Events::render();
    event::Events::postRender();

    cylinder = scene->GetVisual("cylinder");
    common::Time::MSleep(1000);
    sleep++;
  }

  // Check that the cylinder model, link, and visual were properly added

  // cylinder
  ASSERT_NE(cylinder, nullptr);
  rendering::VisualPtr cylinderLink;
  for (unsigned int i = 0; i < cylinder->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = cylinder->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_LINK)
      cylinderLink = vis;
  }
  ASSERT_NE(cylinderLink, nullptr);

  rendering::VisualPtr cylinderVisual;
  for (unsigned int i = 0; i < cylinderLink->GetChildCount(); ++i)
  {
    rendering::VisualPtr vis = cylinderLink->GetChild(i);
    if (vis->GetType() == rendering::Visual::VT_VISUAL)
      cylinderVisual = vis;
  }
  ASSERT_NE(cylinderVisual, nullptr);

  // set visibility flags - by default they cascade

  // cylinder
  uint32_t newCylinderVisibilityFlags = GZ_VISIBILITY_GUI;
  cylinder->SetVisibilityFlags(newCylinderVisibilityFlags);
  EXPECT_EQ(cylinder->GetVisibilityFlags(), newCylinderVisibilityFlags);

  // cylinder link
  uint32_t newCylinderLinkVisibilityFlags = GZ_VISIBILITY_ALL;
  cylinderLink->SetVisibilityFlags(newCylinderLinkVisibilityFlags);
  EXPECT_EQ(cylinderLink->GetVisibilityFlags(), newCylinderLinkVisibilityFlags);

  // cylinder visual
  uint32_t newCylinderVisualVisibilityFlags = GZ_VISIBILITY_SELECTABLE;
  cylinderVisual->SetVisibilityFlags(newCylinderVisualVisibilityFlags);
  EXPECT_EQ(cylinderVisual->GetVisibilityFlags(),
      newCylinderVisualVisibilityFlags);

  // clone visuals and verify visibility flags
  rendering::VisualPtr cylinderClone = cylinder->Clone("cylinderClone",
      cylinder->GetParent());

  std::queue<std::pair<rendering::VisualPtr, rendering::VisualPtr> >
      cloneVisuals;
  cloneVisuals.push(std::make_pair(cylinder, cylinderClone));
  while (!cloneVisuals.empty())
  {
    auto visualPair = cloneVisuals.front();
    cloneVisuals.pop();
    EXPECT_EQ(visualPair.first->GetVisibilityFlags(),
        visualPair.second->GetVisibilityFlags());
    EXPECT_EQ(visualPair.first->GetChildCount(),
        visualPair.second->GetChildCount());
    for (unsigned int i  = 0; i < visualPair.first->GetChildCount(); ++i)
    {
      cloneVisuals.push(std::make_pair(
          visualPair.first->GetChild(i), visualPair.second->GetChild(i)));
    }
  }
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, DestroyWithPlugin)
{
  this->Load("worlds/empty.world");

  auto scene = gazebo::rendering::get_scene();
  ASSERT_NE(scene, nullptr);

  // Load visual
  std::stringstream visualString;
  visualString
      << "<sdf version='" << SDF_VERSION <<"'>"
      << "  <visual name='visual_with_plugin'>"
      << "    <geometry>"
      << "      <box>"
      << "        <size>1 1 1</size>"
      << "      </box>"
      << "    </geometry>"
      << "    <plugin name='blink_plugin' filename='libBlinkVisualPlugin.so'/>"
      << "  </visual>"
      << "</sdf>";

  sdf::ElementPtr sdf(new sdf::Element);
  sdf::initFile("visual.sdf", sdf);
  sdf::readString(visualString.str(), sdf);

  gazebo::rendering::VisualPtr vis(
      new gazebo::rendering::Visual("box_visual", scene));
  vis->Load(sdf);

  ASSERT_NE(vis, nullptr);

  // Call Fini multiple times, reset and check there's no crash
  vis->Fini();
  vis->Fini();
  vis->Fini();
  vis.reset();

  ASSERT_EQ(vis, nullptr);
}

/////////////////////////////////////////////////
TEST_F(Visual_TEST, Pose)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  ASSERT_TRUE(scene != nullptr);

  // parent box vis
  ignition::math::Pose3d boxPose(0.0, 1.0, 2.0, 0.0, 0.0, 0.0);
  sdf::ElementPtr boxSDF(new sdf::Element);
  sdf::initFile("visual.sdf", boxSDF);
  sdf::readString(GetVisualSDFString("visual_box", "box",
      ignition::math::Vector3d::One, boxPose), boxSDF);
  gazebo::rendering::VisualPtr boxVis(
      new gazebo::rendering::Visual("box_visual", scene));
  boxVis->Load(boxSDF);
  EXPECT_EQ(boxVis->Pose(), boxPose);
  EXPECT_EQ(boxVis->WorldPose(), boxPose);
  EXPECT_EQ(boxVis->InitialRelativePose(), boxPose);

  // child sphere vis
  ignition::math::Pose3d spherePose(0.0, 5.0, 5.0, 1.57, 0.0, 0.0);
  sdf::ElementPtr sphereSDF(new sdf::Element);
  sdf::initFile("visual.sdf", sphereSDF);
  sdf::readString(GetVisualSDFString("visual_sphere", "sphere",
      ignition::math::Vector3d::One, spherePose), sphereSDF);
  gazebo::rendering::VisualPtr sphereVis(
      new gazebo::rendering::Visual("sphere_visual", boxVis));
  sphereVis->Load(sphereSDF);
  EXPECT_EQ(sphereVis->Pose(), spherePose);
  EXPECT_EQ(sphereVis->WorldPose(), spherePose + boxPose);
  EXPECT_EQ(sphereVis->InitialRelativePose(), spherePose);

  // set new sphere pose and verify
  ignition::math::Pose3d newSpherePose(1.0, 20.0, 0.0, 0.0, 0.0, 1.57);
  sphereVis->SetPose(newSpherePose);
  EXPECT_EQ(sphereVis->Pose(), newSpherePose);
  EXPECT_EQ(sphereVis->WorldPose(), newSpherePose + boxPose);
  EXPECT_EQ(sphereVis->InitialRelativePose(), spherePose);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
