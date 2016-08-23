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
#include <ignition/msgs/Utility.hh>

#include "gazebo/util/IgnMsgSdf.hh"
#include "test/util.hh"

using namespace gazebo;

class IgnMsgSdfTest : public gazebo::testing::AutoLogFixture {};

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, ConvertNotSupportedType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("plugin.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <plugin name='plugin_name' filename='plugin_filename'>\
         </plugin>\
       </sdf>", sdf));

  auto msg = util::Convert<ignition::msgs::WorldControl>(sdf);
  EXPECT_FALSE(msg.has_pause());
  EXPECT_FALSE(msg.has_step());
  EXPECT_FALSE(msg.has_multi_step());
  EXPECT_FALSE(msg.has_reset());
  EXPECT_FALSE(msg.has_seed());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, ConvertWrongType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <gui fullscreen='true'>\
         </gui>\
       </sdf>", sdf));

  // Error message is printed
  auto msg = util::Convert<ignition::msgs::Plugin>(sdf);
  EXPECT_FALSE(msg.has_name());
  EXPECT_FALSE(msg.has_filename());
  EXPECT_FALSE(msg.has_innerxml());
  EXPECT_EQ(msg.DebugString(), "");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, PluginSdfToIgnMsg)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("plugin.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <plugin name='plugin_name' filename='plugin_filename'>\
           <param1>1</param1>\
           <param2>true</param2>\
         </plugin>\
       </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Plugin>(sdf);

  EXPECT_TRUE(msg.has_name());
  EXPECT_EQ(msg.name(), "plugin_name");
  EXPECT_TRUE(msg.has_filename());
  EXPECT_EQ(msg.filename(), "plugin_filename");
  EXPECT_TRUE(msg.has_innerxml());
  EXPECT_EQ(msg.innerxml(), "<param1>1</param1>\n<param2>true</param2>\n");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, PluginIgnMsgToSdf)
{
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<plugin_param>param</plugin_param>";

  ignition::msgs::Plugin msg;
  msg.set_name(name);
  msg.set_filename(filename);
  msg.set_innerxml(innerxml);

  auto pluginSDF = util::Convert(msg);

  EXPECT_TRUE(pluginSDF->HasAttribute("name"));
  EXPECT_EQ(pluginSDF->Get<std::string>("name"), name);

  EXPECT_TRUE(pluginSDF->HasAttribute("filename"));
  EXPECT_EQ(pluginSDF->Get<std::string>("filename"), filename);

  EXPECT_TRUE(pluginSDF->HasElement("plugin_param"));
  EXPECT_EQ(pluginSDF->Get<std::string>("plugin_param"), "param");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, PluginToFromSDF)
{
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<plugin_param>param</plugin_param>\n";

  // Create message
  ignition::msgs::Plugin msg1;
  msg1.set_name(name);
  msg1.set_filename(filename);
  msg1.set_innerxml(innerxml);

  // To SDF
  auto sdf1 = util::Convert(msg1);

  // Back to Msg
  auto msg2 = util::Convert<ignition::msgs::Plugin>(sdf1);
  EXPECT_EQ(msg2.name(), name);
  EXPECT_EQ(msg2.filename(), filename);
  EXPECT_EQ(msg2.innerxml(), innerxml);

  // Back to SDF
  sdf::ElementPtr sdf2;
  sdf2.reset(new sdf::Element);
  sdf::initFile("plugin.sdf", sdf2);
  util::Convert(msg2, sdf2);
  EXPECT_TRUE(sdf2 != NULL);
  EXPECT_EQ(sdf2->Get<std::string>("name"), name);
  EXPECT_EQ(sdf2->Get<std::string>("filename"), filename);
  EXPECT_TRUE(sdf2->HasElement("plugin_param"));
  EXPECT_EQ(sdf2->Get<std::string>("plugin_param"), "param");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_PlaneVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <plane><normal>0 0 1</normal></plane>\
           </geometry>\
           <material><script>Gazebo/Grey</script></material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_BoxVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <box><size>1 1 1</size></box>\
           </geometry>\
           <material><script>Gazebo/Grey'</script></material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_SphereVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <sphere><radius>1</radius></sphere>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='normal_map_tangent_space'>\
             <normal_map>test.map</normal_map>\
           </shader>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_CylinderVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <cylinder><radius>1</radius><length>1.0</length></cylinder>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='normal_map_object_space'/>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_MeshVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <mesh><scale>1 1 1</scale><uri>test1.mesh</uri></mesh>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='vertex'/>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_ImageVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
             <image>\
               <scale>1</scale>\
               <height>1</height>\
               <threshold>255</threshold>\
               <granularity>10</granularity>\
               <uri>test2.mesh</uri>\
             </image>\
           </geometry>\
           <material>\
             <script>Gazebo/Grey</script>\
             <shader type='pixel'/>\
             <ambient>.1 .2 .3 1</ambient>\
             <diffuse>.1 .2 .3 1</diffuse>\
             <specular>.1 .2 .3 1</specular>\
             <emissive>.1 .2 .3 1</emissive>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_HeigthmapVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
             <heightmap>\
               <size>1 2 3</size>\
               <uri>test3.mesh</uri>\
               <pos>0 0 1</pos>\
             </heightmap>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='pixel'/>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_NoGeometry)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
         </visual>\
      </sdf>", sdf));

  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
  EXPECT_FALSE(msg.geometry().has_type());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_ShaderTypeThrow)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
             <heightmap>\
               <size>1 2 3</size>\
               <uri>test4.mesh</uri>\
               <pos>0 0 0</pos>\
             </heightmap>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
             <shader type='unknown'/>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
  EXPECT_EQ(msg.material().shader_type(), ignition::msgs::Material::VERTEX);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_BadGeometryVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
             <shader type='pixel'/>\
           </material>\
         </visual>\
      </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
  EXPECT_FALSE(msg.geometry().has_type());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualFromSDF_BadGeometryType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  // As of sdformat pull request 148 (released in version 2.3.1),
  // unknown elements are now ignored with a warning message
  // rather than causing an error.
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
         </visual>\
      </sdf>", sdf));

  sdf::ElementPtr badElement(new sdf::Element());
  badElement->SetName("bad_type");
  sdf->GetElement("geometry")->InsertElement(badElement);

  auto msg = util::Convert<ignition::msgs::Visual>(sdf);
  EXPECT_FALSE(msg.has_material());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, VisualToSDF)
{
  const std::string name("visual");
  const double laserRetro = 0.2;
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(1, 2, 3),
      ignition::math::Quaterniond(0, 0, 1, 0));
  const double radius = 3.3;
  const std::string materialName("Gazebo/Grey");
  const std::string uri("pretend_this_is_a_URI");

  ignition::msgs::Visual visualMsg;
  visualMsg.set_name(name);
  visualMsg.set_laser_retro(laserRetro);
  ignition::msgs::Set(visualMsg.mutable_pose(), pose);

  // geometry - see GeometryToSDF for a more detailed test
  auto geomMsg = visualMsg.mutable_geometry();
  geomMsg->set_type(ignition::msgs::Geometry::SPHERE);
  geomMsg->mutable_sphere()->set_radius(radius);

  // material - see MaterialToSDF for a more detailed test
  auto scriptMsg = visualMsg.mutable_material()->mutable_script();
  scriptMsg->set_name(materialName);
  scriptMsg->add_uri();
  scriptMsg->set_uri(0, uri);

  auto visualSDF = util::Convert(visualMsg);

  EXPECT_TRUE(visualSDF->HasAttribute("name"));
  EXPECT_EQ(name, visualSDF->Get<std::string>("name"));

  EXPECT_DOUBLE_EQ(visualSDF->Get<double>("laser_retro"), laserRetro);

  EXPECT_EQ(pose, visualSDF->Get<ignition::math::Pose3d>("pose"));

  ASSERT_TRUE(visualSDF->HasElement("geometry"));
  auto geomElem = visualSDF->GetElement("geometry");
  EXPECT_TRUE(geomElem->HasElement("sphere"));
  auto sphereElem = geomElem->GetElement("sphere");
  EXPECT_TRUE(sphereElem->HasElement("radius"));
  EXPECT_DOUBLE_EQ(sphereElem->Get<double>("radius"), radius);

  ASSERT_TRUE(visualSDF->HasElement("material"));
  auto materialElem = visualSDF->GetElement("material");
  EXPECT_TRUE(materialElem->HasElement("script"));
  auto scriptElem = materialElem->GetElement("script");
  EXPECT_TRUE(scriptElem->HasElement("name"));
  EXPECT_EQ(materialName, scriptElem->Get<std::string>("name"));
  EXPECT_TRUE(scriptElem->HasElement("uri"));
  EXPECT_EQ(uri, scriptElem->Get<std::string>("uri"));

  // Test the meta.layer property
  {
    ignition::msgs::Visual msg;
    auto metaMsg = msg.mutable_meta();
    metaMsg->set_layer(1);

    auto visSdf = util::Convert(msg);
    EXPECT_TRUE(visSdf->HasElement("meta"));
    EXPECT_TRUE(visSdf->GetElement("meta")->HasElement("layer"));
    EXPECT_EQ(visSdf->GetElement("meta")->Get<int32_t>("layer"), 1);

    auto msg2 = util::Convert<ignition::msgs::Visual>(visSdf);
    EXPECT_TRUE(msg2.has_meta());
    EXPECT_TRUE(msg2.meta().has_layer());
    EXPECT_EQ(msg2.meta().layer(), 1);
  }
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, GeometryToSDF)
{
  // box
  const ignition::math::Vector3d boxSize(0.5, 0.75, 1.0);
  ignition::msgs::Geometry boxMsg;
  boxMsg.set_type(ignition::msgs::Geometry::BOX);
  auto boxGeom = boxMsg.mutable_box();
  ignition::msgs::Set(boxGeom->mutable_size(), boxSize);

  auto boxSDF = util::Convert(boxMsg);
  auto boxElem = boxSDF->GetElement("box");
  EXPECT_EQ(boxElem->Get<ignition::math::Vector3d>("size"), boxSize);

  // cylinder
  ignition::msgs::Geometry cylinderMsg;
  cylinderMsg.set_type(ignition::msgs::Geometry::CYLINDER);
  auto cylinderGeom = cylinderMsg.mutable_cylinder();
  cylinderGeom->set_radius(0.3);
  cylinderGeom->set_length(1.0);

  auto cylinderSDF = util::Convert(cylinderMsg);
  auto cylinderElem = cylinderSDF->GetElement("cylinder");
  EXPECT_DOUBLE_EQ(cylinderElem->Get<double>("radius"), 0.3);
  EXPECT_DOUBLE_EQ(cylinderElem->Get<double>("length"), 1.0);

  // sphere
  ignition::msgs::Geometry sphereMsg;
  sphereMsg.set_type(ignition::msgs::Geometry::SPHERE);
  auto sphereGeom = sphereMsg.mutable_sphere();
  sphereGeom->set_radius(3.0);

  auto sphereSDF = util::Convert(sphereMsg);
  auto sphereElem = sphereSDF->GetElement("sphere");
  EXPECT_DOUBLE_EQ(sphereElem->Get<double>("radius"), 3.0);

  // plane
  ignition::msgs::Geometry planeMsg;
  planeMsg.set_type(ignition::msgs::Geometry::PLANE);
  auto planeGeom = planeMsg.mutable_plane();
  ignition::msgs::Set(planeGeom->mutable_normal(),
      ignition::math::Vector3d(0, 0, 1.0));
  ignition::msgs::Set(planeGeom->mutable_size(),
      ignition::math::Vector2d(0.5, 0.8));

  auto planeSDF = util::Convert(planeMsg);
  auto planeElem = planeSDF->GetElement("plane");
  EXPECT_TRUE(planeElem->Get<ignition::math::Vector3d>("normal") ==
      ignition::math::Vector3d(0, 0, 1.0));
  EXPECT_TRUE(planeElem->Get<ignition::math::Vector2d>("size") ==
      ignition::math::Vector2d(0.5, 0.8));

  // image
  ignition::msgs::Geometry imageMsg;
  imageMsg.set_type(ignition::msgs::Geometry::IMAGE);
  auto imageGeom = imageMsg.mutable_image();
  imageGeom->set_uri("test_uri");
  imageGeom->set_scale(1.8);
  imageGeom->set_threshold(255);
  imageGeom->set_height(1.3);
  imageGeom->set_granularity(2);

  auto imageSDF = util::Convert(imageMsg);
  auto imageElem = imageSDF->GetElement("image");
  EXPECT_STREQ(imageElem->Get<std::string>("uri").c_str(), "test_uri");
  EXPECT_DOUBLE_EQ(imageElem->Get<double>("scale"), 1.8);
  EXPECT_DOUBLE_EQ(imageElem->Get<double>("threshold"), 255);
  EXPECT_DOUBLE_EQ(imageElem->Get<double>("height"), 1.3);
  EXPECT_DOUBLE_EQ(imageElem->Get<int>("granularity"), 2);

  // heightmap
  ignition::msgs::Geometry heightmapMsg;
  heightmapMsg.set_type(ignition::msgs::Geometry::HEIGHTMAP);
  auto heightmapGeom = heightmapMsg.mutable_heightmap();
  heightmapGeom->set_filename("test_heightmap_filename");
  ignition::msgs::Set(heightmapGeom->mutable_size(),
      ignition::math::Vector3d(100, 200, 30));
  ignition::msgs::Set(heightmapGeom->mutable_origin(),
      ignition::math::Vector3d(50, 100, 15));
  heightmapGeom->set_use_terrain_paging(true);

  auto texture1 = heightmapGeom->add_texture();
  texture1->set_diffuse("test_diffuse1");
  texture1->set_normal("test_normal1");
  texture1->set_size(10);

  auto texture2 = heightmapGeom->add_texture();
  texture2->set_diffuse("test_diffuse2");
  texture2->set_normal("test_normal2");
  texture2->set_size(20);

  auto blend = heightmapGeom->add_blend();
  blend->set_min_height(25);
  blend->set_fade_dist(5);

  auto heightmapSDF = util::Convert(heightmapMsg);
  auto heightmapElem = heightmapSDF->GetElement("heightmap");
  EXPECT_STREQ(heightmapElem->Get<std::string>("uri").c_str(),
      "test_heightmap_filename");
  EXPECT_TRUE(heightmapElem->Get<ignition::math::Vector3d>("size") ==
      ignition::math::Vector3d(100, 200, 30));
  EXPECT_TRUE(heightmapElem->Get<ignition::math::Vector3d>("pos") ==
      ignition::math::Vector3d(50, 100, 15));
  EXPECT_TRUE(heightmapElem->Get<bool>("use_terrain_paging"));

  auto textureElem1 = heightmapElem->GetElement("texture");
  EXPECT_STREQ(textureElem1->Get<std::string>("diffuse").c_str(),
      "test_diffuse1");
  EXPECT_STREQ(textureElem1->Get<std::string>("normal").c_str(),
      "test_normal1");
  EXPECT_DOUBLE_EQ(textureElem1->Get<double>("size"), 10);
  auto textureElem2 = textureElem1->GetNextElement("texture");
  EXPECT_STREQ(textureElem2->Get<std::string>("diffuse").c_str(),
      "test_diffuse2");
  EXPECT_STREQ(textureElem2->Get<std::string>("normal").c_str(),
      "test_normal2");
  EXPECT_DOUBLE_EQ(textureElem2->Get<double>("size"), 20);

  auto blendElem = heightmapElem->GetElement("blend");
  EXPECT_DOUBLE_EQ(blendElem->Get<double>("min_height"), 25);
  EXPECT_DOUBLE_EQ(blendElem->Get<double>("fade_dist"), 5);

  // mesh
  ignition::msgs::Geometry meshMsg;
  meshMsg.set_type(ignition::msgs::Geometry::MESH);
  auto meshGeom = meshMsg.mutable_mesh();
  meshGeom->set_filename("test_mesh_filename");
  ignition::msgs::Set(meshGeom->mutable_scale(),
      ignition::math::Vector3d(2.3, 1.2, 2.9));
  meshGeom->set_submesh("test_mesh_submesh");
  meshGeom->set_center_submesh(false);

  auto meshSDF = util::Convert(meshMsg);
  auto meshElem = meshSDF->GetElement("mesh");
  EXPECT_STREQ(meshElem->Get<std::string>("uri").c_str(),
      "test_mesh_filename");
  EXPECT_TRUE(meshElem->Get<ignition::math::Vector3d>("scale") ==
      ignition::math::Vector3d(2.3, 1.2, 2.9));
  auto submeshElem = meshElem->GetElement("submesh");
  EXPECT_STREQ(submeshElem->Get<std::string>("name").c_str(),
      "test_mesh_submesh");
  EXPECT_TRUE(!submeshElem->Get<bool>("center"));

  // polyline
  ignition::msgs::Geometry polylineMsg;
  polylineMsg.set_type(ignition::msgs::Geometry::POLYLINE);
  ignition::msgs::Polyline *polylineGeom = polylineMsg.add_polyline();
  polylineGeom->set_height(2.33);
  const ignition::math::Vector2d point1(0.5, 0.7);
  const ignition::math::Vector2d point2(3.5, 4.7);
  const ignition::math::Vector2d point3(1000, 2000);
  ignition::msgs::Set(polylineGeom->add_point(), point1);
  ignition::msgs::Set(polylineGeom->add_point(), point2);
  ignition::msgs::Set(polylineGeom->add_point(), point3);

  auto polylineSDF = util::Convert(polylineMsg);
  auto polylineElem = polylineSDF->GetElement("polyline");
  EXPECT_DOUBLE_EQ(polylineElem->Get<double>("height"), 2.33);

  auto pointElem1 = polylineElem->GetElement("point");
  auto pointElem2 = pointElem1->GetNextElement("point");
  auto pointElem3 = pointElem2->GetNextElement("point");
  EXPECT_EQ(pointElem1->Get<ignition::math::Vector2d>(), point1);
  EXPECT_EQ(pointElem2->Get<ignition::math::Vector2d>(), point2);
  EXPECT_EQ(pointElem3->Get<ignition::math::Vector2d>(), point3);
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, MeshToSDF)
{
  const ignition::math::Vector3d meshScale(0.1, 0.2, 0.3);
  ignition::msgs::MeshGeom msg;
  msg.set_filename("test_filename");
  ignition::msgs::Set(msg.mutable_scale(), meshScale);
  msg.set_submesh("test_submesh");
  msg.set_center_submesh(true);

  auto meshSDF = util::Convert(msg);

  EXPECT_STREQ(meshSDF->Get<std::string>("uri").c_str(), "test_filename");
  EXPECT_TRUE(meshSDF->HasElement("scale"));
  ignition::math::Vector3d scale =
      meshSDF->Get<ignition::math::Vector3d>("scale");
  EXPECT_DOUBLE_EQ(scale.X(), meshScale.X());
  EXPECT_DOUBLE_EQ(scale.Y(), meshScale.Y());
  EXPECT_DOUBLE_EQ(scale.Z(), meshScale.Z());

  auto submeshElem = meshSDF->GetElement("submesh");
  EXPECT_STREQ(submeshElem->Get<std::string>("name").c_str(), "test_submesh");
  EXPECT_TRUE(submeshElem->Get<bool>("center"));

  // no submesh
  const ignition::math::Vector3d meshScale2(1, 2, 3);
  ignition::msgs::MeshGeom msg2;
  msg2.set_filename("test_filename2");
  ignition::msgs::Set(msg2.mutable_scale(), meshScale2);

  auto meshSDF2 = util::Convert(msg2);

  EXPECT_STREQ(meshSDF2->Get<std::string>("uri").c_str(), "test_filename2");
  EXPECT_TRUE(meshSDF2->HasElement("scale"));
  ignition::math::Vector3d scale2 =
      meshSDF2->Get<ignition::math::Vector3d>("scale");
  EXPECT_DOUBLE_EQ(scale2.X(), meshScale2.X());
  EXPECT_DOUBLE_EQ(scale2.Y(), meshScale2.Y());
  EXPECT_DOUBLE_EQ(scale2.Z(), meshScale2.Z());

  EXPECT_FALSE(meshSDF2->HasElement("submesh"));
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, MeshFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("geometry.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
           <geometry>\
             <mesh>\
               <uri>test/mesh.dae</uri>\
               <scale>1 2 3</scale>\
               <submesh>\
                 <name>test_name</name>\
                 <center>true</center>\
               </submesh>\
             </mesh>\
           </geometry>\
      </sdf>", sdf));

  auto msg = util::Convert<ignition::msgs::MeshGeom>(sdf->GetElement("mesh"));
  EXPECT_TRUE(msg.has_filename());
  EXPECT_STREQ("test/mesh.dae", msg.filename().c_str());

  EXPECT_TRUE(msg.has_scale());
  EXPECT_DOUBLE_EQ(msg.scale().x(), 1.0);
  EXPECT_DOUBLE_EQ(msg.scale().y(), 2.0);
  EXPECT_DOUBLE_EQ(msg.scale().z(), 3.0);

  EXPECT_TRUE(msg.has_submesh());
  EXPECT_STREQ("test_name", msg.submesh().c_str());

  EXPECT_TRUE(msg.has_center_submesh());
  EXPECT_TRUE(msg.center_submesh());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, MaterialToSDF)
{
  ignition::msgs::Material msg;

  const std::string name("Gazebo/Grey");
  const std::string uri("file://media/materials/scripts/gazebo.material");
  const auto type = ignition::msgs::Material::VERTEX;
  const std::string normalMap("normalMap");
  const bool lighting = true;
  const common::Color ambient(.1, .2, .3, 1.0);
  const common::Color diffuse(.4, .5, .6, 1.0);
  const common::Color emissive(.5, .5, .5, 0.5);
  const common::Color specular(.7, .8, .9, 1.0);

  msg.mutable_script()->set_name(name);
  msg.mutable_script()->add_uri();
  msg.mutable_script()->set_uri(0, uri);
  msg.set_shader_type(type);
  msg.set_normal_map(normalMap);
  msg.set_lighting(lighting);
  msg.mutable_ambient()->CopyFrom(util::Convert(ambient));
  msg.mutable_diffuse()->CopyFrom(util::Convert(diffuse));
  msg.mutable_emissive()->CopyFrom(util::Convert(emissive));
  msg.mutable_specular()->CopyFrom(util::Convert(specular));

  auto materialSDF = util::Convert(msg);

  {
    ASSERT_TRUE(materialSDF->HasElement("script"));
    auto scriptElem = materialSDF->GetElement("script");
    EXPECT_TRUE(scriptElem->HasElement("name"));
    EXPECT_EQ(name, scriptElem->Get<std::string>("name"));
    EXPECT_TRUE(scriptElem->HasElement("uri"));
    EXPECT_EQ(uri, scriptElem->Get<std::string>("uri"));
  }

  {
    ASSERT_TRUE(materialSDF->HasElement("shader"));
    auto shaderElem = materialSDF->GetElement("shader");
    EXPECT_TRUE(shaderElem->HasAttribute("type"));
    EXPECT_EQ(util::Convert(type), shaderElem->Get<std::string>("type"));
    EXPECT_TRUE(shaderElem->HasElement("normal_map"));
    EXPECT_EQ(normalMap, shaderElem->Get<std::string>("normal_map"));
  }

  EXPECT_TRUE(materialSDF->HasElement("ambient"));
  EXPECT_EQ(ambient, materialSDF->Get<common::Color>("ambient"));
  EXPECT_TRUE(materialSDF->HasElement("diffuse"));
  EXPECT_EQ(diffuse, materialSDF->Get<common::Color>("diffuse"));
  EXPECT_TRUE(materialSDF->HasElement("emissive"));
  EXPECT_EQ(emissive, materialSDF->Get<common::Color>("emissive"));
  EXPECT_TRUE(materialSDF->HasElement("specular"));
  EXPECT_EQ(specular, materialSDF->Get<common::Color>("specular"));
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, ColorFromCommon)
{
  auto msg = util::Convert(gazebo::common::Color(.1, .2, .3, 1.0));

  EXPECT_FLOAT_EQ(0.1f, msg.r());
  EXPECT_FLOAT_EQ(0.2f, msg.g());
  EXPECT_FLOAT_EQ(0.3f, msg.b());
  EXPECT_FLOAT_EQ(1.0f, msg.a());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, ColorToCommon)
{
  auto msg = util::Convert(gazebo::common::Color(.1, .2, .3, 1.0));
  auto v = util::Convert(msg);

  EXPECT_FLOAT_EQ(0.1f, v.r);
  EXPECT_FLOAT_EQ(0.2f, v.g);
  EXPECT_FLOAT_EQ(0.3f, v.b);
  EXPECT_FLOAT_EQ(1.0f, v.a);
}

