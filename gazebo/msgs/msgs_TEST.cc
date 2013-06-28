/*
 * Copyright 2013 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Exception.hh"

using namespace gazebo;

TEST(MsgsTest, Msg)
{
  common::Time t = common::Time::GetWallTime();

  msgs::Test msg, msg2;
  msgs::Init(msg, "_test_");
  msgs::Init(msg2);

  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_TRUE(msg2.header().has_stamp());

  EXPECT_EQ(t.sec, msg.header().stamp().sec());
  EXPECT_TRUE(t.nsec <= msg.header().stamp().nsec());
  EXPECT_STREQ("_test_", msg.header().str_id().c_str());

  EXPECT_FALSE(msg2.header().has_str_id());

  msgs::Header *header = msgs::GetHeader(msg);
  EXPECT_STREQ("_test_", header->str_id().c_str());

  msgs::Header testHeader;
  testHeader.set_str_id("_hello_");
  header = msgs::GetHeader(testHeader);
  EXPECT_STREQ("_hello_", header->str_id().c_str());
}

TEST(MsgsTest, Request)
{
  msgs::Request *request = msgs::CreateRequest("help", "me");
  EXPECT_STREQ("help", request->request().c_str());
  EXPECT_STREQ("me", request->data().c_str());
  EXPECT_GT(request->id(), 0);
}

TEST(MsgsTest, Time)
{
  common::Time t = common::Time::GetWallTime();
  msgs::Time msg;
  msgs::Stamp(&msg);
  EXPECT_EQ(t.sec, msg.sec());
  EXPECT_TRUE(t.nsec <= msg.nsec());
}


TEST(MsgsTest, TimeFromHeader)
{
  common::Time t = common::Time::GetWallTime();
  msgs::Header msg;
  msgs::Stamp(&msg);
  EXPECT_EQ(t.sec, msg.stamp().sec());
  EXPECT_TRUE(t.nsec <= msg.stamp().nsec());
}


TEST(MsgsTest, Packet)
{
  msgs::GzString msg;
  msg.set_data("test_string");
  std::string data = msgs::Package("test_type", msg);

  msgs::Packet packet;
  packet.ParseFromString(data);
  msg.ParseFromString(packet.serialized_data());

  EXPECT_STREQ("test_type", packet.type().c_str());
  EXPECT_STREQ("test_string", msg.data().c_str());
}

TEST(MsgsTest, BadPackage)
{
  msgs::GzString msg;
  EXPECT_THROW(msgs::Package("test_type", msg), common::Exception);
}

TEST(MsgsTest, CovertMathVector3ToMsgs)
{
  msgs::Vector3d msg = msgs::Convert(math::Vector3(1, 2, 3));
  EXPECT_DOUBLE_EQ(1, msg.x());
  EXPECT_DOUBLE_EQ(2, msg.y());
  EXPECT_DOUBLE_EQ(3, msg.z());
}

TEST(MsgsTest, ConvertMsgsVector3dToMath)
{
  msgs::Vector3d msg = msgs::Convert(math::Vector3(1, 2, 3));
  math::Vector3 v    = msgs::Convert(msg);
  EXPECT_DOUBLE_EQ(1, v.x);
  EXPECT_DOUBLE_EQ(2, v.y);
  EXPECT_DOUBLE_EQ(3, v.z);
}

TEST(MsgsTest, ConvertMathQuaterionToMsgs)
{
  msgs::Quaternion msg =
    msgs::Convert(math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));

  EXPECT_TRUE(math::equal(msg.x(), -0.65328148243818818));
  EXPECT_TRUE(math::equal(msg.y(), 0.27059805007309856));
  EXPECT_TRUE(math::equal(msg.z(), 0.65328148243818829));
  EXPECT_TRUE(math::equal(msg.w(), 0.27059805007309851));
}

TEST(MsgsTest, ConvertMsgsQuaterionToMath)
{
  msgs::Quaternion msg =
    msgs::Convert(math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));
  math::Quaternion v = msgs::Convert(msg);

  // TODO: to real unit test move math::equal to EXPECT_DOUBLE_EQ
  EXPECT_TRUE(math::equal(v.x, -0.65328148243818818));
  EXPECT_TRUE(math::equal(v.y, 0.27059805007309856));
  EXPECT_TRUE(math::equal(v.z, 0.65328148243818829));
  EXPECT_TRUE(math::equal(v.w, 0.27059805007309851));
}

TEST(MsgsTest, ConvertPoseMathToMsgs)
{
  msgs::Pose msg = msgs::Convert(math::Pose(math::Vector3(1, 2, 3),
        math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI)));

  EXPECT_DOUBLE_EQ(1, msg.position().x());
  EXPECT_DOUBLE_EQ(2, msg.position().y());
  EXPECT_DOUBLE_EQ(3, msg.position().z());

  EXPECT_TRUE(math::equal(msg.orientation().x(), -0.65328148243818818));
  EXPECT_TRUE(math::equal(msg.orientation().y(), 0.27059805007309856));
  EXPECT_TRUE(math::equal(msg.orientation().z(), 0.65328148243818829));
  EXPECT_TRUE(math::equal(msg.orientation().w(), 0.27059805007309851));
}

TEST(MsgsTest, ConvertMsgPoseToMath)
{
  msgs::Pose msg = msgs::Convert(math::Pose(math::Vector3(1, 2, 3),
        math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI)));
  math::Pose v = msgs::Convert(msg);

  EXPECT_DOUBLE_EQ(1, v.pos.x);
  EXPECT_DOUBLE_EQ(2, v.pos.y);
  EXPECT_DOUBLE_EQ(3, v.pos.z);
  EXPECT_TRUE(math::equal(v.rot.x, -0.65328148243818818));
  EXPECT_TRUE(math::equal(v.rot.y, 0.27059805007309856));
  EXPECT_TRUE(math::equal(v.rot.z, 0.65328148243818829));
  EXPECT_TRUE(math::equal(v.rot.w, 0.27059805007309851));
}

TEST(MsgsTest, ConvertCommonColorToMsgs)
{
  msgs::Color msg = msgs::Convert(common::Color(.1, .2, .3, 1.0));

  EXPECT_TRUE(math::equal(0.1f, msg.r()));
  EXPECT_TRUE(math::equal(0.2f, msg.g()));
  EXPECT_TRUE(math::equal(0.3f, msg.b()));
  EXPECT_TRUE(math::equal(1.0f, msg.a()));
}

TEST(MsgsTest, ConvertMsgsColorToCommon)
{
  msgs::Color msg = msgs::Convert(common::Color(.1, .2, .3, 1.0));
  common::Color v = msgs::Convert(msg);

  EXPECT_TRUE(math::equal(0.1f, v.r));
  EXPECT_TRUE(math::equal(0.2f, v.g));
  EXPECT_TRUE(math::equal(0.3f, v.b));
  EXPECT_TRUE(math::equal(1.0f, v.a));
}

TEST(MsgsTest, ConvertCommonTimeToMsgs)
{
  msgs::Time msg = msgs::Convert(common::Time(2, 123));
  EXPECT_EQ(2, msg.sec());
  EXPECT_EQ(123, msg.nsec());

  common::Time v = msgs::Convert(msg);
  EXPECT_EQ(2, v.sec);
  EXPECT_EQ(123, v.nsec);
}

TEST(MsgsTest, ConvertMathPlaneToMsgs)
{
  msgs::PlaneGeom msg = msgs::Convert(math::Plane(math::Vector3(0, 0, 1),
        math::Vector2d(123, 456), 1.0));

  EXPECT_DOUBLE_EQ(0, msg.normal().x());
  EXPECT_DOUBLE_EQ(0, msg.normal().y());
  EXPECT_DOUBLE_EQ(1, msg.normal().z());

  EXPECT_DOUBLE_EQ(123, msg.size().x());
  EXPECT_DOUBLE_EQ(456, msg.size().y());
}

TEST(MsgsTest, ConvertMsgsPlaneToMath)
{
  msgs::PlaneGeom msg = msgs::Convert(math::Plane(math::Vector3(0, 0, 1),
        math::Vector2d(123, 456), 1.0));
  math::Plane v = msgs::Convert(msg);

  EXPECT_DOUBLE_EQ(0, v.normal.x);
  EXPECT_DOUBLE_EQ(0, v.normal.y);
  EXPECT_DOUBLE_EQ(1, v.normal.z);

  EXPECT_DOUBLE_EQ(123, v.size.x);
  EXPECT_DOUBLE_EQ(456, v.size.y);

  EXPECT_TRUE(math::equal(1.0, v.d));
}

TEST(MsgsTest, SetVector3)
{
  msgs::Vector3d msg;
  msgs::Set(&msg, math::Vector3(1, 2, 3));
  EXPECT_DOUBLE_EQ(1, msg.x());
  EXPECT_DOUBLE_EQ(2, msg.y());
  EXPECT_DOUBLE_EQ(3, msg.z());
}

TEST(MsgsTest, SetVector2d)
{
  msgs::Vector2d msg;
  msgs::Set(&msg, math::Vector2d(1, 2));
  EXPECT_DOUBLE_EQ(1, msg.x());
  EXPECT_DOUBLE_EQ(2, msg.y());
}

TEST(MsgsTest, SetQuaternion)
{
  msgs::Quaternion msg;
  msgs::Set(&msg, math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));
  EXPECT_TRUE(math::equal(msg.x(), -0.65328148243818818));
  EXPECT_TRUE(math::equal(msg.y(), 0.27059805007309856));
  EXPECT_TRUE(math::equal(msg.z(), 0.65328148243818829));
  EXPECT_TRUE(math::equal(msg.w(), 0.27059805007309851));
}

TEST(MsgsTest, SetPose)
{
  msgs::Pose msg;
  msgs::Set(&msg, math::Pose(math::Vector3(1, 2, 3),
        math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI)));

  EXPECT_DOUBLE_EQ(1, msg.position().x());
  EXPECT_DOUBLE_EQ(2, msg.position().y());
  EXPECT_DOUBLE_EQ(3, msg.position().z());

  EXPECT_TRUE(math::equal(msg.orientation().x(), -0.65328148243818818));
  EXPECT_TRUE(math::equal(msg.orientation().y(), 0.27059805007309856));
  EXPECT_TRUE(math::equal(msg.orientation().z(), 0.65328148243818829));
  EXPECT_TRUE(math::equal(msg.orientation().w(), 0.27059805007309851));
}

TEST(MsgsTest, SetColor)
{
  msgs::Color msg;
  msgs::Set(&msg, common::Color(.1, .2, .3, 1.0));
  EXPECT_TRUE(math::equal(0.1f, msg.r()));
  EXPECT_TRUE(math::equal(0.2f, msg.g()));
  EXPECT_TRUE(math::equal(0.3f, msg.b()));
  EXPECT_TRUE(math::equal(1.0f, msg.a()));
}

TEST(MsgsTest, SetTime)
{
  msgs::Time msg;
  msgs::Set(&msg, common::Time(2, 123));
  EXPECT_EQ(2, msg.sec());
  EXPECT_EQ(123, msg.nsec());
}

TEST(MsgsTest, SetPlane)
{
  msgs::PlaneGeom msg;
  msgs::Set(&msg, math::Plane(math::Vector3(0, 0, 1),
                              math::Vector2d(123, 456), 1.0));

  EXPECT_DOUBLE_EQ(0, msg.normal().x());
  EXPECT_DOUBLE_EQ(0, msg.normal().y());
  EXPECT_DOUBLE_EQ(1, msg.normal().z());

  EXPECT_DOUBLE_EQ(123, msg.size().x());
  EXPECT_DOUBLE_EQ(456, msg.size().y());

  EXPECT_TRUE(math::equal(1.0, msg.d()));
}

TEST(MsgsTest, GUIFromRML)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("gui.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <gui fullscreen='true'>\
         <camera name='camera'>\
           <view_controller>fps</view_controller>\
           <pose>1 2 3 0 0 0</pose>\
           <track_visual>\
             <name>track</name>\
             <min_dist>0.2</min_dist>\
             <max_dist>1.0</max_dist>\
           </track_visual>\
         </camera>\
         </gui>\
       </gazebo>", rml);
  msgs::GUI msg = msgs::GUIFromRML(rml);
}

TEST(MsgsTest, GUIFromRML_EmptyTrackVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("gui.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <gui fullscreen='true'>\
         <camera name='camera'>\
           <view_controller>fps</view_controller>\
           <pose>1 2 3 0 0 0</pose>\
           <track_visual>\
             <name>visual_name</name>\
             <min_dist>0.1</min_dist>\
             <max_dist>1.0</max_dist>\
           </track_visual>\
         </camera>\
         </gui>\
       </gazebo>", rml);
  msgs::GUI msg = msgs::GUIFromRML(rml);
}

TEST(MsgsTest, GUIFromRML_WithEmptyCamera)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("gui.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <gui fullscreen='true'>\
         <camera name='camera'>\
         </camera>\
         </gui>\
       </gazebo>", rml);
  msgs::GUI msg = msgs::GUIFromRML(rml);
}

TEST(MsgsTest, GUIFromRML_WithoutCamera)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("gui.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <gui fullscreen='true'>\
         </gui>\
       </gazebo>", rml);
  msgs::GUI msg = msgs::GUIFromRML(rml);
}

TEST(MsgsTest, LightFromRML_ListDirectional)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("light.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <light type='directional' name='sun'>\
           <cast_shadows>true</cast_shadows>\
           <pose>0 0 10 0 0 0</pose>\
           <diffuse>0.8 0.8 0.8 1</diffuse>\
           <specular>0 0 0 1</specular>\
           <attenuation>\
             <range>20</range>\
             <constant>0.8</constant>\
             <linear>0.01</liner>\
             <quadratic>0.0</quadratic>\
           </attenuation>\
           <direction>1.0 1.0 -1.0</direction>\
         </light>\
       </gazebo>", rml);
  msgs::Light msg = msgs::LightFromRML(rml);
}

TEST(MsgsTest, LightFromRML_LightSpot)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("light.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <light type='spot' name='lamp'>\
           <pose>0 0 10 0 0 0</pose>\
           <diffuse>0.8 0.8 0.8 1</diffuse>\
           <specular>0 0 0 1</specular>\
           <spot>\
             <inner_angle>0</inner_angle>\
             <outer_angle>1</outer_angle>\
             <falloff>0.1</falloff>\
           </spot>\
           <attenuation>\
             <range>20</range>\
             <constant>0.8</constant>\
             <linear>0.01</linear>\
             <quadratic>0.0</quadratic>\
           </attenuation>\
           <direction>1.0 1.0 -1.0</direction>\
         </light>\
       </gazebo>", rml);
  msgs::Light msg = msgs::LightFromRML(rml);
}

TEST(MsgsTest, LightFromRML_LightPoint)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("light.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <light type='point' name='lamp'>\
           <pose>0 0 10 0 0 0</pose>\
           <diffuse>0.8 0.8 0.8 1</diffuse>\
           <specular>0 0 0 1</specular>\
           <attenuation>\
             <range>20</range>\
             <constant>0.8</constant>\
             <linear>0.01</linear>\
             <quadratic>0.0</quadratic>\
           </attenuation>\
         </light>\
       </gazebo>", rml);
  msgs::Light msg = msgs::LightFromRML(rml);
}

TEST(MsgsTest, LightFromRML_LighBadType)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("light.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <light type='_bad_' name='lamp'>\
         </light>\
       </gazebo>", rml);
  msgs::Light msg = msgs::LightFromRML(rml);
}

// Plane visual
TEST(MsgsTest, VisualFromRML_PlaneVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <plane><normal>0 0 1</normal></plane>\
           </geometry>\
           <material><script>Gazebo/Grey</script></material>\
         </visual>\
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_BoxVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <box><size>1 1 1</size></box>\
           </geometry>\
           <material><script>Gazebo/Grey'</script></material>\
         </visual>\
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_SphereVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
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
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_CylinderVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <cylinder><radius>1</radius><length>1.0</length></cylinder\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='normal_map_object_space'/>\
           </material>\
         </visual>\
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_MeshVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <geometry>\
             <mesh><scale>1 1 1</scale><uri>test1.mesh</uri></mesh>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='vertex'/>\
           </material>\
         </visual>\
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_ImageVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <cast_shadows>false</cast_shadows>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
             <image>\
               <scale>1</scale>\
               <height>1</height>\
               <threshold>255</threshold>\
               <granularity>10</granularit>\
               <uri>test2.mesh</uri>\
             <image>\
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
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_HeigthmapVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
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
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_NoGeometry)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
         </visual>\
      </gazebo>", rml);
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromRML(rml),
      common::Exception);
}

TEST(MsgsTest, VisualFromRML_ShaderTypeThrow)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
             <heightmap>\
               <size>1 2 3</size>\
               <uri>test4.mesh</uri>\
               <pos>0 0 0</pos>\
             </heightmap>\
           </geometry>\
           <shader type='throw'/>\
           </material>\
         </visual>\
      </gazebo>", rml);
  msgs::Visual msg = msgs::VisualFromRML(rml);
}

TEST(MsgsTest, VisualFromRML_BadGeometryVisual)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='pixel'/>\
           </material>\
         </visual>\
      </gazebo>", rml);
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromRML(rml),
               common::Exception);
}

TEST(MsgsTest, VisualFromRML_BadGeometryType)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("visual.rml", rml);
  EXPECT_FALSE(rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
           <pose>1 1 1 1 2 3</pose>\
           <geometry>\
             <bad_type/>\
           </geometry>\
           <material><script>Gazebo/Grey</script>\
           <shader type='pixel'/>\
           </material>\
         </visual>\
      </gazebo>", rml));

  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <visual name='visual'>\
         </visual>\
      </gazebo>", rml);

  rml::ElementPtr badElement(new rml::Element());
  badElement->SetName("bad_type");
  rml->GetElement("geometry")->InsertElement(badElement);
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromRML(rml),
      common::Exception);
}

TEST(MsgsTest, VisualFromRML_BadFogType)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("scene.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>true</shadows>\
           <fog><color>1 1 1 1</color> <type>throw</type>\
           <start>0</start> <end>10</end> <density>1</density> </fog>\
           <grid>false</grid>\
         </scene>\
      </gazebo>", rml);
  EXPECT_THROW(msgs::Scene msg = msgs::SceneFromRML(rml), common::Exception);
}

TEST(MsgsTest, VisualSceneFromRML_A)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("scene.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>true</shadows>\
           <fog>1 1 1 1' type='linear' start='0' end='10' density='1'/>\
           <grid>false</grid>\
         </scene>\
      </gazebo>", rml);
  msgs::Scene msg = msgs::SceneFromRML(rml);
}

TEST(MsgsTest, VisualSceneFromRML_B)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("scene.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>false</shadows>\
           <fog><color>1 1 1 1</color><type>exp</type><start>0</start>\
           <end>10</end><density>1<density/>\
         </scene>\
      </gazebo>", rml);
  msgs::Scene msg = msgs::SceneFromRML(rml);
}

TEST(MsgsTest, VisualSceneFromRML_C)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("scene.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>false</shadows>\
           <fog><color>1 1 1 1</color>\
           <type>exp2</type><start>0</start><end>10</end>\
           <density>1</density>\
           <grid>true</grid>\
         </scene>\
      </gazebo>", rml);
  msgs::Scene msg = msgs::SceneFromRML(rml);
}

TEST(MsgsTest, VisualSceneFromRML_CEmpty)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("scene.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <scene>\
         </scene>\
      </gazebo>", rml);
  msgs::Scene msg = msgs::SceneFromRML(rml);
}

TEST(MsgsTest, VisualSceneFromRML_CEmptyNoSky)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("scene.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
         <scene>\
           <background>0 0 0 1</background>\
         </scene>\
      </gazebo>", rml);
  msgs::Scene msg = msgs::SceneFromRML(rml);
}

/////////////////////////////////////////////////
TEST(MsgsTest, MeshFromRML)
{
  rml::ElementPtr rml(new rml::Element());
  rml::initFile("geometry.rml", rml);
  rml::readString(
      "<rml version='" RML_VERSION "'>\
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
         </visual>\
      </rml>", rml);

  msgs::MeshGeom msg = msgs::MeshFromRML(rml->GetElement("mesh"));
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
