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
#include "msgs/msgs.hh"
#include "ServerFixture.hh"

using namespace gazebo;
class MsgsTest : public ServerFixture
{
};

TEST_F(MsgsTest, Misc)
{
  // Test msgs::Init, and msgs::GetHeader
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

  {
    msgs::Request *request = msgs::CreateRequest("help", "me");
    EXPECT_STREQ("help", request->request().c_str());
    EXPECT_STREQ("me", request->data().c_str());
    EXPECT_GT(request->id(), 0);
  }

  {
    common::Time t = common::Time::GetWallTime();
    msgs::Time msg;
    msgs::Stamp(&msg);
    EXPECT_EQ(t.sec, msg.sec());
    EXPECT_TRUE(t.nsec <= msg.nsec());
  }

  {
    common::Time t = common::Time::GetWallTime();
    msgs::Header msg;
    msgs::Stamp(&msg);
    EXPECT_EQ(t.sec, msg.stamp().sec());
    EXPECT_TRUE(t.nsec <= msg.stamp().nsec());
  }

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

  // Throw, msgs::Package, bad message
  {
    msgs::GzString msg;
    EXPECT_THROW(msgs::Package("test_type", msg), common::Exception);
  }
}

TEST_F(MsgsTest, Convert)
{
  {
    msgs::Vector3d msg = msgs::Convert(math::Vector3(1, 2, 3));
    EXPECT_EQ(1, msg.x());
    EXPECT_EQ(2, msg.y());
    EXPECT_EQ(3, msg.z());

    math::Vector3 v = msgs::Convert(msg);
    EXPECT_EQ(1, v.x);
    EXPECT_EQ(2, v.y);
    EXPECT_EQ(3, v.z);
  }

  {
    msgs::Quaternion msg =
      msgs::Convert(math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));
    EXPECT_TRUE(math::equal(msg.x(), -0.65328148243818818));
    EXPECT_TRUE(math::equal(msg.y(), 0.27059805007309856));
    EXPECT_TRUE(math::equal(msg.z(), 0.65328148243818829));
    EXPECT_TRUE(math::equal(msg.w(), 0.27059805007309851));

    math::Quaternion v = msgs::Convert(msg);
    EXPECT_TRUE(math::equal(v.x, -0.65328148243818818));
    EXPECT_TRUE(math::equal(v.y, 0.27059805007309856));
    EXPECT_TRUE(math::equal(v.z, 0.65328148243818829));
    EXPECT_TRUE(math::equal(v.w, 0.27059805007309851));
  }

  {
    msgs::Pose msg = msgs::Convert(math::Pose(math::Vector3(1, 2, 3),
          math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI)));

    EXPECT_EQ(1, msg.position().x());
    EXPECT_EQ(2, msg.position().y());
    EXPECT_EQ(3, msg.position().z());

    EXPECT_TRUE(math::equal(msg.orientation().x(), -0.65328148243818818));
    EXPECT_TRUE(math::equal(msg.orientation().y(), 0.27059805007309856));
    EXPECT_TRUE(math::equal(msg.orientation().z(), 0.65328148243818829));
    EXPECT_TRUE(math::equal(msg.orientation().w(), 0.27059805007309851));

    math::Pose v = msgs::Convert(msg);
    EXPECT_EQ(1, v.pos.x);
    EXPECT_EQ(2, v.pos.y);
    EXPECT_EQ(3, v.pos.z);

    EXPECT_TRUE(math::equal(v.rot.x, -0.65328148243818818));
    EXPECT_TRUE(math::equal(v.rot.y, 0.27059805007309856));
    EXPECT_TRUE(math::equal(v.rot.z, 0.65328148243818829));
    EXPECT_TRUE(math::equal(v.rot.w, 0.27059805007309851));
  }

  {
    msgs::Color msg = msgs::Convert(common::Color(.1, .2, .3, 1.0));
    EXPECT_TRUE(math::equal(0.1f, msg.r()));
    EXPECT_TRUE(math::equal(0.2f, msg.g()));
    EXPECT_TRUE(math::equal(0.3f, msg.b()));
    EXPECT_TRUE(math::equal(1.0f, msg.a()));

    common::Color v = msgs::Convert(msg);
    EXPECT_TRUE(math::equal(0.1f, v.r));
    EXPECT_TRUE(math::equal(0.2f, v.g));
    EXPECT_TRUE(math::equal(0.3f, v.b));
    EXPECT_TRUE(math::equal(1.0f, v.a));
  }

  {
    msgs::Time msg = msgs::Convert(common::Time(2, 123));
    EXPECT_EQ(2, msg.sec());
    EXPECT_EQ(123, msg.nsec());

    common::Time v = msgs::Convert(msg);
    EXPECT_EQ(2, v.sec);
    EXPECT_EQ(123, v.nsec);
  }

  {
    msgs::PlaneGeom msg = msgs::Convert(math::Plane(math::Vector3(0, 0, 1),
          math::Vector2d(123, 456), 1.0));

    EXPECT_EQ(0, msg.normal().x());
    EXPECT_EQ(0, msg.normal().y());
    EXPECT_EQ(1, msg.normal().z());

    EXPECT_EQ(123, msg.size().x());
    EXPECT_EQ(456, msg.size().y());

    EXPECT_TRUE(math::equal(1.0, msg.d()));

    math::Plane v = msgs::Convert(msg);
    EXPECT_EQ(0, v.normal.x);
    EXPECT_EQ(0, v.normal.y);
    EXPECT_EQ(1, v.normal.z);

    EXPECT_EQ(123, v.size.x);
    EXPECT_EQ(456, v.size.y);

    EXPECT_TRUE(math::equal(1.0, v.d));
  }
}

TEST_F(MsgsTest, Set)
{
  {
    msgs::Vector3d msg;
    msgs::Set(&msg, math::Vector3(1, 2, 3));
    EXPECT_EQ(1, msg.x());
    EXPECT_EQ(2, msg.y());
    EXPECT_EQ(3, msg.z());
  }

  {
    msgs::Vector2d msg;
    msgs::Set(&msg, math::Vector2d(1, 2));
    EXPECT_EQ(1, msg.x());
    EXPECT_EQ(2, msg.y());
  }

  {
    msgs::Quaternion msg;
    msgs::Set(&msg, math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));
    EXPECT_TRUE(math::equal(msg.x(), -0.65328148243818818));
    EXPECT_TRUE(math::equal(msg.y(), 0.27059805007309856));
    EXPECT_TRUE(math::equal(msg.z(), 0.65328148243818829));
    EXPECT_TRUE(math::equal(msg.w(), 0.27059805007309851));
  }

  {
    msgs::Pose msg;
    msgs::Set(&msg, math::Pose(math::Vector3(1, 2, 3),
          math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI)));

    EXPECT_EQ(1, msg.position().x());
    EXPECT_EQ(2, msg.position().y());
    EXPECT_EQ(3, msg.position().z());

    EXPECT_TRUE(math::equal(msg.orientation().x(), -0.65328148243818818));
    EXPECT_TRUE(math::equal(msg.orientation().y(), 0.27059805007309856));
    EXPECT_TRUE(math::equal(msg.orientation().z(), 0.65328148243818829));
    EXPECT_TRUE(math::equal(msg.orientation().w(), 0.27059805007309851));
  }

  {
    msgs::Color msg;
    msgs::Set(&msg, common::Color(.1, .2, .3, 1.0));
    EXPECT_TRUE(math::equal(0.1f, msg.r()));
    EXPECT_TRUE(math::equal(0.2f, msg.g()));
    EXPECT_TRUE(math::equal(0.3f, msg.b()));
    EXPECT_TRUE(math::equal(1.0f, msg.a()));
  }

  {
    msgs::Time msg;
    msgs::Set(&msg, common::Time(2, 123));
    EXPECT_EQ(2, msg.sec());
    EXPECT_EQ(123, msg.nsec());
  }

  {
    msgs::PlaneGeom msg;
    msgs::Set(&msg, math::Plane(math::Vector3(0, 0, 1),
                                math::Vector2d(123, 456), 1.0));

    EXPECT_EQ(0, msg.normal().x());
    EXPECT_EQ(0, msg.normal().y());
    EXPECT_EQ(1, msg.normal().z());

    EXPECT_EQ(123, msg.size().x());
    EXPECT_EQ(456, msg.size().y());

    EXPECT_TRUE(math::equal(1.0, msg.d()));
  }
}

TEST_F(MsgsTest, SDF)
{
  // GUI With camera
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("gui.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
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
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }

  // Empty track visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("gui.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
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
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }


  // GUI With empty camera
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("gui.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <gui fullscreen='true'>\
           <camera name='camera'>\
           </camera>\
           </gui>\
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }
  // GUI without camera
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("gui.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <gui fullscreen='true'>\
           </gui>\
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }

  // List Directional
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("light.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
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
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }

  // Light Spot
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("light.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
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
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }

  // Light Point
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("light.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
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
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }

  // Light bad type
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("light.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <light type='_bad_' name='lamp'>\
           </light>\
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }


  // Plane visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <cast_shadows>false</cast_shadows>\
             <geometry>\
               <plane><normal>0 0 1</normal></plane>\
             </geometry>\
             <material><script>Gazebo/Grey</script></material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Box visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <cast_shadows>false</cast_shadows>\
             <geometry>\
               <box><size>1 1 1</size></box>\
             </geometry>\
             <material><script>Gazebo/Grey'</script></material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Sphere visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
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
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Cylinder visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <cast_shadows>false</cast_shadows>\
             <geometry>\
               <cylinder><radius>1</radius><length>1.0</length></cylinder\
             </geometry>\
             <material><script>Gazebo/Grey</script>\
             <shader type='normal_map_object_space'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Mesh visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <cast_shadows>false</cast_shadows>\
             <geometry>\
               <mesh><scale>1 1 1</scale><uri>test.mesh</uri></mesh>\
             </geometry>\
             <material><script>Gazebo/Grey</script>\
             <shader type='vertex'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Image visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <cast_shadows>false</cast_shadows>\
             <pose>1 1 1 1 2 3</pose>\
             <geometry>\
               <image>\
                 <scale>1</scale>\
                 <height>1</height>\
                 <threshold>255</threshold>\
                 <granularity>10</granularit>\
                 <uri>test.mesh</uri>\
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
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Heightmap visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <cast_shadows>false</cast_shadows>\
             <pose>1 1 1 1 2 3</pose>\
             <geometry>\
               <heightmap>\
                 <size>1 2 3</size>\
                 <uri>test.mesh</uri>\
                 <pos>0 0 1</pos>\
               </heightmap>\
             </geometry>\
             <material><script>Gazebo/Grey</script>\
             <shader type='pixel'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Visual no geometry
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Shader type throw
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <pose>1 1 1 1 2 3</pose>\
             <geometry>\
               <heightmap>\
                 <size>1 2 3</size>\
                 <uri>test.mesh</uri>\
                 <pos>0 0 0</pos>\
               </heightmap>\
             </geometry>\
             <shader type='throw'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // THROW bad geometry visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <pose>1 1 1 1 2 3</pose>\
             <geometry>\
             </geometry>\
             <material><script>Gazebo/Grey</script>\
             <shader type='pixel'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
                 common::Exception);
  }

  // Error bad geometry type
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("visual.sdf", sdf);
    EXPECT_FALSE(sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
             <pose>1 1 1 1 2 3</pose>\
             <geometry>\
               <bad_type/>\
             </geometry>\
             <material><script>Gazebo/Grey</script>\
             <shader type='pixel'/>\
             </material>\
           </visual>\
        </gazebo>", sdf));

    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <visual name='visual'>\
           </visual>\
        </gazebo>", sdf);

    sdf::ElementPtr badElement(new sdf::Element());
    badElement->SetName("bad_type");
    sdf->GetElement("geometry")->InsertElement(badElement);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Throw bad fog type
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("scene.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <scene>\
             <ambient>0.1 0.1 0.1 1</ambient>\
             <background>0 0 0 1</background>\
             <shadows>true</shadows>\
             <fog><color>1 1 1 1</color> <type>throw</type>\
             <start>0</start> <end>10</end> <density>1</density> </fog>\
             <grid>false</grid>\
           </scene>\
        </gazebo>", sdf);
    EXPECT_THROW(msgs::Scene msg = msgs::SceneFromSDF(sdf), common::Exception);
  }


  // Scene A
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("scene.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <scene>\
             <ambient>0.1 0.1 0.1 1</ambient>\
             <background>0 0 0 1</background>\
             <shadows>true</shadows>\
             <fog>1 1 1 1' type='linear' start='0' end='10' density='1'/>\
             <grid>false</grid>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene B
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("scene.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <scene>\
             <ambient>0.1 0.1 0.1 1</ambient>\
             <background>0 0 0 1</background>\
             <shadows>false</shadows>\
             <fog><color>1 1 1 1</color><type>exp</type><start>0</start>\
             <end>10</end><density>1<density/>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene C
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("scene.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <scene>\
             <ambient>0.1 0.1 0.1 1</ambient>\
             <background>0 0 0 1</background>\
             <shadows>false</shadows>\
             <fog><color>1 1 1 1</color>\
             <type>exp2</type><start>0</start><end>10</end>\
             <density>1</density>\
             <grid>true</grid>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene C empty
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("scene.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <scene>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene C no sky
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("scene.sdf", sdf);
    sdf::readString(
        "<sdf version='" SDF_VERSION "'>\
           <scene>\
             <background>0 0 0 1</background>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
