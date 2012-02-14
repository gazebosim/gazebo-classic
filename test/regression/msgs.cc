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
#include "msgs/msgs.h"
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
    msgs::String msg;
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
    msgs::String msg;
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
    EXPECT_TRUE(math::equal(0.1f, v.R()));
    EXPECT_TRUE(math::equal(0.2f, v.G()));
    EXPECT_TRUE(math::equal(0.3f, v.B()));
    EXPECT_TRUE(math::equal(1.0f, v.A()));
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
    sdf::initFile("sdf/gui.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <gui fullscreen='true'>\
           <camera name='camera'>\
             <view_controller type='fps'/>\
             <origin pose='1 2 3 0 0 0'/>\
             <track_visual name='track'>\
               <min_dist>0.2</min_dist>\
               <max_dist>1.0</max_dist>\
             </track_visual>\
           </camera>\
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }

  // Empty track visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/gui.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <gui fullscreen='true'>\
           <camera name='camera'>\
             <view_controller type='fps'/>\
             <origin pose='1 2 3 0 0 0'/>\
             <track_visual name='track'>\
             </track_visual>\
           </camera>\
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }


  // GUI With empty camera
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/gui.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <gui fullscreen='true'>\
           <camera name='camera'>\
           </camera>\
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }
  // GUI without camera
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/gui.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <gui fullscreen='true'>\
         </gazebo>", sdf);
    msgs::GUI msg = msgs::GUIFromSDF(sdf);
  }

  // List Directional
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/light.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <light type='directional' name='sun' cast_shadows='true'>\
             <origin pose='0 0 10 0 0 0'/>\
             <diffuse rgba='0.8 0.8 0.8 1'/>\
             <specular rgba='0 0 0 1'/>\
             <attenuation range='20' constant='0.8' linear='0.01'\
                          quadratic='0.0'/>\
             <direction xyz='1.0 1.0 -1.0'/>\
           </light>\
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }

  // Light Spot
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/light.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <light type='spot' name='lamp' cast_shadows='true'>\
             <origin pose='0 0 10 0 0 0'/>\
             <diffuse rgba='0.8 0.8 0.8 1'/>\
             <specular rgba='0 0 0 1'/>\
             <spot inner_angle='0' outer_angle='1' falloff='0.1'/>\
             <attenuation range='20' constant='0.8' linear='0.01'\
                          quadratic='0.0'/>\
             <direction xyz='1.0 1.0 -1.0'/>\
           </light>\
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }

  // Light Point
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/light.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <light type='point' name='lamp' cast_shadows='true'>\
             <origin pose='0 0 10 0 0 0'/>\
             <diffuse rgba='0.8 0.8 0.8 1'/>\
             <specular rgba='0 0 0 1'/>\
             <attenuation range='20' constant='0.8' linear='0.01'\
                          quadratic='0.0'/>\
           </light>\
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }

  // Light bad type
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/light.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <light type='_bad_' name='lamp' cast_shadows='true'>\
           </light>\
         </gazebo>", sdf);
    msgs::Light msg = msgs::LightFromSDF(sdf);
  }


  // Plane visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <geometry>\
               <plane normal='0 0 1'/>\
             </geometry>\
             <material script='Gazebo/Grey'/>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Box visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <geometry>\
               <box size='1 1 1'/>\
             </geometry>\
             <material script='Gazebo/Grey'/>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Sphere visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <geometry>\
               <sphere radius='1'/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
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
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <geometry>\
               <cylinder radius='1' length='1.0'/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
             <shader type='normal_map_object_space'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Mesh visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <geometry>\
               <mesh scale='1 1 1' filename='test.mesh'/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
             <shader type='vertex'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Image visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <origin pose='1 1 1 1 2 3'/>\
             <geometry>\
               <image scale='1' height='1' threshold='255' granularity='10'\
                      filename='test.mesh'/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
               <shader type='pixel'/>\
               <ambient rgba='.1 .2 .3 1'/>\
               <diffuse rgba='.1 .2 .3 1'/>\
               <specular rgba='.1 .2 .3 1'/>\
               <emissive rgba='.1 .2 .3 1'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Heightmap visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <origin pose='1 1 1 1 2 3'/>\
             <geometry>\
               <heightmap size='1 2 3' filename='test.mesh' offset='0 0 1'/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
             <shader type='pixel'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }

  // Visual no geometry
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
           </visual>\
        </gazebo>", sdf);
    msgs::Visual msg = msgs::VisualFromSDF(sdf);
  }


  // Shader type throw
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <origin pose='1 1 1 1 2 3'/>\
             <geometry>\
               <heightmap size='1 2 3' filename='test.mesh' offset='0 0 1'/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
             <shader type='throw'/>\
             </material>\
           </visual>\
        </gazebo>", sdf);
    EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
                 common::Exception);
  }

  // THROW bad geometry visual
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/visual.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <origin pose='1 1 1 1 2 3'/>\
             <geometry>\
             </geometry>\
             <material script='Gazebo/Grey'>\
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
    sdf::initFile("sdf/visual.sdf", sdf);
    EXPECT_FALSE(sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual' cast_shadows='false'>\
             <origin pose='1 1 1 1 2 3'/>\
             <geometry>\
               <bad_type/>\
             </geometry>\
             <material script='Gazebo/Grey'>\
             <shader type='pixel'/>\
             </material>\
           </visual>\
        </gazebo>", sdf));

    sdf::readString(
        "<gazebo version='1.0'>\
           <visual name='visual'>\
           </visual>\
        </gazebo>", sdf);

    sdf::ElementPtr badElement(new sdf::Element());
    badElement->SetName("bad_type");
    sdf->GetOrCreateElement("geometry")->InsertElement(badElement);
    EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
                 common::Exception);
  }

  // Throw bad fog type
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/scene.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <scene>\
             <ambient rgba='0.1 0.1 0.1 1'/>\
             <background rgba='0 0 0 1'/>\
             <shadows enabled='true'/>\
             <fog rgba='1 1 1 1' type='throw' start='0' end='10' density='1'/>\
             <grid enabled='false'/>\
           </scene>\
        </gazebo>", sdf);
    EXPECT_THROW(msgs::Scene msg = msgs::SceneFromSDF(sdf), common::Exception);
  }


  // Scene A
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/scene.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <scene>\
             <ambient rgba='0.1 0.1 0.1 1'/>\
             <background rgba='0 0 0 1'/>\
             <shadows enabled='true'/>\
             <fog rgba='1 1 1 1' type='linear' start='0' end='10' density='1'/>\
             <grid enabled='false'/>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene B
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/scene.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <scene>\
             <ambient rgba='0.1 0.1 0.1 1'/>\
             <background rgba='0 0 0 1'/>\
             <shadows enabled='false'/>\
             <fog rgba='1 1 1 1' type='exp' start='0' end='10' density='1'/>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene C
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/scene.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <scene>\
             <ambient rgba='0.1 0.1 0.1 1'/>\
             <background rgba='0 0 0 1'/>\
             <shadows enabled='false'/>\
             <fog rgba='1 1 1 1' type='exp2' start='0' end='10' density='1'/>\
             <grid enabled='true'/>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene C empty
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/scene.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <scene>\
           </scene>\
        </gazebo>", sdf);
    msgs::Scene msg = msgs::SceneFromSDF(sdf);
  }

  // Scene C no sky
  {
    sdf::ElementPtr sdf(new sdf::Element());
    sdf::initFile("sdf/scene.sdf", sdf);
    sdf::readString(
        "<gazebo version='1.0'>\
           <scene>\
             <background rgba='0 0 0 1'>\
               <sky material='Gazebo/Clouds'/>\
             </background>\
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
