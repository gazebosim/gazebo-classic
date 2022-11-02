/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <ignition/msgs.hh>
#include <ignition/msgs/MessageTypes.hh>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Exception.hh"
#include "test/util.hh"

using namespace gazebo;

class MsgsTest : public gazebo::testing::AutoLogFixture { };

void TimeTest(const common::Time &_t, const msgs::Time &_msg)
{
  EXPECT_LE(_t.sec, _msg.sec());
  if (_t.sec == _msg.sec())
  {
    EXPECT_LE(_t.nsec, _msg.nsec());
  }
}

TEST_F(MsgsTest, Msg)
{
  common::Time t = common::Time::GetWallTime();

  msgs::Test msg, msg2;
  msgs::Init(msg, "_test_");
  msgs::Init(msg2);

  ASSERT_TRUE(msg.header().has_stamp());
  TimeTest(t, msg.header().stamp());
  EXPECT_STREQ("_test_", msg.header().str_id().c_str());

  ASSERT_TRUE(msg2.header().has_stamp());
  TimeTest(t, msg2.header().stamp());
  EXPECT_FALSE(msg2.header().has_str_id());

  msgs::Header *header = msgs::GetHeader(msg);
  EXPECT_STREQ("_test_", header->str_id().c_str());

  msgs::Header testHeader;
  testHeader.set_str_id("_hello_");
  header = msgs::GetHeader(testHeader);
  EXPECT_STREQ("_hello_", header->str_id().c_str());
}

TEST_F(MsgsTest, Request)
{
  msgs::Request *request = msgs::CreateRequest("help", "me");
  EXPECT_STREQ("help", request->request().c_str());
  EXPECT_STREQ("me", request->data().c_str());
  EXPECT_GT(request->id(), 0);
}

TEST_F(MsgsTest, Time)
{
  common::Time t = common::Time::GetWallTime();
  msgs::Time msg;
  msgs::Stamp(&msg);
  TimeTest(t, msg);
}


TEST_F(MsgsTest, TimeFromHeader)
{
  common::Time t = common::Time::GetWallTime();
  msgs::Header msg;
  msgs::Stamp(&msg);
  TimeTest(t, msg.stamp());
}


TEST_F(MsgsTest, Packet)
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

TEST_F(MsgsTest, BadPackage)
{
  msgs::GzString msg;
  EXPECT_THROW(msgs::Package("test_type", msg), common::Exception);
}

TEST_F(MsgsTest, ConvertDoubleToAny)
{
  msgs::Any msg = msgs::ConvertAny(1.0);
  EXPECT_EQ(msg.type(), msgs::Any::DOUBLE);
  ASSERT_TRUE(msg.has_double_value());
  EXPECT_DOUBLE_EQ(1, msg.double_value());
}

TEST_F(MsgsTest, ConvertIntToAny)
{
  msgs::Any msg = msgs::ConvertAny(2);
  EXPECT_EQ(msg.type(), msgs::Any::INT32);
  ASSERT_TRUE(msg.has_int_value());
  EXPECT_DOUBLE_EQ(2, msg.int_value());
}

TEST_F(MsgsTest, ConvertStringToAny)
{
  msgs::Any msg = msgs::ConvertAny("test_string");
  EXPECT_EQ(msg.type(), msgs::Any::STRING);
  ASSERT_TRUE(msg.has_string_value());
  EXPECT_EQ("test_string", msg.string_value());
}

TEST_F(MsgsTest, ConvertBoolToAny)
{
  msgs::Any msg = msgs::ConvertAny(true);

  EXPECT_EQ(msg.type(), msgs::Any::BOOLEAN);
  ASSERT_TRUE(msg.has_bool_value());
  EXPECT_TRUE(msg.bool_value());
}

TEST_F(MsgsTest, ConvertVector3dToAny)
{
  msgs::Any msg = msgs::ConvertAny(ignition::math::Vector3d(1, 2, 3));
  EXPECT_EQ(msg.type(), msgs::Any::VECTOR3D);
  ASSERT_TRUE(msg.has_vector3d_value());

  EXPECT_DOUBLE_EQ(1, msg.vector3d_value().x());
  EXPECT_DOUBLE_EQ(2, msg.vector3d_value().y());
  EXPECT_DOUBLE_EQ(3, msg.vector3d_value().z());
}

TEST_F(MsgsTest, ConvertColorToAny)
{
  msgs::Any msg = msgs::ConvertAny(ignition::math::Color(.1, .2, .3, 1.0));
  EXPECT_EQ(msg.type(), msgs::Any::COLOR);
  ASSERT_TRUE(msg.has_color_value());

  EXPECT_DOUBLE_EQ(0.1f, msg.color_value().r());
  EXPECT_DOUBLE_EQ(0.2f, msg.color_value().g());
  EXPECT_DOUBLE_EQ(0.3f, msg.color_value().b());
  EXPECT_DOUBLE_EQ(1.0f, msg.color_value().a());
}

TEST_F(MsgsTest, ConvertPose3dToAny)
{
  msgs::Any msg = msgs::ConvertAny(ignition::math::Pose3d(
      ignition::math::Vector3d(1, 2, 3),
      ignition::math::Quaterniond(4, 5, 6, 7)));

  EXPECT_EQ(msg.type(), msgs::Any::POSE3D);
  ASSERT_TRUE(msg.has_pose3d_value());

  EXPECT_DOUBLE_EQ(1, msg.pose3d_value().position().x());
  EXPECT_DOUBLE_EQ(2, msg.pose3d_value().position().y());
  EXPECT_DOUBLE_EQ(3, msg.pose3d_value().position().z());

  EXPECT_DOUBLE_EQ(msg.pose3d_value().orientation().w(), 4.0);
  EXPECT_DOUBLE_EQ(msg.pose3d_value().orientation().x(), 5.0);
  EXPECT_DOUBLE_EQ(msg.pose3d_value().orientation().y(), 6.0);
  EXPECT_DOUBLE_EQ(msg.pose3d_value().orientation().z(), 7.0);
}

TEST_F(MsgsTest, ConvertQuaternionToAny)
{
  msgs::Any msg = msgs::ConvertAny(
      ignition::math::Quaterniond(1, 2, 3, 4));

  EXPECT_EQ(msg.type(), msgs::Any::QUATERNIOND);
  ASSERT_TRUE(msg.has_quaternion_value());

  EXPECT_DOUBLE_EQ(msg.quaternion_value().w(), 1.0);
  EXPECT_DOUBLE_EQ(msg.quaternion_value().x(), 2.0);
  EXPECT_DOUBLE_EQ(msg.quaternion_value().y(), 3.0);
  EXPECT_DOUBLE_EQ(msg.quaternion_value().z(), 4.0);
}

TEST_F(MsgsTest, ConvertTimeToAny)
{
  msgs::Any msg = msgs::ConvertAny(common::Time(2, 123));

  EXPECT_EQ(msg.type(), msgs::Any::TIME);
  ASSERT_TRUE(msg.has_time_value());

  EXPECT_EQ(2, msg.time_value().sec());
  EXPECT_EQ(123, msg.time_value().nsec());
}

TEST_F(MsgsTest, CovertMathVector3ToMsgs)
{
  msgs::Vector3d msg = msgs::Convert(ignition::math::Vector3d(1, 2, 3));
  EXPECT_DOUBLE_EQ(1, msg.x());
  EXPECT_DOUBLE_EQ(2, msg.y());
  EXPECT_DOUBLE_EQ(3, msg.z());
}

TEST_F(MsgsTest, ConvertMsgsVector3dToMath)
{
  msgs::Vector3d msg = msgs::Convert(ignition::math::Vector3d(1, 2, 3));
  ignition::math::Vector3d v = msgs::ConvertIgn(msg);
  EXPECT_DOUBLE_EQ(1, v.X());
  EXPECT_DOUBLE_EQ(2, v.Y());
  EXPECT_DOUBLE_EQ(3, v.Z());
}

TEST_F(MsgsTest, ConvertMathQuaternionToMsgs)
{
  msgs::Quaternion msg =
    msgs::Convert(ignition::math::Quaterniond(M_PI * 0.25, M_PI * 0.5, M_PI));

  EXPECT_DOUBLE_EQ(msg.x(), -0.65328148243818818);
  EXPECT_DOUBLE_EQ(msg.y(), 0.27059805007309856);
  EXPECT_DOUBLE_EQ(msg.z(), 0.65328148243818829);
  EXPECT_DOUBLE_EQ(msg.w(), 0.27059805007309851);
}

TEST_F(MsgsTest, ConvertMsgsQuaternionToMath)
{
  msgs::Quaternion msg =
    msgs::Convert(ignition::math::Quaterniond(M_PI * 0.25, M_PI * 0.5, M_PI));
  ignition::math::Quaterniond v = msgs::ConvertIgn(msg);

  EXPECT_DOUBLE_EQ(v.X(), -0.65328148243818818);
  EXPECT_DOUBLE_EQ(v.Y(), 0.27059805007309856);
  EXPECT_DOUBLE_EQ(v.Z(), 0.65328148243818829);
  EXPECT_DOUBLE_EQ(v.W(), 0.27059805007309851);
}

TEST_F(MsgsTest, ConvertPoseMathToMsgs)
{
  msgs::Pose msg = msgs::Convert(ignition::math::Pose3d(
        ignition::math::Vector3d(1, 2, 3),
        ignition::math::Quaterniond(M_PI * 0.25, M_PI * 0.5, M_PI)));

  EXPECT_DOUBLE_EQ(1, msg.position().x());
  EXPECT_DOUBLE_EQ(2, msg.position().y());
  EXPECT_DOUBLE_EQ(3, msg.position().z());

  EXPECT_DOUBLE_EQ(msg.orientation().x(), -0.65328148243818818);
  EXPECT_DOUBLE_EQ(msg.orientation().y(), 0.27059805007309856);
  EXPECT_DOUBLE_EQ(msg.orientation().z(), 0.65328148243818829);
  EXPECT_DOUBLE_EQ(msg.orientation().w(), 0.27059805007309851);
}

TEST_F(MsgsTest, ConvertMsgPoseToMath)
{
  msgs::Pose msg = msgs::Convert(
      ignition::math::Pose3d(ignition::math::Vector3d(1, 2, 3),
        ignition::math::Quaterniond(M_PI * 0.25, M_PI * 0.5, M_PI)));
  ignition::math::Pose3d v = msgs::ConvertIgn(msg);

  EXPECT_DOUBLE_EQ(1, v.Pos().X());
  EXPECT_DOUBLE_EQ(2, v.Pos().Y());
  EXPECT_DOUBLE_EQ(3, v.Pos().Z());
  EXPECT_DOUBLE_EQ(v.Rot().X(), -0.65328148243818818);
  EXPECT_DOUBLE_EQ(v.Rot().Y(), 0.27059805007309856);
  EXPECT_DOUBLE_EQ(v.Rot().Z(), 0.65328148243818829);
  EXPECT_DOUBLE_EQ(v.Rot().W(), 0.27059805007309851);
}

TEST_F(MsgsTest, ConvertCommonColorToMsgs)
{
  msgs::Color msg = msgs::Convert(ignition::math::Color(.1, .2, .3, 1.0));

  EXPECT_FLOAT_EQ(0.1f, msg.r());
  EXPECT_FLOAT_EQ(0.2f, msg.g());
  EXPECT_FLOAT_EQ(0.3f, msg.b());
  EXPECT_FLOAT_EQ(1.0f, msg.a());
}

TEST_F(MsgsTest, ConvertMsgsColorToCommon)
{
  msgs::Color msg = msgs::Convert(ignition::math::Color(.1, .2, .3, 1.0));
  auto v = msgs::Convert(msg);

  EXPECT_FLOAT_EQ(0.1f, v.R());
  EXPECT_FLOAT_EQ(0.2f, v.G());
  EXPECT_FLOAT_EQ(0.3f, v.B());
  EXPECT_FLOAT_EQ(1.0f, v.A());
}

TEST_F(MsgsTest, ConvertCommonTimeToMsgs)
{
  msgs::Time msg = msgs::Convert(common::Time(2, 123));
  EXPECT_EQ(2, msg.sec());
  EXPECT_EQ(123, msg.nsec());

  common::Time v = msgs::Convert(msg);
  EXPECT_EQ(2, v.sec);
  EXPECT_EQ(123, v.nsec);
}

TEST_F(MsgsTest, ConvertMathInertialToMsgs)
{
  const auto pose = ignition::math::Pose3d(5, 6, 7, 0.4, 0.5, 0.6);
  msgs::Inertial msg = msgs::Convert(
      ignition::math::Inertiald(
        ignition::math::MassMatrix3d(12.0,
          ignition::math::Vector3d(2, 3, 4),
          ignition::math::Vector3d(0.1, 0.2, 0.3)),
        pose));

  EXPECT_DOUBLE_EQ(12.0, msg.mass());
  EXPECT_DOUBLE_EQ(2.0, msg.ixx());
  EXPECT_DOUBLE_EQ(3.0, msg.iyy());
  EXPECT_DOUBLE_EQ(4.0, msg.izz());
  EXPECT_DOUBLE_EQ(0.1, msg.ixy());
  EXPECT_DOUBLE_EQ(0.2, msg.ixz());
  EXPECT_DOUBLE_EQ(0.3, msg.iyz());
  EXPECT_EQ(pose, msgs::ConvertIgn(msg.pose()));
}

TEST_F(MsgsTest, ConvertMsgsInertialToMath)
{
  const auto pose = ignition::math::Pose3d(5, 6, 7, 0.4, 0.5, 0.6);
  msgs::Inertial msg = msgs::Convert(
      ignition::math::Inertiald(
        ignition::math::MassMatrix3d(12.0,
          ignition::math::Vector3d(2, 3, 4),
          ignition::math::Vector3d(0.1, 0.2, 0.3)),
        pose));
  auto inertial = msgs::Convert(msg);

  EXPECT_DOUBLE_EQ(12.0, inertial.MassMatrix().Mass());
  EXPECT_DOUBLE_EQ(2.0, inertial.MassMatrix().Ixx());
  EXPECT_DOUBLE_EQ(3.0, inertial.MassMatrix().Iyy());
  EXPECT_DOUBLE_EQ(4.0, inertial.MassMatrix().Izz());
  EXPECT_DOUBLE_EQ(0.1, inertial.MassMatrix().Ixy());
  EXPECT_DOUBLE_EQ(0.2, inertial.MassMatrix().Ixz());
  EXPECT_DOUBLE_EQ(0.3, inertial.MassMatrix().Iyz());
  EXPECT_EQ(pose, inertial.Pose());
}

TEST_F(MsgsTest, ConvertMathMassMatrix3ToMsgs)
{
  msgs::Inertial msg = msgs::Convert(
      ignition::math::MassMatrix3d(12.0,
        ignition::math::Vector3d(2, 3, 4),
        ignition::math::Vector3d(0.1, 0.2, 0.3)));

  EXPECT_DOUBLE_EQ(12.0, msg.mass());
  EXPECT_DOUBLE_EQ(2.0, msg.ixx());
  EXPECT_DOUBLE_EQ(3.0, msg.iyy());
  EXPECT_DOUBLE_EQ(4.0, msg.izz());
  EXPECT_DOUBLE_EQ(0.1, msg.ixy());
  EXPECT_DOUBLE_EQ(0.2, msg.ixz());
  EXPECT_DOUBLE_EQ(0.3, msg.iyz());
}

TEST_F(MsgsTest, ConvertMathPlaneToMsgs)
{
  msgs::PlaneGeom msg = msgs::Convert(
      ignition::math::Planed(ignition::math::Vector3d(0, 0, 1),
        ignition::math::Vector2d(123, 456), 1.0));

  EXPECT_DOUBLE_EQ(0, msg.normal().x());
  EXPECT_DOUBLE_EQ(0, msg.normal().y());
  EXPECT_DOUBLE_EQ(1, msg.normal().z());

  EXPECT_DOUBLE_EQ(123, msg.size().x());
  EXPECT_DOUBLE_EQ(456, msg.size().y());
}

TEST_F(MsgsTest, ConvertMsgsPlaneToMath)
{
  msgs::PlaneGeom msg = msgs::Convert(
      ignition::math::Planed(ignition::math::Vector3d(0, 0, 1),
        ignition::math::Vector2d(123, 456), 1.0));
  ignition::math::Planed v = msgs::ConvertIgn(msg);

  EXPECT_DOUBLE_EQ(0, v.Normal().X());
  EXPECT_DOUBLE_EQ(0, v.Normal().Y());
  EXPECT_DOUBLE_EQ(1, v.Normal().Z());

  EXPECT_DOUBLE_EQ(123, v.Size().X());
  EXPECT_DOUBLE_EQ(456, v.Size().Y());

  EXPECT_DOUBLE_EQ(1.0, v.Offset());
}

//////////////////////////////////////////////////
void CompareMsgsShaderTypeToString(const msgs::Material::ShaderType _type)
{
  EXPECT_EQ(_type, msgs::ConvertShaderType(msgs::ConvertShaderType(_type)));
}

//////////////////////////////////////////////////
TEST_F(MsgsTest, ConvertMsgsShaderTypeToString)
{
  CompareMsgsShaderTypeToString(msgs::Material::NORMAL_MAP_OBJECT_SPACE);
  CompareMsgsShaderTypeToString(msgs::Material::NORMAL_MAP_TANGENT_SPACE);
  CompareMsgsShaderTypeToString(msgs::Material::PIXEL);
  CompareMsgsShaderTypeToString(msgs::Material::VERTEX);
}

//////////////////////////////////////////////////
void CompareMsgsJointTypeToString(const msgs::Joint::Type _type)
{
  EXPECT_EQ(_type, msgs::ConvertJointType(msgs::ConvertJointType(_type)));
}

//////////////////////////////////////////////////
TEST_F(MsgsTest, ConvertMsgsJointTypeToString)
{
  CompareMsgsJointTypeToString(msgs::Joint::REVOLUTE);
  CompareMsgsJointTypeToString(msgs::Joint::REVOLUTE2);
  CompareMsgsJointTypeToString(msgs::Joint::PRISMATIC);
  CompareMsgsJointTypeToString(msgs::Joint::UNIVERSAL);
  CompareMsgsJointTypeToString(msgs::Joint::BALL);
  CompareMsgsJointTypeToString(msgs::Joint::SCREW);
  CompareMsgsJointTypeToString(msgs::Joint::GEARBOX);
  CompareMsgsJointTypeToString(msgs::Joint::FIXED);
}

//////////////////////////////////////////////////
void CompareMsgsGeometryTypeToString(const msgs::Geometry::Type _type)
{
  EXPECT_EQ(_type, msgs::ConvertGeometryType(msgs::ConvertGeometryType(_type)));
}

//////////////////////////////////////////////////
TEST_F(MsgsTest, ConvertMsgsGeometryTypeToString)
{
  CompareMsgsGeometryTypeToString(msgs::Geometry::BOX);
  CompareMsgsGeometryTypeToString(msgs::Geometry::SPHERE);
  CompareMsgsGeometryTypeToString(msgs::Geometry::CYLINDER);
  CompareMsgsGeometryTypeToString(msgs::Geometry::PLANE);
  CompareMsgsGeometryTypeToString(msgs::Geometry::IMAGE);
  CompareMsgsGeometryTypeToString(msgs::Geometry::HEIGHTMAP);
  CompareMsgsGeometryTypeToString(msgs::Geometry::MESH);
  CompareMsgsGeometryTypeToString(msgs::Geometry::POLYLINE);

  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::BOX), "box");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::SPHERE), "sphere");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::CYLINDER), "cylinder");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::PLANE), "plane");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::IMAGE), "image");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::HEIGHTMAP), "heightmap");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::MESH), "mesh");
  EXPECT_EQ(msgs::ConvertGeometryType(msgs::Geometry::POLYLINE), "polyline");

  EXPECT_EQ(msgs::ConvertGeometryType("box"), msgs::Geometry::BOX);
  EXPECT_EQ(msgs::ConvertGeometryType("sphere"), msgs::Geometry::SPHERE);
  EXPECT_EQ(msgs::ConvertGeometryType("cylinder"), msgs::Geometry::CYLINDER);
  EXPECT_EQ(msgs::ConvertGeometryType("plane"), msgs::Geometry::PLANE);
  EXPECT_EQ(msgs::ConvertGeometryType("image"), msgs::Geometry::IMAGE);
  EXPECT_EQ(msgs::ConvertGeometryType("heightmap"), msgs::Geometry::HEIGHTMAP);
  EXPECT_EQ(msgs::ConvertGeometryType("mesh"), msgs::Geometry::MESH);
  EXPECT_EQ(msgs::ConvertGeometryType("polyline"), msgs::Geometry::POLYLINE);
}

TEST_F(MsgsTest, SetVector3)
{
  msgs::Vector3d msg;
  msgs::Set(&msg, ignition::math::Vector3d(1, 2, 3));
  EXPECT_DOUBLE_EQ(1, msg.x());
  EXPECT_DOUBLE_EQ(2, msg.y());
  EXPECT_DOUBLE_EQ(3, msg.z());
}

TEST_F(MsgsTest, SetVector2d)
{
  msgs::Vector2d msg;
  msgs::Set(&msg, ignition::math::Vector2d(1, 2));
  EXPECT_DOUBLE_EQ(1, msg.x());
  EXPECT_DOUBLE_EQ(2, msg.y());
}

TEST_F(MsgsTest, SetQuaternion)
{
  msgs::Quaternion msg;
  msgs::Set(&msg, ignition::math::Quaterniond(M_PI * 0.25, M_PI * 0.5, M_PI));
  EXPECT_DOUBLE_EQ(msg.x(), -0.65328148243818818);
  EXPECT_DOUBLE_EQ(msg.y(), 0.27059805007309856);
  EXPECT_DOUBLE_EQ(msg.z(), 0.65328148243818829);
  EXPECT_DOUBLE_EQ(msg.w(), 0.27059805007309851);
}

TEST_F(MsgsTest, SetPose)
{
  msgs::Pose msg;
  msgs::Set(&msg, ignition::math::Pose3d(ignition::math::Vector3d(1, 2, 3),
        ignition::math::Quaterniond(M_PI * 0.25, M_PI * 0.5, M_PI)));

  EXPECT_DOUBLE_EQ(1, msg.position().x());
  EXPECT_DOUBLE_EQ(2, msg.position().y());
  EXPECT_DOUBLE_EQ(3, msg.position().z());

  EXPECT_DOUBLE_EQ(msg.orientation().x(), -0.65328148243818818);
  EXPECT_DOUBLE_EQ(msg.orientation().y(), 0.27059805007309856);
  EXPECT_DOUBLE_EQ(msg.orientation().z(), 0.65328148243818829);
  EXPECT_DOUBLE_EQ(msg.orientation().w(), 0.27059805007309851);
}

TEST_F(MsgsTest, SetColor)
{
  msgs::Color msg;
  msgs::Set(&msg, ignition::math::Color(.1, .2, .3, 1.0));
  EXPECT_FLOAT_EQ(0.1f, msg.r());
  EXPECT_FLOAT_EQ(0.2f, msg.g());
  EXPECT_FLOAT_EQ(0.3f, msg.b());
  EXPECT_FLOAT_EQ(1.0f, msg.a());
}

TEST_F(MsgsTest, SetTime)
{
  msgs::Time msg;
  msgs::Set(&msg, common::Time(2, 123));
  EXPECT_EQ(2, msg.sec());
  EXPECT_EQ(123, msg.nsec());
}

TEST_F(MsgsTest, SetInertial)
{
  const auto pose = ignition::math::Pose3d(5, 6, 7, 0.4, 0.5, 0.6);
  msgs::Inertial msg;
  msgs::Set(&msg, ignition::math::Inertiald(
      ignition::math::MassMatrix3d(
        12.0,
        ignition::math::Vector3d(2, 3, 4),
        ignition::math::Vector3d(0.1, 0.2, 0.3)),
      pose));

  EXPECT_DOUBLE_EQ(12.0, msg.mass());
  EXPECT_DOUBLE_EQ(2.0, msg.ixx());
  EXPECT_DOUBLE_EQ(3.0, msg.iyy());
  EXPECT_DOUBLE_EQ(4.0, msg.izz());
  EXPECT_DOUBLE_EQ(0.1, msg.ixy());
  EXPECT_DOUBLE_EQ(0.2, msg.ixz());
  EXPECT_DOUBLE_EQ(0.3, msg.iyz());
  EXPECT_EQ(pose, msgs::ConvertIgn(msg.pose()));
}

TEST_F(MsgsTest, SetMassMatrix3)
{
  msgs::Inertial msg;
  msgs::Set(&msg, ignition::math::MassMatrix3d(
        12.0,
        ignition::math::Vector3d(2, 3, 4),
        ignition::math::Vector3d(0.1, 0.2, 0.3)));

  EXPECT_DOUBLE_EQ(12.0, msg.mass());
  EXPECT_DOUBLE_EQ(2.0, msg.ixx());
  EXPECT_DOUBLE_EQ(3.0, msg.iyy());
  EXPECT_DOUBLE_EQ(4.0, msg.izz());
  EXPECT_DOUBLE_EQ(0.1, msg.ixy());
  EXPECT_DOUBLE_EQ(0.2, msg.ixz());
  EXPECT_DOUBLE_EQ(0.3, msg.iyz());
}

TEST_F(MsgsTest, SetPlane)
{
  msgs::PlaneGeom msg;
  msgs::Set(&msg, ignition::math::Planed(
        ignition::math::Vector3d(0, 0, 1),
        ignition::math::Vector2d(123, 456), 1.0));

  EXPECT_DOUBLE_EQ(0, msg.normal().x());
  EXPECT_DOUBLE_EQ(0, msg.normal().y());
  EXPECT_DOUBLE_EQ(1, msg.normal().z());

  EXPECT_DOUBLE_EQ(123, msg.size().x());
  EXPECT_DOUBLE_EQ(456, msg.size().y());

  EXPECT_DOUBLE_EQ(1.0, msg.d());
}

TEST_F(MsgsTest, Initialization)
{
  {
    msgs::Vector3d msg;
    EXPECT_DOUBLE_EQ(0, msg.x());
    EXPECT_DOUBLE_EQ(0, msg.y());
    EXPECT_DOUBLE_EQ(0, msg.z());
  }

  {
    msgs::Wrench msg;
    EXPECT_DOUBLE_EQ(0, msg.force().x());
    EXPECT_DOUBLE_EQ(0, msg.force().y());
    EXPECT_DOUBLE_EQ(0, msg.force().z());
    EXPECT_DOUBLE_EQ(0, msg.torque().x());
    EXPECT_DOUBLE_EQ(0, msg.torque().y());
    EXPECT_DOUBLE_EQ(0, msg.torque().z());
  }
}

/////////////////////////////////////////////////
/// Helper function for GPSSensorFromSDF
void CheckGPSSensorMsg(const msgs::GPSSensor &_msg)
{
  EXPECT_EQ(_msg.position().horizontal_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.position().horizontal_noise().mean(), 0.1, 1e-4);
  EXPECT_NEAR(_msg.position().horizontal_noise().stddev(), 0.2, 1e-4);
  EXPECT_NEAR(_msg.position().horizontal_noise().bias_mean(), 0.3, 1e-4);
  EXPECT_NEAR(_msg.position().horizontal_noise().bias_stddev(), 0.4, 1e-4);
  EXPECT_NEAR(_msg.position().horizontal_noise().precision(), 0.5, 1e-4);

  EXPECT_EQ(_msg.position().vertical_noise().type(),
      msgs::SensorNoise::GAUSSIAN_QUANTIZED);
  EXPECT_NEAR(_msg.position().vertical_noise().mean(), 0.6, 1e-4);
  EXPECT_NEAR(_msg.position().vertical_noise().stddev(), 0.7, 1e-4);
  EXPECT_NEAR(_msg.position().vertical_noise().bias_mean(), 0.8, 1e-4);
  EXPECT_NEAR(_msg.position().vertical_noise().bias_stddev(), 0.9, 1e-4);
  EXPECT_NEAR(_msg.position().vertical_noise().precision(), 1.0, 1e-4);

  EXPECT_EQ(_msg.velocity().horizontal_noise().type(), msgs::SensorNoise::NONE);
  EXPECT_NEAR(_msg.velocity().horizontal_noise().mean(), 1.1, 1e-4);
  EXPECT_NEAR(_msg.velocity().horizontal_noise().stddev(), 1.2, 1e-4);
  EXPECT_NEAR(_msg.velocity().horizontal_noise().bias_mean(), 1.3, 1e-4);
  EXPECT_NEAR(_msg.velocity().horizontal_noise().bias_stddev(), 1.4, 1e-4);
  EXPECT_NEAR(_msg.velocity().horizontal_noise().precision(), 1.5, 1e-4);

  EXPECT_EQ(_msg.velocity().vertical_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.velocity().vertical_noise().mean(), 1.6, 1e-4);
  EXPECT_NEAR(_msg.velocity().vertical_noise().stddev(), 1.7, 1e-4);
  EXPECT_NEAR(_msg.velocity().vertical_noise().bias_mean(), 1.8, 1e-4);
  EXPECT_NEAR(_msg.velocity().vertical_noise().bias_stddev(), 1.9, 1e-4);
  EXPECT_NEAR(_msg.velocity().vertical_noise().precision(), 2.0, 1e-4);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, GPSSensorFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("sensor.sdf", sdf);

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <sensor name='gps' type='gps'>\
           <always_on>true</always_on>\
           <update_rate>15</update_rate>\
           <pose>1 2 3 0.1 0.2 0.3</pose>\
           <visualize>true</visualize>\
           <topic>/gazebo/test</topic>\
           <gps>\
             <position_sensing>\
               <horizontal>\
                 <noise type='gaussian'>\
                   <mean>0.1</mean>\
                   <stddev>0.2</stddev>\
                   <bias_mean>0.3</bias_mean>\
                   <bias_stddev>0.4</bias_stddev>\
                   <precision>0.5</precision>\
                 </noise>\
               </horizontal>\
               <vertical>\
                 <noise type='gaussian_quantized'>\
                   <mean>0.6</mean>\
                   <stddev>0.7</stddev>\
                   <bias_mean>0.8</bias_mean>\
                   <bias_stddev>0.9</bias_stddev>\
                   <precision>1.0</precision>\
                 </noise>\
               </vertical>\
             </position_sensing>\
             <velocity_sensing>\
               <horizontal>\
                 <noise type='none'>\
                   <mean>1.1</mean>\
                   <stddev>1.2</stddev>\
                   <bias_mean>1.3</bias_mean>\
                   <bias_stddev>1.4</bias_stddev>\
                   <precision>1.5</precision>\
                 </noise>\
               </horizontal>\
               <vertical>\
                 <noise type='gaussian'>\
                   <mean>1.6</mean>\
                   <stddev>1.7</stddev>\
                   <bias_mean>1.8</bias_mean>\
                   <bias_stddev>1.9</bias_stddev>\
                   <precision>2.0</precision>\
                 </noise>\
               </vertical>\
             </velocity_sensing>\
           </gps>\
         </sensor>\
       </sdf>", sdf));

  msgs::Sensor msg = msgs::SensorFromSDF(sdf);

  EXPECT_EQ(msg.name(), "gps");
  EXPECT_EQ(msg.type(), "gps");
  EXPECT_EQ(msg.topic(), "/gazebo/test");
  EXPECT_TRUE(msg.always_on());
  EXPECT_TRUE(msg.visualize());
  EXPECT_NEAR(msg.update_rate(), 15.0, 1e-4);
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_TRUE(msg.has_gps());
  EXPECT_FALSE(msg.has_camera());
  EXPECT_FALSE(msg.has_ray());

  CheckGPSSensorMsg(msg.gps());

  sdf::ElementPtr elem = msgs::GPSSensorToSDF(msg.gps());
  msgs::GPSSensor sensorMsg = msgs::GPSSensorFromSDF(elem);
  CheckGPSSensorMsg(sensorMsg);
}

/////////////////////////////////////////////////
/// Helper function for IMUSensorFromSDF
void CheckIMUSensorMsg(const msgs::IMUSensor &_msg)
{
  EXPECT_EQ(_msg.angular_velocity().x_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.angular_velocity().x_noise().mean(), 0, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().x_noise().stddev(), 0.0002, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().x_noise().bias_mean(), 7.5e-6, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().x_noise().bias_stddev(), 8e-7, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().x_noise().precision(), 0.0, 1e-4);

  EXPECT_EQ(_msg.angular_velocity().y_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.angular_velocity().y_noise().mean(), 0, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().y_noise().stddev(), 0.0002, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().y_noise().bias_mean(), 7.5e-6, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().y_noise().bias_stddev(), 8e-7, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().y_noise().precision(), 0.0, 1e-4);

  EXPECT_EQ(_msg.angular_velocity().z_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.angular_velocity().z_noise().mean(), 0, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().z_noise().stddev(), 0.0002, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().z_noise().bias_mean(), 7.5e-6, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().z_noise().bias_stddev(), 8e-7, 1e-4);
  EXPECT_NEAR(_msg.angular_velocity().z_noise().precision(), 0.0, 1e-4);

  EXPECT_EQ(_msg.linear_acceleration().x_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.linear_acceleration().x_noise().mean(), 0, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().x_noise().stddev(), 0.017, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().x_noise().bias_mean(), 0.1, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().x_noise().bias_stddev(), 0.001, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().x_noise().precision(), 0.0, 1e-4);

  EXPECT_EQ(_msg.linear_acceleration().y_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.linear_acceleration().y_noise().mean(), 0, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().y_noise().stddev(), 0.017, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().y_noise().bias_mean(), 0.1, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().y_noise().bias_stddev(), 0.001, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().y_noise().precision(), 0.0, 1e-4);

  EXPECT_EQ(_msg.linear_acceleration().z_noise().type(),
      msgs::SensorNoise::GAUSSIAN);
  EXPECT_NEAR(_msg.linear_acceleration().z_noise().mean(), 0, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().z_noise().stddev(), 0.017, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().z_noise().bias_mean(), 0.1, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().z_noise().bias_stddev(), 0.001, 1e-4);
  EXPECT_NEAR(_msg.linear_acceleration().z_noise().precision(), 0.0, 1e-4);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, IMUSensorFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("sensor.sdf", sdf);

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
        <sensor name='imu_sensor' type='imu'>\
          <update_rate>1000</update_rate>\
          <always_on>true</always_on>\
          <pose>1 2 3 0.1 0.2 0.3</pose>\
          <visualize>true</visualize>\
          <topic>/gazebo/test</topic>\
          <imu>\
            <angular_velocity>\
              <x>\
                <noise type='gaussian'>\
                 <mean>0</mean>\
                 <stddev>0.0002</stddev>\
                 <bias_mean>7.5e-06</bias_mean>\
                 <bias_stddev>8e-07</bias_stddev>\
                </noise>\
              </x>\
              <y>\
                <noise type='gaussian'>\
                 <mean>0</mean>\
                 <stddev>0.0002</stddev>\
                 <bias_mean>7.5e-06</bias_mean>\
                 <bias_stddev>8e-07</bias_stddev>\
                </noise>\
              </y>\
              <z>\
                <noise type='gaussian'>\
                 <mean>0</mean>\
                 <stddev>0.0002</stddev>\
                 <bias_mean>7.5e-06</bias_mean>\
                 <bias_stddev>8e-07</bias_stddev>\
                </noise>\
              </z>\
            </angular_velocity>\
            <linear_acceleration>\
              <x>\
                <noise type='gaussian'>\
                  <mean>0</mean>\
                  <stddev>0.017</stddev>\
                  <bias_mean>0.1</bias_mean>\
                  <bias_stddev>0.001</bias_stddev>\
                </noise>\
              </x>\
              <y>\
                <noise type='gaussian'>\
                  <mean>0</mean>\
                  <stddev>0.017</stddev>\
                  <bias_mean>0.1</bias_mean>\
                  <bias_stddev>0.001</bias_stddev>\
                </noise>\
              </y>\
              <z>\
                <noise type='gaussian'>\
                  <mean>0</mean>\
                  <stddev>0.017</stddev>\
                  <bias_mean>0.1</bias_mean>\
                  <bias_stddev>0.001</bias_stddev>\
                </noise>\
              </z>\
            </linear_acceleration>\
          </imu>\
         </sensor>\
       </sdf>", sdf));

  msgs::Sensor msg = msgs::SensorFromSDF(sdf);

  EXPECT_EQ(msg.name(), "imu_sensor");
  EXPECT_EQ(msg.type(), "imu");
  EXPECT_EQ(msg.topic(), "/gazebo/test");
  EXPECT_TRUE(msg.always_on());
  EXPECT_TRUE(msg.visualize());
  EXPECT_NEAR(msg.update_rate(), 1000.0, 1e-4);
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_TRUE(msg.has_imu());
  EXPECT_FALSE(msg.has_gps());
  EXPECT_FALSE(msg.has_camera());
  EXPECT_FALSE(msg.has_ray());

  CheckIMUSensorMsg(msg.imu());

  sdf::ElementPtr elem = msgs::IMUSensorToSDF(msg.imu());
  msgs::IMUSensor sensorMsg = msgs::IMUSensorFromSDF(elem);
  CheckIMUSensorMsg(sensorMsg);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, LogicalCameraSensorFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("sensor.sdf", sdf);

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <sensor name='camera' type='logical_camera'>\
           <always_on>true</always_on>\
           <update_rate>15</update_rate>\
           <pose>1 2 3 0.1 0.2 0.3</pose>\
           <visualize>true</visualize>\
           <topic>/gazebo/test</topic>\
           <logical_camera>\
             <near>0.1</near>\
             <far>100.2</far>\
             <aspect_ratio>1.43</aspect_ratio>\
             <horizontal_fov>1.23</horizontal_fov>\
           </logical_camera>\
         </sensor>\
       </sdf>", sdf));

  msgs::Sensor msg = msgs::SensorFromSDF(sdf);

  EXPECT_EQ(msg.name(), "camera");
  EXPECT_EQ(msg.type(), "logical_camera");
  EXPECT_EQ(msg.topic(), "/gazebo/test");
  EXPECT_TRUE(msg.always_on());
  EXPECT_TRUE(msg.visualize());
  EXPECT_NEAR(msg.update_rate(), 15.0, 1e-4);
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_TRUE(msg.has_logical_camera());
  EXPECT_FALSE(msg.has_camera());
  EXPECT_FALSE(msg.has_ray());
  EXPECT_FALSE(msg.has_contact());

  EXPECT_NEAR(msg.logical_camera().near_clip(), 0.1, 1e-4);
  EXPECT_NEAR(msg.logical_camera().far_clip(), 100.2, 1e-4);
  EXPECT_NEAR(msg.logical_camera().aspect_ratio(), 1.43, 1e-4);
  EXPECT_NEAR(msg.logical_camera().horizontal_fov(), 1.23, 1e-4);

  sdf::ElementPtr elem = msgs::LogicalCameraSensorToSDF(msg.logical_camera());
  msgs::LogicalCameraSensor sensorMsg = msgs::LogicalCameraSensorFromSDF(elem);

  EXPECT_NEAR(sensorMsg.near_clip(), 0.1, 1e-4);
  EXPECT_NEAR(sensorMsg.far_clip(), 100.2, 1e-4);
  EXPECT_NEAR(sensorMsg.aspect_ratio(), 1.43, 1e-4);
  EXPECT_NEAR(sensorMsg.horizontal_fov(), 1.23, 1e-4);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, CameraSensorFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("sensor.sdf", sdf);

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <sensor name='camera' type='camera'>\
           <always_on>true</always_on>\
           <update_rate>15</update_rate>\
           <pose>1 2 3 0.1 0.2 0.3</pose>\
           <visualize>true</visualize>\
           <topic>/gazebo/test</topic>\
           <camera>\
             <horizontal_fov>0.123</horizontal_fov>\
             <image>\
               <width>320</width>\
               <height>240</height>\
               <format>R8G8B8</format>\
             </image>\
             <clip>\
               <near>0.1</near>\
               <far>10.5</far>\
             </clip>\
             <save enabled='true'>\
               <path>/tmp</path>\
             </save>\
             <distortion>\
               <k1>0.1</k1>\
               <k2>0.2</k2>\
               <k3>0.3</k3>\
               <p1>0.4</p1>\
               <p2>0.5</p2>\
               <center>10 20</center>\
             </distortion>\
           </camera>\
         </sensor>\
       </sdf>", sdf));

  msgs::Sensor msg = msgs::SensorFromSDF(sdf);

  EXPECT_EQ(msg.name(), "camera");
  EXPECT_EQ(msg.type(), "camera");
  EXPECT_EQ(msg.topic(), "/gazebo/test");
  EXPECT_TRUE(msg.always_on());
  EXPECT_TRUE(msg.visualize());
  EXPECT_NEAR(msg.update_rate(), 15.0, 1e-4);
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_TRUE(msg.has_camera());
  EXPECT_FALSE(msg.has_ray());
  EXPECT_FALSE(msg.has_contact());

  EXPECT_NEAR(msg.camera().horizontal_fov(), 0.123, 1e-4);
  EXPECT_NEAR(msg.camera().image_size().x(), 320, 1e-4);
  EXPECT_NEAR(msg.camera().image_size().y(), 240, 1e-4);
  EXPECT_NEAR(msg.camera().near_clip(), 0.1, 1e-4);
  EXPECT_NEAR(msg.camera().far_clip(), 10.5, 1e-4);
  EXPECT_TRUE(msg.camera().save_enabled());
  EXPECT_EQ(msg.camera().save_path(), "/tmp");

  EXPECT_NEAR(msg.camera().distortion().k1(), 0.1, 1e-4);
  EXPECT_NEAR(msg.camera().distortion().k2(), 0.2, 1e-4);
  EXPECT_NEAR(msg.camera().distortion().k3(), 0.3, 1e-4);
  EXPECT_NEAR(msg.camera().distortion().p1(), 0.4, 1e-4);
  EXPECT_NEAR(msg.camera().distortion().p2(), 0.5, 1e-4);
  EXPECT_NEAR(msg.camera().distortion().center().x(), 10, 1e-4);
  EXPECT_NEAR(msg.camera().distortion().center().y(), 20, 1e-4);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, ContactSensorFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("sensor.sdf", sdf);

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <sensor name='contact' type='contact'>\
           <always_on>false</always_on>\
           <update_rate>5</update_rate>\
           <pose>1 2 3 0.1 0.2 0.3</pose>\
           <visualize>false</visualize>\
           <topic>/test</topic>\
           <contact>\
             <collision>my_collision</collision>\
           </contact>\
         </sensor>\
       </sdf>", sdf));

  msgs::Sensor msg = msgs::SensorFromSDF(sdf);

  EXPECT_EQ(msg.name(), "contact");
  EXPECT_EQ(msg.type(), "contact");
  EXPECT_EQ(msg.topic(), "/test");
  EXPECT_FALSE(msg.always_on());
  EXPECT_FALSE(msg.visualize());
  EXPECT_NEAR(msg.update_rate(), 5.0, 1e-4);
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_FALSE(msg.has_camera());
  EXPECT_FALSE(msg.has_ray());
  EXPECT_TRUE(msg.has_contact());

  EXPECT_EQ(msg.contact().collision_name(), "my_collision");
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, RaySensorFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("sensor.sdf", sdf);

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <sensor name='ray' type='ray'>\
           <always_on>false</always_on>\
           <update_rate>5</update_rate>\
           <pose>1 2 3 0.1 0.2 0.3</pose>\
           <visualize>false</visualize>\
           <topic>/test</topic>\
           <ray>\
             <scan>\
               <horizontal>\
                 <samples>10</samples>\
                 <resolution>0.2</resolution>\
                 <min_angle>0</min_angle>\
                 <max_angle>1</max_angle>\
               </horizontal>\
               <vertical>\
                 <samples>20</samples>\
                 <resolution>0.4</resolution>\
                 <min_angle>-1</min_angle>\
                 <max_angle>0</max_angle>\
               </vertical>\
             </scan>\
             <range>\
               <min>0</min>\
               <max>10</max>\
               <resolution>0.1</resolution>\
             </range>\
           </ray>\
         </sensor>\
       </sdf>", sdf));

  msgs::Sensor msg = msgs::SensorFromSDF(sdf);

  EXPECT_EQ(msg.name(), "ray");
  EXPECT_EQ(msg.type(), "ray");
  EXPECT_EQ(msg.topic(), "/test");
  EXPECT_FALSE(msg.always_on());
  EXPECT_FALSE(msg.visualize());
  EXPECT_NEAR(msg.update_rate(), 5.0, 1e-4);
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0.1, 0.2, 0.3));

  EXPECT_FALSE(msg.has_camera());
  EXPECT_TRUE(msg.has_ray());
  EXPECT_FALSE(msg.has_contact());

  EXPECT_EQ(msg.ray().horizontal_samples(), 10);
  EXPECT_NEAR(msg.ray().horizontal_resolution(), 0.2, 1e-4);
  EXPECT_NEAR(msg.ray().horizontal_min_angle(), 0, 1e-4);
  EXPECT_NEAR(msg.ray().horizontal_max_angle(), 1, 1e-4);

  EXPECT_EQ(msg.ray().vertical_samples(), 20);
  EXPECT_NEAR(msg.ray().vertical_resolution(), 0.4, 1e-4);
  EXPECT_NEAR(msg.ray().vertical_min_angle(), -1, 1e-4);
  EXPECT_NEAR(msg.ray().vertical_max_angle(), 0, 1e-4);

  EXPECT_NEAR(msg.ray().range_min(), 0, 1e-4);
  EXPECT_NEAR(msg.ray().range_max(), 10, 1e-4);
  EXPECT_NEAR(msg.ray().range_resolution(), 0.1, 1e-4);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, GUIFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
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
       </sdf>", sdf));
  msgs::GUI msg = msgs::GUIFromSDF(sdf);
}

TEST_F(MsgsTest, GUIFromSDF_EmptyTrackVisual)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
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
       </sdf>", sdf));
  msgs::GUI msg = msgs::GUIFromSDF(sdf);
}

TEST_F(MsgsTest, GUIFromSDF_WithEmptyCamera)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <gui fullscreen='true'>\
         <camera name='camera'>\
         </camera>\
         </gui>\
       </sdf>", sdf));
  msgs::GUI msg = msgs::GUIFromSDF(sdf);
}

TEST_F(MsgsTest, GUIFromSDF_WithoutCamera)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <gui fullscreen='true'>\
         </gui>\
       </sdf>", sdf));
  msgs::GUI msg = msgs::GUIFromSDF(sdf);
}

TEST_F(MsgsTest, GUIFromSDF_WithPlugin)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <gui fullscreen='true'>\
           <plugin name='plugin_name' filename='plugin_filename'>\
           </plugin>\
         </gui>\
       </sdf>", sdf));
  msgs::GUI msg = msgs::GUIFromSDF(sdf);
}

TEST_F(MsgsTest, PluginFromSDF)
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
  msgs::Plugin msg = msgs::PluginFromSDF(sdf);

  EXPECT_TRUE(msg.has_name());
  EXPECT_EQ(msg.name(), "plugin_name");
  EXPECT_TRUE(msg.has_filename());
  EXPECT_EQ(msg.filename(), "plugin_filename");
  EXPECT_TRUE(msg.has_innerxml());
  EXPECT_EQ(msg.innerxml(), "<param1>1</param1>\n<param2>1</param2>\n");
}

TEST_F(MsgsTest, LightFromSDF_ListDirectional)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("light.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <light type='directional' name='sun'>\
           <cast_shadows>true</cast_shadows>\
           <pose>0 0 10 0 0 0</pose>\
           <diffuse>0.8 0.8 0.8 1</diffuse>\
           <specular>0 0 0 1</specular>\
           <attenuation>\
             <range>20</range>\
             <constant>0.8</constant>\
             <linear>0.01</linear>\
             <quadratic>0.0</quadratic>\
           </attenuation>\
           <direction>1.0 1.0 -1.0</direction>\
         </light>\
       </sdf>", sdf));
  msgs::Light msg = msgs::LightFromSDF(sdf);
}

TEST_F(MsgsTest, LightFromSDF_LightSpot)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("light.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
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
       </sdf>", sdf));
  msgs::Light msg = msgs::LightFromSDF(sdf);
}

TEST_F(MsgsTest, LightFromSDF_LightPoint)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("light.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
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
       </sdf>", sdf));
  msgs::Light msg = msgs::LightFromSDF(sdf);
}

TEST_F(MsgsTest, LightFromSDF_LighBadType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("light.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <light type='_bad_' name='lamp'>\
         </light>\
       </sdf>", sdf));
  msgs::Light msg = msgs::LightFromSDF(sdf);
}

// Plane visual
TEST_F(MsgsTest, VisualFromSDF_PlaneVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_BoxVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_SphereVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_CylinderVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_MeshVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_ImageVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_HeigthmapVisual)
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
  msgs::Visual msg = msgs::VisualFromSDF(sdf);
}

TEST_F(MsgsTest, VisualFromSDF_NoGeometry)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
         </visual>\
      </sdf>", sdf));
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
      common::Exception);
}

TEST_F(MsgsTest, VisualFromSDF_ShaderTypeThrow)
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
             <shader type='throw'/>\
           </material>\
         </visual>\
      </sdf>", sdf));
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
      common::Exception);
}

TEST_F(MsgsTest, VisualFromSDF_BadGeometryVisual)
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
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
               common::Exception);
}

TEST_F(MsgsTest, VisualFromSDF_BadGeometryType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("visual.sdf", sdf);
  // As of sdformat pull request 148 (released in version 2.3.1),
  // unknown elements are now ignored with a warning message
  // rather than causing an error.
  EXPECT_TRUE(sdf::readString(
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
      </sdf>", sdf));

  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='visual'>\
         </visual>\
      </sdf>", sdf));

  sdf::ElementPtr badElement(new sdf::Element());
  badElement->SetName("bad_type");
  sdf->GetElement("geometry")->InsertElement(badElement);
  EXPECT_THROW(msgs::Visual msg = msgs::VisualFromSDF(sdf),
      common::Exception);
}

TEST_F(MsgsTest, VisualFromSDF_BadFogType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("scene.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>true</shadows>\
           <fog>\
             <color>1 1 1 1</color>\
             <type>throw</type>\
             <start>0</start>\
             <end>10</end>\
             <density>1</density>\
           </fog>\
           <grid>false</grid>\
         </scene>\
      </sdf>", sdf));
  EXPECT_THROW(msgs::Scene msg = msgs::SceneFromSDF(sdf), common::Exception);
}

TEST_F(MsgsTest, VisualSceneFromSDF_A)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("scene.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>true</shadows>\
           <fog>\
             <color>1 1 1 1</color>\
             <type>linear</type>\
             <start>0</start>\
             <end>10</end>\
             <density>1</density>\
           </fog>\
           <grid>false</grid>\
         </scene>\
      </sdf>", sdf));
  msgs::Scene msg = msgs::SceneFromSDF(sdf);
}

TEST_F(MsgsTest, VisualSceneFromSDF_B)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("scene.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>false</shadows>\
           <fog>\
             <color>1 1 1 1</color>\
             <type>exp</type>\
             <start>0</start>\
             <end>10</end>\
             <density>1</density>\
           </fog>\
         </scene>\
      </sdf>", sdf));
  msgs::Scene msg = msgs::SceneFromSDF(sdf);
}

TEST_F(MsgsTest, VisualSceneFromSDF_C)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("scene.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <scene>\
           <ambient>0.1 0.1 0.1 1</ambient>\
           <background>0 0 0 1</background>\
           <shadows>false</shadows>\
           <fog>\
             <color>1 1 1 1</color>\
             <type>exp2</type>\
             <start>0</start>\
             <end>10</end>\
             <density>1</density>\
           </fog>\
           <grid>true</grid>\
         </scene>\
      </sdf>", sdf));
  msgs::Scene msg = msgs::SceneFromSDF(sdf);
}

TEST_F(MsgsTest, VisualSceneFromSDF_CEmpty)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("scene.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <scene>\
         </scene>\
      </sdf>", sdf));
  msgs::Scene msg = msgs::SceneFromSDF(sdf);
}

TEST_F(MsgsTest, VisualSceneFromSDF_CEmptyNoSky)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("scene.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <scene>\
           <background>0 0 0 1</background>\
         </scene>\
      </sdf>", sdf));
  msgs::Scene msg = msgs::SceneFromSDF(sdf);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, MeshFromSDF)
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

  msgs::MeshGeom msg = msgs::MeshFromSDF(sdf->GetElement("mesh"));
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
TEST_F(MsgsTest, CollisionFromSDF_Friction)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("collision.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
        <collision name='collision_name'>\
          <geometry>\
            <cylinder>\
              <radius>1.23</radius>\
              <length>4.56</length>\
            </cylinder>\
          </geometry>\
          <surface>\
            <friction>\
              <ode>\
                <mu>0.1</mu>\
                <mu2>0.2</mu2>\
                <fdir1>0.3 0.4 0.5</fdir1>\
                <slip1>0.6</slip1>\
                <slip2>0.7</slip2>\
              </ode>\
              <torsional>\
                <coefficient>0.8</coefficient>\
                <patch_radius>0.9</patch_radius>\
                <surface_radius>1.0</surface_radius>\
                <use_patch_radius>0</use_patch_radius>\
                <ode>\
                  <slip>1.1</slip>\
                </ode>\
              </torsional>\
            </friction>\
          </surface>\
        </collision>\
      </sdf>", sdf));

  msgs::Collision msg = msgs::CollisionFromSDF(sdf);

  // Translational friction
  EXPECT_TRUE(msg.has_surface());
  EXPECT_TRUE(msg.surface().has_friction());

  msgs::Friction friction = msg.surface().friction();

  EXPECT_TRUE(friction.has_mu());
  EXPECT_DOUBLE_EQ(0.1, friction.mu());

  EXPECT_TRUE(friction.has_mu2());
  EXPECT_DOUBLE_EQ(0.2, friction.mu2());

  EXPECT_TRUE(friction.has_fdir1());
  EXPECT_EQ(ignition::math::Vector3d(0.3, 0.4, 0.5),
      msgs::ConvertIgn(friction.fdir1()));

  EXPECT_TRUE(friction.has_slip1());
  EXPECT_DOUBLE_EQ(0.6, friction.slip1());

  EXPECT_TRUE(friction.has_slip2());
  EXPECT_DOUBLE_EQ(0.7, friction.slip2());

  // Torsional friction
  EXPECT_TRUE(friction.has_torsional());
  msgs::Friction::Torsional torsional = friction.torsional();

  EXPECT_TRUE(torsional.has_coefficient());
  EXPECT_DOUBLE_EQ(0.8, torsional.coefficient());

  EXPECT_TRUE(torsional.has_patch_radius());
  EXPECT_DOUBLE_EQ(0.9, torsional.patch_radius());

  EXPECT_TRUE(torsional.has_surface_radius());
  EXPECT_DOUBLE_EQ(1.0, torsional.surface_radius());

  EXPECT_TRUE(torsional.has_patch_radius());
  EXPECT_FALSE(torsional.use_patch_radius());

  EXPECT_TRUE(torsional.has_ode());
  EXPECT_TRUE(torsional.ode().has_slip());
  EXPECT_DOUBLE_EQ(1.1, torsional.ode().slip());
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, CollisionFromSDF_Contact)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("collision.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
        <collision name='collision_name'>\
          <geometry>\
            <cylinder>\
              <radius>1.23</radius>\
              <length>4.56</length>\
            </cylinder>\
          </geometry>\
          <laser_retro>0.7</laser_retro>\
          <max_contacts>8</max_contacts>\
          <surface>\
            <bounce>\
              <restitution_coefficient>0.9</restitution_coefficient>\
              <threshold>1.0</threshold>\
            </bounce>\
            <contact>\
              <collide_without_contact>1</collide_without_contact>\
              <collide_without_contact_bitmask>\
                2\
              </collide_without_contact_bitmask>\
              <collide_bitmask>3</collide_bitmask>\
              <ode>\
                <soft_cfm>0.4</soft_cfm>\
                <soft_erp>0.5</soft_erp>\
                <kp>0.6</kp>\
                <kd>0.7</kd>\
                <max_vel>0.8</max_vel>\
                <min_depth>0.9</min_depth>\
              </ode>\
            </contact>\
          </surface>\
        </collision>\
      </sdf>", sdf));

  msgs::Collision msg = msgs::CollisionFromSDF(sdf);

  EXPECT_TRUE(msg.has_surface());
  msgs::Surface surface = msg.surface();

  EXPECT_TRUE(surface.has_restitution_coefficient());
  EXPECT_DOUBLE_EQ(0.9, surface.restitution_coefficient());

  EXPECT_TRUE(surface.has_bounce_threshold());
  EXPECT_DOUBLE_EQ(1.0, surface.bounce_threshold());

  EXPECT_TRUE(surface.has_soft_cfm());
  EXPECT_DOUBLE_EQ(0.4, surface.soft_cfm());

  EXPECT_TRUE(surface.has_soft_erp());
  EXPECT_DOUBLE_EQ(0.5, surface.soft_erp());

  EXPECT_TRUE(surface.has_kp());
  EXPECT_DOUBLE_EQ(0.6, surface.kp());

  EXPECT_TRUE(surface.has_kd());
  EXPECT_DOUBLE_EQ(0.7, surface.kd());

  EXPECT_TRUE(surface.has_max_vel());
  EXPECT_DOUBLE_EQ(0.8, surface.max_vel());

  EXPECT_TRUE(surface.has_min_depth());
  EXPECT_DOUBLE_EQ(0.9, surface.min_depth());

  EXPECT_TRUE(surface.has_collide_without_contact());
  EXPECT_TRUE(surface.collide_without_contact());

  EXPECT_TRUE(surface.has_collide_without_contact_bitmask());
  EXPECT_EQ(2u, surface.collide_without_contact_bitmask());

  EXPECT_TRUE(surface.has_collide_bitmask());
  EXPECT_EQ(3u, surface.collide_bitmask());
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, SurfaceToFromSDF)
{
  // Surface
  double restitution_coefficient = 0.1;
  double threshold = 0.2;


  // Friction
  double mu = 0.1;
  double mu2 = 0.2;
  ignition::math::Vector3d fdir1(-0.3, 0.4, -0.5);
  double slip1 = 0.6;
  double slip2 = 0.7;
  double mu3 = 0.8;
  double patch_radius = 0.9;
  double surface_radius = 1.0;
  bool use_patch_radius = false;
  double slip3 = 1.1;

  // Contact
  bool collide_without_contact = true;
  int32_t collide_without_contact_bitmask = 123;
  int32_t collide_bitmask = 456;
  double soft_cfm = 0.1;
  double soft_erp = 0.2;
  double kp = 0.3;
  double kd = 0.4;
  double max_vel = 0.5;
  double min_depth = 0.6;

  // Create message
  msgs::Surface surfaceMsg1;
  surfaceMsg1.set_restitution_coefficient(restitution_coefficient);
  surfaceMsg1.set_bounce_threshold(threshold);
  surfaceMsg1.set_soft_cfm(soft_cfm);
  surfaceMsg1.set_soft_erp(soft_erp);
  surfaceMsg1.set_kp(kp);
  surfaceMsg1.set_kd(kd);
  surfaceMsg1.set_max_vel(max_vel);
  surfaceMsg1.set_min_depth(min_depth);
  surfaceMsg1.set_collide_without_contact(collide_without_contact);
  surfaceMsg1.set_collide_without_contact_bitmask(
      collide_without_contact_bitmask);
  surfaceMsg1.set_collide_bitmask(collide_bitmask);

  msgs::Friction frictionMsg1;
  frictionMsg1.set_mu(mu);
  frictionMsg1.set_mu2(mu2);
  msgs::Set(frictionMsg1.mutable_fdir1(), fdir1);
  frictionMsg1.set_slip1(slip1);
  frictionMsg1.set_slip2(slip2);

  msgs::Friction::Torsional torsionalMsg1;
  torsionalMsg1.set_coefficient(mu3);
  torsionalMsg1.set_patch_radius(patch_radius);
  torsionalMsg1.set_surface_radius(surface_radius);
  torsionalMsg1.set_use_patch_radius(use_patch_radius);

  msgs::Friction::Torsional::ODE torsionalOdeMsg1;
  torsionalOdeMsg1.set_slip(slip3);

  auto torsionalODE = torsionalMsg1.mutable_ode();
  torsionalODE->CopyFrom(torsionalOdeMsg1);

  auto torsional = frictionMsg1.mutable_torsional();
  torsional->CopyFrom(torsionalMsg1);

  auto friction = surfaceMsg1.mutable_friction();
  friction->CopyFrom(frictionMsg1);

  // To SDF
  sdf::ElementPtr sdf1 = msgs::SurfaceToSDF(surfaceMsg1);

  // Back to Msg
  msgs::Surface surfaceMsg2 = msgs::SurfaceFromSDF(sdf1);
  EXPECT_DOUBLE_EQ(surfaceMsg2.restitution_coefficient(),
      restitution_coefficient);
  EXPECT_DOUBLE_EQ(surfaceMsg2.bounce_threshold(), threshold);
  EXPECT_DOUBLE_EQ(surfaceMsg2.soft_cfm(), soft_cfm);
  EXPECT_DOUBLE_EQ(surfaceMsg2.soft_erp(), soft_erp);
  EXPECT_DOUBLE_EQ(surfaceMsg2.kp(), kp);
  EXPECT_DOUBLE_EQ(surfaceMsg2.kd(), kd);
  EXPECT_DOUBLE_EQ(surfaceMsg2.max_vel(), max_vel);
  EXPECT_DOUBLE_EQ(surfaceMsg2.min_depth(), min_depth);
  EXPECT_EQ(surfaceMsg2.collide_without_contact(), collide_without_contact);
  EXPECT_EQ(surfaceMsg2.collide_without_contact_bitmask(),
      static_cast<unsigned int>(collide_without_contact_bitmask));
  EXPECT_EQ(surfaceMsg2.collide_bitmask(),
      static_cast<unsigned int>(collide_bitmask));

  ASSERT_TRUE(surfaceMsg2.has_friction());
  msgs::Friction frictionMsg2 = surfaceMsg2.friction();

  EXPECT_DOUBLE_EQ(frictionMsg2.mu(), mu);
  EXPECT_DOUBLE_EQ(frictionMsg2.mu2(), mu2);
  EXPECT_EQ(msgs::ConvertIgn(frictionMsg2.fdir1()), fdir1);
  EXPECT_DOUBLE_EQ(frictionMsg2.slip1(), slip1);
  EXPECT_DOUBLE_EQ(frictionMsg2.slip2(), slip2);
  ASSERT_TRUE(frictionMsg2.has_torsional());
  EXPECT_DOUBLE_EQ(frictionMsg2.torsional().coefficient(), mu3);
  EXPECT_DOUBLE_EQ(frictionMsg2.torsional().patch_radius(), patch_radius);
  EXPECT_DOUBLE_EQ(frictionMsg2.torsional().surface_radius(), surface_radius);
  EXPECT_EQ(frictionMsg2.torsional().use_patch_radius(), use_patch_radius);
  ASSERT_TRUE(frictionMsg2.torsional().has_ode());
  EXPECT_DOUBLE_EQ(frictionMsg2.torsional().ode().slip(), slip3);

  // Back to SDF
  sdf::ElementPtr sdf2;
  sdf2.reset(new sdf::Element);
  sdf::initFile("surface.sdf", sdf2);
  msgs::SurfaceToSDF(surfaceMsg2, sdf2);

  ASSERT_TRUE(sdf2 != nullptr);

  ASSERT_TRUE(sdf2->HasElement("bounce"));
  EXPECT_DOUBLE_EQ(
      sdf2->GetElement("bounce")->Get<double>("restitution_coefficient"),
      restitution_coefficient);
  EXPECT_DOUBLE_EQ(
      sdf2->GetElement("bounce")->Get<double>("threshold"), threshold);

  ASSERT_TRUE(sdf2->HasElement("contact"));
  sdf::ElementPtr contactSdf2 = sdf2->GetElement("contact");

  EXPECT_EQ(contactSdf2->Get<bool>("collide_without_contact"),
      collide_without_contact);
  EXPECT_EQ(contactSdf2->Get<int>("collide_without_contact_bitmask"),
      collide_without_contact_bitmask);
  EXPECT_EQ(contactSdf2->Get<int>("collide_bitmask"), collide_bitmask);

  ASSERT_TRUE(sdf2->HasElement("friction"));
  ASSERT_TRUE(sdf2->GetElement("friction")->HasElement("ode"));
  sdf::ElementPtr odeSdf2 =
      sdf2->GetElement("friction")->GetElement("ode");

  EXPECT_DOUBLE_EQ(odeSdf2->Get<double>("mu"), mu);
  EXPECT_DOUBLE_EQ(odeSdf2->Get<double>("mu2"), mu2);
  EXPECT_EQ(odeSdf2->Get<ignition::math::Vector3d>("fdir1"), fdir1);
  EXPECT_DOUBLE_EQ(odeSdf2->Get<double>("slip1"), slip1);
  EXPECT_DOUBLE_EQ(odeSdf2->Get<double>("slip2"), slip2);

  ASSERT_TRUE(sdf2->GetElement("friction")->HasElement("torsional"));
  sdf::ElementPtr torsionalSdf2 =
      sdf2->GetElement("friction")->GetElement("torsional");

  EXPECT_DOUBLE_EQ(torsionalSdf2->Get<double>("coefficient"), mu3);
  EXPECT_DOUBLE_EQ(torsionalSdf2->Get<double>("patch_radius"), patch_radius);
  EXPECT_DOUBLE_EQ(torsionalSdf2->Get<double>("surface_radius"),
      surface_radius);
  EXPECT_EQ(torsionalSdf2->Get<bool>("use_patch_radius"),
      use_patch_radius);
  ASSERT_TRUE(torsionalSdf2->HasElement("ode"));
  EXPECT_DOUBLE_EQ(torsionalSdf2->GetElement("ode")->Get<double>("slip"),
      slip3);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, AxisFromSDF)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("joint.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <joint name='arm' type='revolute'>\
           <parent>arm_base</parent>\
           <child>arm_shoulder</child>\
           <axis>\
             <xyz>0 0 1</xyz>\
             <limit>\
               <lower>0.01</lower>\
               <upper>9</upper>\
               <effort>2.2</effort>\
               <velocity>0.1</velocity>\
             </limit>\
             <dynamics>\
               <damping>0.1</damping>\
               <friction>0.2</friction>\
             </dynamics>\
           </axis>\
         </joint>\
      </sdf>", sdf));
  msgs::Axis msg = msgs::AxisFromSDF(sdf->GetElement("axis"));

  EXPECT_TRUE(msg.has_xyz());
  EXPECT_EQ(msgs::ConvertIgn(msg.xyz()), ignition::math::Vector3d(0, 0, 1));
  EXPECT_TRUE(msg.has_limit_lower());
  EXPECT_NEAR(msg.limit_lower(), 0.01, 1e-6);
  EXPECT_TRUE(msg.has_limit_upper());
  EXPECT_NEAR(msg.limit_upper(), 9, 1e-6);
  EXPECT_TRUE(msg.has_limit_effort());
  EXPECT_NEAR(msg.limit_effort(), 2.2, 1e-6);
  EXPECT_TRUE(msg.has_limit_velocity());
  EXPECT_NEAR(msg.limit_velocity(), 0.1, 1e-6);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  EXPECT_TRUE(msg.has_use_parent_model_frame());
  EXPECT_EQ(msg.use_parent_model_frame(), false);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  EXPECT_TRUE(msg.has_damping());
  EXPECT_NEAR(msg.damping(), 0.1, 1e-6);
  EXPECT_TRUE(msg.has_friction());
  EXPECT_NEAR(msg.friction(), 0.2, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, JointFromSDF)
{
  // revolute
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("joint.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <joint name='arm' type='revolute'>\
           <pose>1 2 3 0 1.57 0</pose>\
           <parent>arm_base</parent>\
           <child>arm_shoulder</child>\
           <axis>\
             <xyz expressed_in='__model__'>1 0 0</xyz>\
             <limit>\
               <lower>0.1</lower>\
               <upper>3.14</upper>\
               <effort>2.4</effort>\
               <velocity>0.4</velocity>\
             </limit>\
             <dynamics>\
               <damping>1.0</damping>\
               <friction>0.1</friction>\
             </dynamics>\
           </axis>\
           <physics>\
             <ode>\
               <cfm>0.2</cfm>\
               <bounce>0.1</bounce>\
               <velocity>1.1</velocity>\
               <fudge_factor>0.4</fudge_factor>\
               <limit>\
                 <cfm>0.0</cfm>\
                 <erp>0.9</erp>\
               </limit>\
               <suspension>\
                 <cfm>0.1</cfm>\
                 <erp>0.3</erp>\
               </suspension>\
             </ode>\
           </physics>\
         </joint>\
      </sdf>", sdf));
  msgs::Joint msg = msgs::JointFromSDF(sdf);

  EXPECT_TRUE(msg.has_name());
  EXPECT_EQ(msg.name(), "arm");
  EXPECT_TRUE(msg.has_type());
  EXPECT_EQ(msgs::ConvertJointType(msg.type()), "revolute");
  EXPECT_TRUE(msg.has_pose());
  EXPECT_EQ(msgs::ConvertIgn(msg.pose()),
      ignition::math::Pose3d(1, 2, 3, 0, 1.57, 0));
  EXPECT_TRUE(msg.has_parent());
  EXPECT_EQ(msg.parent(), "arm_base");
  EXPECT_TRUE(msg.has_child());
  EXPECT_EQ(msg.child(), "arm_shoulder");
  EXPECT_TRUE(msg.has_cfm());
  EXPECT_NEAR(msg.cfm(), 0.2, 1e-6);
  EXPECT_TRUE(msg.has_bounce());
  EXPECT_NEAR(msg.bounce(), 0.1, 1e-6);
  EXPECT_TRUE(msg.has_velocity());
  EXPECT_NEAR(msg.velocity(), 1.1, 1e-6);
  EXPECT_TRUE(msg.has_fudge_factor());
  EXPECT_NEAR(msg.fudge_factor(), 0.4, 1e-6);
  EXPECT_TRUE(msg.has_limit_cfm());
  EXPECT_NEAR(msg.limit_cfm(), 0.0, 1e-6);
  EXPECT_TRUE(msg.has_limit_erp());
  EXPECT_NEAR(msg.limit_erp(), 0.9, 1e-6);
  EXPECT_TRUE(msg.has_suspension_cfm());
  EXPECT_NEAR(msg.suspension_cfm(), 0.1, 1e-6);
  EXPECT_TRUE(msg.has_suspension_erp());
  EXPECT_NEAR(msg.suspension_erp(), 0.3, 1e-6);

  EXPECT_TRUE(msg.has_axis1());
  EXPECT_TRUE(!msg.has_axis2());
  const msgs::Axis axisMsg = msg.axis1();
  EXPECT_TRUE(axisMsg.has_xyz());
  EXPECT_EQ(msgs::ConvertIgn(axisMsg.xyz()), ignition::math::Vector3d(1, 0, 0));
  EXPECT_TRUE(axisMsg.has_limit_lower());
  EXPECT_NEAR(axisMsg.limit_lower(), 0.1, 1e-6);
  EXPECT_TRUE(axisMsg.has_limit_upper());
  EXPECT_NEAR(axisMsg.limit_upper(), 3.14, 1e-6);
  EXPECT_TRUE(axisMsg.has_limit_effort());
  EXPECT_NEAR(axisMsg.limit_effort(), 2.4, 1e-6);
  EXPECT_TRUE(axisMsg.has_limit_velocity());
  EXPECT_NEAR(axisMsg.limit_velocity(), 0.4, 1e-6);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  EXPECT_TRUE(axisMsg.has_use_parent_model_frame());
  EXPECT_EQ(axisMsg.use_parent_model_frame(), false);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  EXPECT_TRUE(axisMsg.has_xyz_expressed_in());
  EXPECT_EQ(axisMsg.xyz_expressed_in(), "__model__");
  EXPECT_TRUE(axisMsg.has_damping());
  EXPECT_NEAR(axisMsg.damping(), 1.0, 1e-6);
  EXPECT_TRUE(axisMsg.has_friction());
  EXPECT_NEAR(axisMsg.friction(), 0.1, 1e-6);

  // gearbox
  sdf::ElementPtr gearboxSdf(new sdf::Element());
  sdf::initFile("joint.sdf", gearboxSdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <joint name='axle_wheel' type='gearbox'>\
           <pose>5 2 1 1.57 0 0</pose>\
           <parent>axle</parent>\
           <child>wheel</child>\
           <axis>\
             <xyz expressed_in='__model__'>0 0 1</xyz>\
             <limit>\
               <lower>0.01</lower>\
               <upper>3.0</upper>\
               <effort>2.1</effort>\
               <velocity>0.2</velocity>\
             </limit>\
             <dynamics>\
               <damping>0.8</damping>\
               <friction>0.1</friction>\
             </dynamics>\
           </axis>\
           <axis2>\
             <xyz expressed_in='__model__'>0 0 1</xyz>\
             <limit>\
               <lower>0.02</lower>\
               <upper>3.01</upper>\
               <effort>2.3</effort>\
               <velocity>0.1</velocity>\
             </limit>\
             <dynamics>\
               <damping>0.9</damping>\
               <friction>0.1</friction>\
             </dynamics>\
           </axis2>\
           <gearbox_reference_body>chassis</gearbox_reference_body>\
           <gearbox_ratio>0.2</gearbox_ratio>\
           <physics>\
             <ode>\
               <cfm>0.3</cfm>\
               <bounce>0.12</bounce>\
               <velocity>1.12</velocity>\
               <fudge_factor>0.4</fudge_factor>\
               <limit>\
                 <cfm>0.1</cfm>\
                 <erp>0.9</erp>\
               </limit>\
               <suspension>\
                 <cfm>0.2</cfm>\
                 <erp>0.4</erp>\
               </suspension>\
             </ode>\
           </physics>\
         </joint>\
      </sdf>", gearboxSdf));
  msgs::Joint gearboxMsg = msgs::JointFromSDF(gearboxSdf);

  EXPECT_TRUE(gearboxMsg.has_name());
  EXPECT_EQ(gearboxMsg.name(), "axle_wheel");
  EXPECT_TRUE(gearboxMsg.has_type());
  EXPECT_EQ(msgs::ConvertJointType(gearboxMsg.type()), "gearbox");
  EXPECT_TRUE(gearboxMsg.has_pose());
  EXPECT_EQ(msgs::ConvertIgn(gearboxMsg.pose()),
      ignition::math::Pose3d(5, 2, 1, 1.57, 0, 0));
  EXPECT_TRUE(gearboxMsg.has_parent());
  EXPECT_EQ(gearboxMsg.parent(), "axle");
  EXPECT_TRUE(gearboxMsg.has_child());
  EXPECT_EQ(gearboxMsg.child(), "wheel");
  EXPECT_TRUE(gearboxMsg.has_cfm());
  EXPECT_NEAR(gearboxMsg.cfm(), 0.3, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_bounce());
  EXPECT_NEAR(gearboxMsg.bounce(), 0.12, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_velocity());
  EXPECT_NEAR(gearboxMsg.velocity(), 1.12, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_fudge_factor());
  EXPECT_NEAR(gearboxMsg.fudge_factor(), 0.4, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_limit_cfm());
  EXPECT_NEAR(gearboxMsg.limit_cfm(), 0.1, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_limit_erp());
  EXPECT_NEAR(gearboxMsg.limit_erp(), 0.9, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_suspension_cfm());
  EXPECT_NEAR(gearboxMsg.suspension_cfm(), 0.2, 1e-6);
  EXPECT_TRUE(gearboxMsg.has_suspension_erp());
  EXPECT_NEAR(gearboxMsg.suspension_erp(), 0.4, 1e-6);

  EXPECT_TRUE(gearboxMsg.has_axis1());
  const msgs::Axis axisGearboxMsg = gearboxMsg.axis1();
  EXPECT_TRUE(axisGearboxMsg.has_xyz());
  EXPECT_EQ(msgs::ConvertIgn(axisGearboxMsg.xyz()),
      ignition::math::Vector3d(0, 0, 1));
  EXPECT_TRUE(axisGearboxMsg.has_limit_lower());
  EXPECT_NEAR(axisGearboxMsg.limit_lower(), 0.01, 1e-6);
  EXPECT_TRUE(axisGearboxMsg.has_limit_upper());
  EXPECT_NEAR(axisGearboxMsg.limit_upper(), 3.0, 1e-6);
  EXPECT_TRUE(axisGearboxMsg.has_limit_effort());
  EXPECT_NEAR(axisGearboxMsg.limit_effort(), 2.1, 1e-6);
  EXPECT_TRUE(axisGearboxMsg.has_limit_velocity());
  EXPECT_NEAR(axisGearboxMsg.limit_velocity(), 0.2, 1e-6);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  EXPECT_TRUE(axisGearboxMsg.has_use_parent_model_frame());
  EXPECT_EQ(axisGearboxMsg.use_parent_model_frame(), false);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  EXPECT_TRUE(axisGearboxMsg.has_xyz_expressed_in());
  EXPECT_EQ(axisGearboxMsg.xyz_expressed_in(), "__model__");
  EXPECT_TRUE(axisGearboxMsg.has_damping());
  EXPECT_NEAR(axisGearboxMsg.damping(), 0.8, 1e-6);
  EXPECT_TRUE(axisGearboxMsg.has_friction());
  EXPECT_NEAR(axisGearboxMsg.friction(), 0.1, 1e-6);

  EXPECT_TRUE(gearboxMsg.has_axis2());
  const msgs::Axis axisGearboxMsg2 = gearboxMsg.axis2();
  EXPECT_TRUE(axisGearboxMsg2.has_xyz());
  EXPECT_EQ(msgs::ConvertIgn(axisGearboxMsg2.xyz()),
      ignition::math::Vector3d(0, 0, 1));
  EXPECT_TRUE(axisGearboxMsg2.has_limit_lower());
  EXPECT_NEAR(axisGearboxMsg2.limit_lower(), 0.02, 1e-6);
  EXPECT_TRUE(axisGearboxMsg2.has_limit_upper());
  EXPECT_NEAR(axisGearboxMsg2.limit_upper(), 3.01, 1e-6);
  EXPECT_TRUE(axisGearboxMsg2.has_limit_effort());
  EXPECT_NEAR(axisGearboxMsg2.limit_effort(), 2.3, 1e-6);
  EXPECT_TRUE(axisGearboxMsg2.has_limit_velocity());
  EXPECT_NEAR(axisGearboxMsg2.limit_velocity(), 0.1, 1e-6);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  EXPECT_TRUE(axisGearboxMsg2.has_use_parent_model_frame());
  EXPECT_EQ(axisGearboxMsg2.use_parent_model_frame(), false);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  EXPECT_TRUE(axisGearboxMsg2.has_xyz_expressed_in());
  EXPECT_EQ(axisGearboxMsg2.xyz_expressed_in(), "__model__");
  EXPECT_TRUE(axisGearboxMsg2.has_damping());
  EXPECT_NEAR(axisGearboxMsg2.damping(), 0.9, 1e-6);
  EXPECT_TRUE(axisGearboxMsg2.has_friction());
  EXPECT_NEAR(axisGearboxMsg2.friction(), 0.1, 1e-6);

  EXPECT_TRUE(gearboxMsg.has_gearbox());
  EXPECT_EQ(gearboxMsg.mutable_gearbox()->gearbox_reference_body(), "chassis");
  EXPECT_NEAR(gearboxMsg.mutable_gearbox()->gearbox_ratio(), 0.2, 1e-6);

  // screw
  sdf::ElementPtr screwSdf(new sdf::Element());
  sdf::initFile("joint.sdf", screwSdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <joint name='box_cylinder' type='screw'>\
           <pose>1 1 0 0 0 1.57</pose>\
           <parent>box</parent>\
           <child>cylinder</child>\
           <axis>\
             <xyz expressed_in='__model__'>0 1 0</xyz>\
             <limit>\
               <lower>0.0</lower>\
               <upper>2.0</upper>\
               <effort>1.21</effort>\
               <velocity>0.12</velocity>\
             </limit>\
             <dynamics>\
               <damping>0.5</damping>\
               <friction>0.12</friction>\
             </dynamics>\
           </axis>\
           <thread_pitch>0.2</thread_pitch>\
           <physics>\
             <ode>\
               <cfm>0.13</cfm>\
               <bounce>0.02</bounce>\
               <velocity>1.1</velocity>\
               <fudge_factor>0.42</fudge_factor>\
               <limit>\
                 <cfm>0.11</cfm>\
                 <erp>0.91</erp>\
               </limit>\
               <suspension>\
                 <cfm>0.22</cfm>\
                 <erp>0.42</erp>\
               </suspension>\
             </ode>\
           </physics>\
         </joint>\
      </sdf>", screwSdf));
  msgs::Joint screwMsg = msgs::JointFromSDF(screwSdf);

  EXPECT_TRUE(screwMsg.has_name());
  EXPECT_EQ(screwMsg.name(), "box_cylinder");
  EXPECT_TRUE(screwMsg.has_type());
  EXPECT_EQ(msgs::ConvertJointType(screwMsg.type()), "screw");
  EXPECT_TRUE(screwMsg.has_pose());
  EXPECT_EQ(msgs::ConvertIgn(screwMsg.pose()),
      ignition::math::Pose3d(1, 1, 0, 0, 0, 1.57));
  EXPECT_TRUE(screwMsg.has_parent());
  EXPECT_EQ(screwMsg.parent(), "box");
  EXPECT_TRUE(screwMsg.has_child());
  EXPECT_EQ(screwMsg.child(), "cylinder");
  EXPECT_TRUE(screwMsg.has_cfm());
  EXPECT_NEAR(screwMsg.cfm(), 0.13, 1e-6);
  EXPECT_TRUE(screwMsg.has_bounce());
  EXPECT_NEAR(screwMsg.bounce(), 0.02, 1e-6);
  EXPECT_TRUE(screwMsg.has_velocity());
  EXPECT_NEAR(screwMsg.velocity(), 1.1, 1e-6);
  EXPECT_TRUE(screwMsg.has_fudge_factor());
  EXPECT_NEAR(screwMsg.fudge_factor(), 0.42, 1e-6);
  EXPECT_TRUE(screwMsg.has_limit_cfm());
  EXPECT_NEAR(screwMsg.limit_cfm(), 0.11, 1e-6);
  EXPECT_TRUE(screwMsg.has_limit_erp());
  EXPECT_NEAR(screwMsg.limit_erp(), 0.91, 1e-6);
  EXPECT_TRUE(screwMsg.has_suspension_cfm());
  EXPECT_NEAR(screwMsg.suspension_cfm(), 0.22, 1e-6);
  EXPECT_TRUE(screwMsg.has_suspension_erp());
  EXPECT_NEAR(screwMsg.suspension_erp(), 0.42, 1e-6);

  EXPECT_TRUE(screwMsg.has_axis1());
  EXPECT_TRUE(!screwMsg.has_axis2());
  const msgs::Axis axisScrewMsg = screwMsg.axis1();
  EXPECT_TRUE(axisScrewMsg.has_xyz());
  EXPECT_EQ(msgs::ConvertIgn(axisScrewMsg.xyz()),
      ignition::math::Vector3d(0, 1, 0));
  EXPECT_TRUE(axisScrewMsg.has_limit_lower());
  EXPECT_NEAR(axisScrewMsg.limit_lower(), 0.0, 1e-6);
  EXPECT_TRUE(axisScrewMsg.has_limit_upper());
  EXPECT_NEAR(axisScrewMsg.limit_upper(), 2.0, 1e-6);
  EXPECT_TRUE(axisScrewMsg.has_limit_effort());
  EXPECT_NEAR(axisScrewMsg.limit_effort(), 1.21, 1e-6);
  EXPECT_TRUE(axisScrewMsg.has_limit_velocity());
  EXPECT_NEAR(axisScrewMsg.limit_velocity(), 0.12, 1e-6);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  EXPECT_TRUE(axisScrewMsg.has_use_parent_model_frame());
  EXPECT_EQ(axisScrewMsg.use_parent_model_frame(), false);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  EXPECT_EQ(axisScrewMsg.has_xyz_expressed_in(), true);
  EXPECT_EQ(axisScrewMsg.xyz_expressed_in(), "__model__");
  EXPECT_TRUE(axisScrewMsg.has_damping());
  EXPECT_NEAR(axisScrewMsg.damping(), 0.5, 1e-6);
  EXPECT_TRUE(axisScrewMsg.has_friction());
  EXPECT_NEAR(axisScrewMsg.friction(), 0.12, 1e-6);

  EXPECT_TRUE(screwMsg.has_screw());
  EXPECT_NEAR(screwMsg.mutable_screw()->thread_pitch(), 0.2, 1e-6);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, LinkToSDF)
{
  const std::string name("test_link");
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(3, 2, 1),
      ignition::math::Quaterniond(0.5, -0.5, -0.5, 0.5));

  msgs::Link linkMsg;
  linkMsg.set_name(name);
  linkMsg.set_self_collide(false);
  linkMsg.set_gravity(true);
  linkMsg.set_kinematic(false);
  linkMsg.set_enable_wind(false);
  msgs::Set(linkMsg.mutable_pose(), pose);

  const double laserRetro1 = 0.4;
  const double laserRetro2 = 0.5;

  // collision - see CollisionToSDF for a more detailed test
  auto collisionMsg1 = linkMsg.add_collision();
  collisionMsg1->set_laser_retro(laserRetro1);
  collisionMsg1->set_max_contacts(100);

  auto collisionMsg2 = linkMsg.add_collision();
  collisionMsg2->set_laser_retro(laserRetro2);
  collisionMsg2->set_max_contacts(300);

  // visual - see VisualToSDF for a more detailed test
  auto visualMsg1 = linkMsg.add_visual();
  visualMsg1->set_laser_retro(laserRetro1);

  auto visualMsg2 = linkMsg.add_visual();
  visualMsg2->set_laser_retro(laserRetro2);

  // inertial - see InertialToSDF for a more detailed test
  auto inertialMsg = linkMsg.mutable_inertial();
  inertialMsg->set_mass(3.5);

  sdf::ElementPtr linkSDF = msgs::LinkToSDF(linkMsg);
  EXPECT_EQ(linkSDF->Get<std::string>("name"), name);
  EXPECT_FALSE(linkSDF->Get<bool>("self_collide"));
  EXPECT_TRUE(linkSDF->Get<bool>("gravity"));
  EXPECT_FALSE(linkSDF->Get<bool>("kinematic"));
  EXPECT_FALSE(linkSDF->Get<bool>("enable_wind"));
  EXPECT_EQ(pose, linkSDF->Get<ignition::math::Pose3d>("pose"));

  sdf::ElementPtr collisionElem1 = linkSDF->GetElement("collision");
  EXPECT_DOUBLE_EQ(collisionElem1->Get<double>("laser_retro"), laserRetro1);
  EXPECT_DOUBLE_EQ(collisionElem1->Get<double>("max_contacts"), 100);

  sdf::ElementPtr collisionElem2 = collisionElem1->GetNextElement("collision");
  EXPECT_DOUBLE_EQ(collisionElem2->Get<double>("laser_retro"), laserRetro2);
  EXPECT_DOUBLE_EQ(collisionElem2->Get<double>("max_contacts"), 300);

  sdf::ElementPtr visualElem1 = linkSDF->GetElement("visual");
  EXPECT_DOUBLE_EQ(visualElem1->Get<double>("laser_retro"), laserRetro1);

  sdf::ElementPtr visualElem2 = visualElem1->GetNextElement("visual");
  EXPECT_DOUBLE_EQ(visualElem2->Get<double>("laser_retro"), laserRetro2);

  sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
  EXPECT_DOUBLE_EQ(inertialElem->Get<double>("mass"), 3.5);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, CollisionToSDF)
{
  const std::string name("collision");

  msgs::Collision collisionMsg;
  collisionMsg.set_name(name);
  collisionMsg.set_laser_retro(0.2);
  collisionMsg.set_max_contacts(5);
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(1, 2, 3),
      ignition::math::Quaterniond(0, 0, 1, 0));
  msgs::Set(collisionMsg.mutable_pose(), pose);

  // geometry - see GeometryToSDF for a more detailed test
  msgs::Geometry *geomMsg = collisionMsg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::CYLINDER);
  msgs::CylinderGeom *cylinderMsg = geomMsg->mutable_cylinder();
  cylinderMsg->set_radius(3.3);
  cylinderMsg->set_length(1.0);

  // surface - see SurfaceToSDF for a more detailed test
  msgs::Surface *surfaceMsg = collisionMsg.mutable_surface();
  surfaceMsg->set_restitution_coefficient(5.1);
  surfaceMsg->set_bounce_threshold(1300);

  sdf::ElementPtr collisionSDF = msgs::CollisionToSDF(collisionMsg);

  EXPECT_TRUE(collisionSDF->HasAttribute("name"));
  EXPECT_EQ(name, collisionSDF->Get<std::string>("name"));

  EXPECT_DOUBLE_EQ(collisionSDF->Get<double>("laser_retro"), 0.2);
  EXPECT_DOUBLE_EQ(collisionSDF->Get<double>("max_contacts"), 5);

  EXPECT_EQ(collisionSDF->Get<ignition::math::Pose3d>("pose"), pose);

  sdf::ElementPtr geomElem = collisionSDF->GetElement("geometry");
  sdf::ElementPtr cylinderElem = geomElem->GetElement("cylinder");
  EXPECT_DOUBLE_EQ(cylinderElem->Get<double>("radius"), 3.3);
  EXPECT_DOUBLE_EQ(cylinderElem->Get<double>("length"), 1.0);

  sdf::ElementPtr surfaceElem = collisionSDF->GetElement("surface");
  sdf::ElementPtr bounceElem = surfaceElem->GetElement("bounce");
  EXPECT_DOUBLE_EQ(bounceElem->Get<double>("restitution_coefficient"), 5.1);
  EXPECT_DOUBLE_EQ(bounceElem->Get<double>("threshold"), 1300);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, VisualToSDF)
{
  const std::string name("visual");
  const double laserRetro = 0.2;
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(1, 2, 3),
      ignition::math::Quaterniond(0, 0, 1, 0));
  const double radius = 3.3;
  const std::string materialName("Gazebo/Grey");
  const std::string uri("pretend_this_is_a_URI");

  msgs::Visual visualMsg;
  visualMsg.set_name(name);
  visualMsg.set_laser_retro(laserRetro);
  msgs::Set(visualMsg.mutable_pose(), pose);

  // geometry - see GeometryToSDF for a more detailed test
  auto geomMsg = visualMsg.mutable_geometry();
  geomMsg->set_type(msgs::Geometry::SPHERE);
  geomMsg->mutable_sphere()->set_radius(radius);

  // material - see MaterialToSDF for a more detailed test
  auto scriptMsg = visualMsg.mutable_material()->mutable_script();
  scriptMsg->set_name(materialName);
  scriptMsg->add_uri();
  scriptMsg->set_uri(0, uri);

  sdf::ElementPtr visualSDF = msgs::VisualToSDF(visualMsg);

  EXPECT_TRUE(visualSDF->HasAttribute("name"));
  EXPECT_EQ(name, visualSDF->Get<std::string>("name"));

  EXPECT_DOUBLE_EQ(visualSDF->Get<double>("laser_retro"), laserRetro);

  EXPECT_EQ(pose, visualSDF->Get<ignition::math::Pose3d>("pose"));

  ASSERT_TRUE(visualSDF->HasElement("geometry"));
  sdf::ElementPtr geomElem = visualSDF->GetElement("geometry");
  EXPECT_TRUE(geomElem->HasElement("sphere"));
  sdf::ElementPtr sphereElem = geomElem->GetElement("sphere");
  EXPECT_TRUE(sphereElem->HasElement("radius"));
  EXPECT_DOUBLE_EQ(sphereElem->Get<double>("radius"), radius);

  ASSERT_TRUE(visualSDF->HasElement("material"));
  sdf::ElementPtr materialElem = visualSDF->GetElement("material");
  EXPECT_TRUE(materialElem->HasElement("script"));
  sdf::ElementPtr scriptElem = materialElem->GetElement("script");
  EXPECT_TRUE(scriptElem->HasElement("name"));
  EXPECT_EQ(materialName, scriptElem->Get<std::string>("name"));
  EXPECT_TRUE(scriptElem->HasElement("uri"));
  EXPECT_EQ(uri, scriptElem->Get<std::string>("uri"));

  // Test the meta.layer property
  {
    msgs::Visual msg;
    auto metaMsg = msg.mutable_meta();
    metaMsg->set_layer(1);

    sdf::ElementPtr visSdf = msgs::VisualToSDF(msg);
    EXPECT_TRUE(visSdf->HasElement("meta"));
    EXPECT_TRUE(visSdf->GetElement("meta")->HasElement("layer"));
    EXPECT_EQ(visSdf->GetElement("meta")->Get<int32_t>("layer"), 1);

    msgs::Visual msg2 = msgs::VisualFromSDF(visSdf);
    EXPECT_TRUE(msg2.has_meta());
    EXPECT_TRUE(msg2.meta().has_layer());
    EXPECT_EQ(msg2.meta().layer(), 1);
  }
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, GeometryToSDF)
{
  // box
  const ignition::math::Vector3d boxSize(0.5, 0.75, 1.0);
  msgs::Geometry boxMsg;
  boxMsg.set_type(msgs::Geometry::BOX);
  msgs::BoxGeom *boxGeom = boxMsg.mutable_box();
  msgs::Set(boxGeom->mutable_size(), boxSize);

  sdf::ElementPtr boxSDF = msgs::GeometryToSDF(boxMsg);
  sdf::ElementPtr boxElem = boxSDF->GetElement("box");
  EXPECT_EQ(boxElem->Get<ignition::math::Vector3d>("size"), boxSize);

  // cylinder
  msgs::Geometry cylinderMsg;
  cylinderMsg.set_type(msgs::Geometry::CYLINDER);
  msgs::CylinderGeom *cylinderGeom = cylinderMsg.mutable_cylinder();
  cylinderGeom->set_radius(0.3);
  cylinderGeom->set_length(1.0);

  sdf::ElementPtr cylinderSDF = msgs::GeometryToSDF(cylinderMsg);
  sdf::ElementPtr cylinderElem = cylinderSDF->GetElement("cylinder");
  EXPECT_DOUBLE_EQ(cylinderElem->Get<double>("radius"), 0.3);
  EXPECT_DOUBLE_EQ(cylinderElem->Get<double>("length"), 1.0);

  // sphere
  msgs::Geometry sphereMsg;
  sphereMsg.set_type(msgs::Geometry::SPHERE);
  msgs::SphereGeom *sphereGeom = sphereMsg.mutable_sphere();
  sphereGeom->set_radius(3.0);

  sdf::ElementPtr sphereSDF = msgs::GeometryToSDF(sphereMsg);
  sdf::ElementPtr sphereElem = sphereSDF->GetElement("sphere");
  EXPECT_DOUBLE_EQ(sphereElem->Get<double>("radius"), 3.0);

  // plane
  msgs::Geometry planeMsg;
  planeMsg.set_type(msgs::Geometry::PLANE);
  msgs::PlaneGeom *planeGeom = planeMsg.mutable_plane();
  msgs::Set(planeGeom->mutable_normal(), ignition::math::Vector3d(0, 0, 1.0));
  msgs::Set(planeGeom->mutable_size(), ignition::math::Vector2d(0.5, 0.8));

  sdf::ElementPtr planeSDF = msgs::GeometryToSDF(planeMsg);
  sdf::ElementPtr planeElem = planeSDF->GetElement("plane");
  EXPECT_TRUE(planeElem->Get<ignition::math::Vector3d>("normal") ==
      ignition::math::Vector3d(0, 0, 1.0));
  EXPECT_TRUE(planeElem->Get<ignition::math::Vector2d>("size") ==
      ignition::math::Vector2d(0.5, 0.8));

  // image
  msgs::Geometry imageMsg;
  imageMsg.set_type(msgs::Geometry::IMAGE);
  msgs::ImageGeom *imageGeom = imageMsg.mutable_image();
  imageGeom->set_uri("test_uri");
  imageGeom->set_scale(1.8);
  imageGeom->set_threshold(255);
  imageGeom->set_height(1.3);
  imageGeom->set_granularity(2);

  sdf::ElementPtr imageSDF = msgs::GeometryToSDF(imageMsg);
  sdf::ElementPtr imageElem = imageSDF->GetElement("image");
  EXPECT_STREQ(imageElem->Get<std::string>("uri").c_str(), "test_uri");
  EXPECT_DOUBLE_EQ(imageElem->Get<double>("scale"), 1.8);
  EXPECT_DOUBLE_EQ(imageElem->Get<double>("threshold"), 255);
  EXPECT_DOUBLE_EQ(imageElem->Get<double>("height"), 1.3);
  EXPECT_DOUBLE_EQ(imageElem->Get<int>("granularity"), 2);

  // heightmap
  msgs::Geometry heightmapMsg;
  heightmapMsg.set_type(msgs::Geometry::HEIGHTMAP);
  msgs::HeightmapGeom *heightmapGeom = heightmapMsg.mutable_heightmap();
  heightmapGeom->set_filename("test_heightmap_filename");
  msgs::Set(heightmapGeom->mutable_size(),
      ignition::math::Vector3d(100, 200, 30));
  msgs::Set(heightmapGeom->mutable_origin(),
      ignition::math::Vector3d(50, 100, 15));
  heightmapGeom->set_sampling(1);
  heightmapGeom->set_use_terrain_paging(true);

  msgs::HeightmapGeom_Texture *texture1 = heightmapGeom->add_texture();
  texture1->set_diffuse("test_diffuse1");
  texture1->set_normal("test_normal1");
  texture1->set_size(10);

  msgs::HeightmapGeom_Texture *texture2 = heightmapGeom->add_texture();
  texture2->set_diffuse("test_diffuse2");
  texture2->set_normal("test_normal2");
  texture2->set_size(20);

  msgs::HeightmapGeom_Blend *blend = heightmapGeom->add_blend();
  blend->set_min_height(25);
  blend->set_fade_dist(5);

  sdf::ElementPtr heightmapSDF = msgs::GeometryToSDF(heightmapMsg);
  sdf::ElementPtr heightmapElem = heightmapSDF->GetElement("heightmap");
  EXPECT_STREQ(heightmapElem->Get<std::string>("uri").c_str(),
      "test_heightmap_filename");
  EXPECT_TRUE(heightmapElem->Get<ignition::math::Vector3d>("size") ==
      ignition::math::Vector3d(100, 200, 30));
  EXPECT_TRUE(heightmapElem->Get<ignition::math::Vector3d>("pos") ==
      ignition::math::Vector3d(50, 100, 15));
  // fix test while we wait for new sdformat4 version to be released
  if (heightmapElem->HasElementDescription("sampling"))
  {
    EXPECT_EQ(heightmapElem->Get<unsigned int>("sampling"), 1u);
  }
  EXPECT_TRUE(heightmapElem->Get<bool>("use_terrain_paging"));

  sdf::ElementPtr textureElem1 = heightmapElem->GetElement("texture");
  EXPECT_STREQ(textureElem1->Get<std::string>("diffuse").c_str(),
      "test_diffuse1");
  EXPECT_STREQ(textureElem1->Get<std::string>("normal").c_str(),
      "test_normal1");
  EXPECT_DOUBLE_EQ(textureElem1->Get<double>("size"), 10);
  sdf::ElementPtr textureElem2 = textureElem1->GetNextElement("texture");
  EXPECT_STREQ(textureElem2->Get<std::string>("diffuse").c_str(),
      "test_diffuse2");
  EXPECT_STREQ(textureElem2->Get<std::string>("normal").c_str(),
      "test_normal2");
  EXPECT_DOUBLE_EQ(textureElem2->Get<double>("size"), 20);

  sdf::ElementPtr blendElem = heightmapElem->GetElement("blend");
  EXPECT_DOUBLE_EQ(blendElem->Get<double>("min_height"), 25);
  EXPECT_DOUBLE_EQ(blendElem->Get<double>("fade_dist"), 5);

  // mesh
  msgs::Geometry meshMsg;
  meshMsg.set_type(msgs::Geometry::MESH);
  msgs::MeshGeom *meshGeom = meshMsg.mutable_mesh();
  meshGeom->set_filename("test_mesh_filename");
  msgs::Set(meshGeom->mutable_scale(), ignition::math::Vector3d(2.3, 1.2, 2.9));
  meshGeom->set_submesh("test_mesh_submesh");
  meshGeom->set_center_submesh(false);

  sdf::ElementPtr meshSDF = msgs::GeometryToSDF(meshMsg);
  sdf::ElementPtr meshElem = meshSDF->GetElement("mesh");
  EXPECT_STREQ(meshElem->Get<std::string>("uri").c_str(),
      "test_mesh_filename");
  EXPECT_TRUE(meshElem->Get<ignition::math::Vector3d>("scale") ==
      ignition::math::Vector3d(2.3, 1.2, 2.9));
  sdf::ElementPtr submeshElem = meshElem->GetElement("submesh");
  EXPECT_STREQ(submeshElem->Get<std::string>("name").c_str(),
      "test_mesh_submesh");
  EXPECT_TRUE(!submeshElem->Get<bool>("center"));

  // polyline
  msgs::Geometry polylineMsg;
  polylineMsg.set_type(msgs::Geometry::POLYLINE);
  msgs::Polyline *polylineGeom = polylineMsg.add_polyline();
  polylineGeom->set_height(2.33);
  const ignition::math::Vector2d point1(0.5, 0.7);
  const ignition::math::Vector2d point2(3.5, 4.7);
  const ignition::math::Vector2d point3(1000, 2000);
  msgs::Set(polylineGeom->add_point(), point1);
  msgs::Set(polylineGeom->add_point(), point2);
  msgs::Set(polylineGeom->add_point(), point3);

  sdf::ElementPtr polylineSDF = msgs::GeometryToSDF(polylineMsg);
  sdf::ElementPtr polylineElem = polylineSDF->GetElement("polyline");
  EXPECT_DOUBLE_EQ(polylineElem->Get<double>("height"), 2.33);

  sdf::ElementPtr pointElem1 = polylineElem->GetElement("point");
  sdf::ElementPtr pointElem2 = pointElem1->GetNextElement("point");
  sdf::ElementPtr pointElem3 = pointElem2->GetNextElement("point");
  EXPECT_EQ(pointElem1->Get<ignition::math::Vector2d>(), point1);
  EXPECT_EQ(pointElem2->Get<ignition::math::Vector2d>(), point2);
  EXPECT_EQ(pointElem3->Get<ignition::math::Vector2d>(), point3);
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, MeshToSDF)
{
  const ignition::math::Vector3d meshScale(0.1, 0.2, 0.3);
  msgs::MeshGeom msg;
  msg.set_filename("test_filename");
  msgs::Set(msg.mutable_scale(), meshScale);
  msg.set_submesh("test_submesh");
  msg.set_center_submesh(true);

  sdf::ElementPtr meshSDF = msgs::MeshToSDF(msg);

  EXPECT_STREQ(meshSDF->Get<std::string>("uri").c_str(), "test_filename");
  EXPECT_TRUE(meshSDF->HasElement("scale"));
  ignition::math::Vector3d scale =
      meshSDF->Get<ignition::math::Vector3d>("scale");
  EXPECT_DOUBLE_EQ(scale.X(), meshScale.X());
  EXPECT_DOUBLE_EQ(scale.Y(), meshScale.Y());
  EXPECT_DOUBLE_EQ(scale.Z(), meshScale.Z());

  sdf::ElementPtr submeshElem = meshSDF->GetElement("submesh");
  EXPECT_STREQ(submeshElem->Get<std::string>("name").c_str(), "test_submesh");
  EXPECT_TRUE(submeshElem->Get<bool>("center"));

  // no submesh
  const ignition::math::Vector3d meshScale2(1, 2, 3);
  msgs::MeshGeom msg2;
  msg2.set_filename("test_filename2");
  msgs::Set(msg2.mutable_scale(), meshScale2);

  sdf::ElementPtr meshSDF2 = msgs::MeshToSDF(msg2);

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
TEST_F(MsgsTest, InertialToSDF)
{
  const double mass = 3.4;
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(1.2, 3.4, 5.6),
      ignition::math::Quaterniond(0.7071, 0.0, 0.7071, 0.0));
  const double ixx = 0.0133;
  const double ixy = -0.0003;
  const double ixz = -0.0004;
  const double iyy = 0.0116;
  const double iyz = 0.0008;
  const double izz = 0.0038;

  msgs::Inertial msg;
  msg.set_mass(mass);
  msgs::Set(msg.mutable_pose(), pose);
  msg.set_ixx(ixx);
  msg.set_ixy(ixy);
  msg.set_ixz(ixz);
  msg.set_iyy(iyy);
  msg.set_iyz(iyz);
  msg.set_izz(izz);

  sdf::ElementPtr inertialSDF = msgs::InertialToSDF(msg);

  EXPECT_TRUE(inertialSDF->HasElement("mass"));
  EXPECT_DOUBLE_EQ(inertialSDF->Get<double>("mass"), mass);

  EXPECT_TRUE(inertialSDF->HasElement("pose"));
  EXPECT_EQ(inertialSDF->Get<ignition::math::Pose3d>("pose"), pose);

  {
    ASSERT_TRUE(inertialSDF->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialSDF->GetElement("inertia");

    EXPECT_TRUE(inertiaElem->HasElement("ixx"));
    EXPECT_DOUBLE_EQ(inertiaElem->Get<double>("ixx"), ixx);

    EXPECT_TRUE(inertiaElem->HasElement("ixy"));
    EXPECT_DOUBLE_EQ(inertiaElem->Get<double>("ixy"), ixy);

    EXPECT_TRUE(inertiaElem->HasElement("ixz"));
    EXPECT_DOUBLE_EQ(inertiaElem->Get<double>("ixz"), ixz);

    EXPECT_TRUE(inertiaElem->HasElement("iyy"));
    EXPECT_DOUBLE_EQ(inertiaElem->Get<double>("iyy"), iyy);

    EXPECT_TRUE(inertiaElem->HasElement("iyz"));
    EXPECT_DOUBLE_EQ(inertiaElem->Get<double>("iyz"), iyz);

    EXPECT_TRUE(inertiaElem->HasElement("izz"));
    EXPECT_DOUBLE_EQ(inertiaElem->Get<double>("izz"), izz);
  }
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, MaterialToSDF)
{
  msgs::Material msg;

  const std::string name("Gazebo/Grey");
  const std::string uri("file://media/materials/scripts/gazebo.material");
  const msgs::Material::ShaderType type = msgs::Material::VERTEX;
  const std::string normalMap("normalMap");
  const bool lighting = true;
  const ignition::math::Color ambient(.1, .2, .3, 1.0);
  const ignition::math::Color diffuse(.4, .5, .6, 1.0);
  const ignition::math::Color emissive(.5, .5, .5, 0.5);
  const ignition::math::Color specular(.7, .8, .9, 1.0);

  msg.mutable_script()->set_name(name);
  msg.mutable_script()->add_uri();
  msg.mutable_script()->set_uri(0, uri);
  msg.set_shader_type(type);
  msg.set_normal_map(normalMap);
  msg.set_lighting(lighting);
  msgs::Set(msg.mutable_ambient(), ambient);
  msgs::Set(msg.mutable_diffuse(), diffuse);
  msgs::Set(msg.mutable_emissive(), emissive);
  msgs::Set(msg.mutable_specular(), specular);

  sdf::ElementPtr materialSDF = msgs::MaterialToSDF(msg);

  {
    ASSERT_TRUE(materialSDF->HasElement("script"));
    sdf::ElementPtr scriptElem = materialSDF->GetElement("script");
    EXPECT_TRUE(scriptElem->HasElement("name"));
    EXPECT_EQ(name, scriptElem->Get<std::string>("name"));
    EXPECT_TRUE(scriptElem->HasElement("uri"));
    EXPECT_EQ(uri, scriptElem->Get<std::string>("uri"));
  }

  {
    ASSERT_TRUE(materialSDF->HasElement("shader"));
    sdf::ElementPtr shaderElem = materialSDF->GetElement("shader");
    EXPECT_TRUE(shaderElem->HasAttribute("type"));
    EXPECT_EQ(msgs::ConvertShaderType(type),
              shaderElem->Get<std::string>("type"));
    EXPECT_TRUE(shaderElem->HasElement("normal_map"));
    EXPECT_EQ(normalMap, shaderElem->Get<std::string>("normal_map"));
  }

  EXPECT_TRUE(materialSDF->HasElement("ambient"));
  EXPECT_EQ(ambient, materialSDF->Get<ignition::math::Color>("ambient"));
  EXPECT_TRUE(materialSDF->HasElement("diffuse"));
  EXPECT_EQ(diffuse, materialSDF->Get<ignition::math::Color>("diffuse"));
  EXPECT_TRUE(materialSDF->HasElement("emissive"));
  EXPECT_EQ(emissive, materialSDF->Get<ignition::math::Color>("emissive"));
  EXPECT_TRUE(materialSDF->HasElement("specular"));
  EXPECT_EQ(specular, materialSDF->Get<ignition::math::Color>("specular"));
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, SurfaceToSDF)
{
  msgs::Surface msg;

  // friction
  const double mu = 0.1;
  const double mu2 = 0.2;
  const ignition::math::Vector3d fdir1(0.3, 0.4, 0.5);
  const double slip1 = 0.6;
  const double slip2 = 0.7;

  msgs::Friction *friction = msg.mutable_friction();
  friction->set_mu(mu);
  friction->set_mu2(mu2);
  msgs::Set(friction->mutable_fdir1(), fdir1);
  friction->set_slip1(slip1);
  friction->set_slip2(slip2);

  // bounce
  msg.set_restitution_coefficient(1.1);
  msg.set_bounce_threshold(1000);

  // other ode surface properties
  msg.set_soft_cfm(0.9);
  msg.set_soft_erp(0.3);
  msg.set_kp(0.4);
  msg.set_kd(0.8);
  msg.set_max_vel(3.8);
  msg.set_min_depth(0.0001);
  msg.set_collide_without_contact(true);
  msg.set_collide_without_contact_bitmask(0x0004);
  msg.set_collide_bitmask(0x01);

  sdf::ElementPtr surfaceSDF = msgs::SurfaceToSDF(msg);
  sdf::ElementPtr frictionElem = surfaceSDF->GetElement("friction");
  sdf::ElementPtr frictionPhysicsElem = frictionElem->GetElement("ode");
  EXPECT_DOUBLE_EQ(frictionPhysicsElem->Get<double>("mu"), mu);
  EXPECT_DOUBLE_EQ(frictionPhysicsElem->Get<double>("mu2"), mu2);
  EXPECT_EQ(frictionPhysicsElem->Get<ignition::math::Vector3d>("fdir1"), fdir1);
  EXPECT_DOUBLE_EQ(frictionPhysicsElem->Get<double>("slip1"), slip1);
  EXPECT_DOUBLE_EQ(frictionPhysicsElem->Get<double>("slip2"), slip2);

  sdf::ElementPtr bounceElem = surfaceSDF->GetElement("bounce");
  EXPECT_DOUBLE_EQ(bounceElem->Get<double>("restitution_coefficient"), 1.1);
  EXPECT_DOUBLE_EQ(bounceElem->Get<double>("threshold"), 1000);

  sdf::ElementPtr contactElem = surfaceSDF->GetElement("contact");
  sdf::ElementPtr contactPhysicsElem = contactElem->GetElement("ode");
  EXPECT_DOUBLE_EQ(contactPhysicsElem->Get<double>("soft_cfm"), 0.9);
  EXPECT_DOUBLE_EQ(contactPhysicsElem->Get<double>("soft_erp"), 0.3);
  EXPECT_DOUBLE_EQ(contactPhysicsElem->Get<double>("kp"), 0.4);
  EXPECT_DOUBLE_EQ(contactPhysicsElem->Get<double>("kd"), 0.8);
  EXPECT_DOUBLE_EQ(contactPhysicsElem->Get<double>("max_vel"), 3.8);
  EXPECT_DOUBLE_EQ(contactPhysicsElem->Get<double>("min_depth"), 0.0001);

  EXPECT_TRUE(contactElem->Get<bool>("collide_without_contact"));
  EXPECT_EQ(contactElem->Get<unsigned int>("collide_without_contact_bitmask"),
      static_cast<unsigned int>(0x0004));
  EXPECT_EQ(contactElem->Get<unsigned int>("collide_bitmask"),
      static_cast<unsigned int>(0x01));
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, JointToSDF)
{
  // universal
  {
    const std::string name("test_joint");
    const msgs::Joint::Type type = msgs::Joint::UNIVERSAL;
    const ignition::math::Pose3d pose(
        ignition::math::Vector3d(9, 1, 1),
        ignition::math::Quaterniond(0, 1, 0, 0));
    const std::string parent("parent_link");
    const std::string child("child_link");

    const double cfm = 0.1;
    const double bounce = 0.2;
    const double velocity = 0.6;
    const double fudge_factor = 0.7;
    const double limit_cfm = 0.3;
    const double limit_erp = 0.4;
    const double suspension_cfm = 0.8;
    const double suspension_erp = 0.9;
    const ignition::math::Vector3d xyz1(0.6, 0.8, 0.0);
    const ignition::math::Vector3d xyz2(0.0, 0.0, 1.0);
    const double limit_lower1 = -2.0;
    const double limit_lower2 = -4.0;
    const double limit_upper1 = 12.0;
    const double limit_upper2 = 24.0;
    const double limit_effort1 = 1e3;
    const double limit_effort2 = 1e4;
    const double limit_velocity1 = 33;
    const double limit_velocity2 = 44;
    const double damping1 = 1e-2;
    const double damping2 = 3e-2;
    const double friction1 = 1e2;
    const double friction2 = 3e2;
    const bool useParentModelFrame1 = true;
    // don't set use_parent_model_frame for axis2
    // expect it to match sdformat default (false)

    msgs::Joint jointMsg;
    jointMsg.set_name(name);
    jointMsg.set_type(type);
    jointMsg.set_parent(parent);
    jointMsg.set_child(child);
    msgs::Set(jointMsg.mutable_pose(), pose);
    jointMsg.set_cfm(cfm);
    jointMsg.set_bounce(bounce);
    jointMsg.set_velocity(velocity);
    jointMsg.set_fudge_factor(fudge_factor);
    jointMsg.set_limit_cfm(limit_cfm);
    jointMsg.set_limit_erp(limit_erp);
    jointMsg.set_suspension_cfm(suspension_cfm);
    jointMsg.set_suspension_erp(suspension_erp);
    {
      auto axis1 = jointMsg.mutable_axis1();
      msgs::Set(axis1->mutable_xyz(), xyz1);
      axis1->set_limit_lower(limit_lower1);
      axis1->set_limit_upper(limit_upper1);
      axis1->set_limit_effort(limit_effort1);
      axis1->set_limit_velocity(limit_velocity1);
      axis1->set_damping(damping1);
      axis1->set_friction(friction1);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      axis1->set_use_parent_model_frame(useParentModelFrame1);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
    }
    {
      auto axis2 = jointMsg.mutable_axis2();
      msgs::Set(axis2->mutable_xyz(), xyz2);
      axis2->set_limit_lower(limit_lower2);
      axis2->set_limit_upper(limit_upper2);
      axis2->set_limit_effort(limit_effort2);
      axis2->set_limit_velocity(limit_velocity2);
      axis2->set_damping(damping2);
      axis2->set_friction(friction2);
    }

    sdf::ElementPtr jointSDF = msgs::JointToSDF(jointMsg);
    EXPECT_TRUE(jointSDF->HasAttribute("name"));
    EXPECT_EQ(jointSDF->Get<std::string>("name"), name);
    EXPECT_TRUE(jointSDF->HasAttribute("type"));
    EXPECT_STREQ(jointSDF->Get<std::string>("type").c_str(), "universal");
    EXPECT_TRUE(jointSDF->HasElement("parent"));
    EXPECT_EQ(jointSDF->Get<std::string>("parent"), parent);
    EXPECT_TRUE(jointSDF->HasElement("child"));
    EXPECT_EQ(jointSDF->Get<std::string>("child"), child);
    EXPECT_TRUE(jointSDF->HasElement("pose"));
    EXPECT_EQ(pose, jointSDF->Get<ignition::math::Pose3d>("pose"));

    EXPECT_TRUE(jointSDF->HasElement("axis"));
    {
      auto axisElem = jointSDF->GetElement("axis");
      EXPECT_TRUE(axisElem->HasElement("xyz"));
      EXPECT_EQ(xyz1, axisElem->Get<ignition::math::Vector3d>("xyz"));
      EXPECT_TRUE(axisElem->GetElement("xyz")->HasAttribute("expressed_in"));
      EXPECT_EQ("__model__",
                axisElem->GetElement("xyz")->Get<std::string>("expressed_in"));

      EXPECT_TRUE(axisElem->HasElement("dynamics"));
      auto axisDynamics = axisElem->GetElement("dynamics");
      EXPECT_TRUE(axisDynamics->HasElement("damping"));
      EXPECT_DOUBLE_EQ(damping1, axisDynamics->Get<double>("damping"));
      EXPECT_TRUE(axisDynamics->HasElement("friction"));
      EXPECT_DOUBLE_EQ(friction1, axisDynamics->Get<double>("friction"));

      EXPECT_TRUE(axisElem->HasElement("limit"));
      auto axisLimit = axisElem->GetElement("limit");
      EXPECT_TRUE(axisLimit->HasElement("lower"));
      EXPECT_DOUBLE_EQ(limit_lower1, axisLimit->Get<double>("lower"));
      EXPECT_TRUE(axisLimit->HasElement("upper"));
      EXPECT_DOUBLE_EQ(limit_upper1, axisLimit->Get<double>("upper"));
      EXPECT_TRUE(axisLimit->HasElement("effort"));
      EXPECT_DOUBLE_EQ(limit_effort1, axisLimit->Get<double>("effort"));
      EXPECT_TRUE(axisLimit->HasElement("velocity"));
      EXPECT_DOUBLE_EQ(limit_velocity1, axisLimit->Get<double>("velocity"));
    }

    EXPECT_TRUE(jointSDF->HasElement("axis2"));
    {
      auto axisElem = jointSDF->GetElement("axis2");
      EXPECT_TRUE(axisElem->HasElement("xyz"));
      EXPECT_EQ(xyz2, axisElem->Get<ignition::math::Vector3d>("xyz"));
      // use_parent_model_frame is required in axis.proto, but since it is
      // false by default, the expressed_in attribute will empty. Actually, the
      // the attribute should not even be set, but it appears that libsdformat
      // always sets the attribute with a default value
      EXPECT_TRUE(axisElem->GetElement("xyz")->HasAttribute("expressed_in"));
      EXPECT_EQ("",
                axisElem->GetElement("xyz")->Get<std::string>("expressed_in"));

      EXPECT_TRUE(axisElem->HasElement("dynamics"));
      auto axisDynamics = axisElem->GetElement("dynamics");
      EXPECT_TRUE(axisDynamics->HasElement("damping"));
      EXPECT_DOUBLE_EQ(damping2, axisDynamics->Get<double>("damping"));
      EXPECT_TRUE(axisDynamics->HasElement("friction"));
      EXPECT_DOUBLE_EQ(friction2, axisDynamics->Get<double>("friction"));

      EXPECT_TRUE(axisElem->HasElement("limit"));
      auto axisLimit = axisElem->GetElement("limit");
      EXPECT_TRUE(axisLimit->HasElement("lower"));
      EXPECT_DOUBLE_EQ(limit_lower2, axisLimit->Get<double>("lower"));
      EXPECT_TRUE(axisLimit->HasElement("upper"));
      EXPECT_DOUBLE_EQ(limit_upper2, axisLimit->Get<double>("upper"));
      EXPECT_TRUE(axisLimit->HasElement("effort"));
      EXPECT_DOUBLE_EQ(limit_effort2, axisLimit->Get<double>("effort"));
      EXPECT_TRUE(axisLimit->HasElement("velocity"));
      EXPECT_DOUBLE_EQ(limit_velocity2, axisLimit->Get<double>("velocity"));
    }

    EXPECT_TRUE(jointSDF->HasElement("physics"));
    auto physicsElem = jointSDF->GetElement("physics");
    EXPECT_TRUE(physicsElem->HasElement("ode"));
    auto odePhysics = physicsElem->GetElement("ode");
    EXPECT_TRUE(odePhysics->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("cfm"), cfm);
    EXPECT_TRUE(odePhysics->HasElement("bounce"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("bounce"), bounce);
    EXPECT_TRUE(odePhysics->HasElement("velocity"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("velocity"), velocity);
    EXPECT_TRUE(odePhysics->HasElement("fudge_factor"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("fudge_factor"), fudge_factor);

    EXPECT_TRUE(odePhysics->HasElement("limit"));
    auto limitElem = odePhysics->GetElement("limit");
    EXPECT_TRUE(limitElem->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(limitElem->Get<double>("cfm"), limit_cfm);
    EXPECT_TRUE(limitElem->HasElement("erp"));
    EXPECT_DOUBLE_EQ(limitElem->Get<double>("erp"), limit_erp);

    EXPECT_TRUE(odePhysics->HasElement("suspension"));
    auto suspensionElem = odePhysics->GetElement("suspension");
    EXPECT_TRUE(suspensionElem->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(suspensionElem->Get<double>("cfm"), suspension_cfm);
    EXPECT_TRUE(suspensionElem->HasElement("erp"));
    EXPECT_DOUBLE_EQ(suspensionElem->Get<double>("erp"), suspension_erp);
  }

  // gearbox
  {
    const std::string name("test_gearbox_joint");
    const msgs::Joint::Type type = msgs::Joint::GEARBOX;
    const ignition::math::Pose3d pose(
        ignition::math::Vector3d(2, 1, 3),
        ignition::math::Quaterniond(0, 0, 1, 0));
    const std::string parent("parent_gearbox_link");
    const std::string child("child_gearbox_link");

    const double cfm = 0.1;
    const double bounce = 0.1;
    const double velocity = 0.2;
    const double fudge_factor = 0.4;
    const double limit_cfm = 0.2;
    const double limit_erp = 0.6;
    const double suspension_cfm = 0.7;
    const double suspension_erp = 0.4;
    const ignition::math::Vector3d xyz1(0.0, 1.0, 0.0);
    const ignition::math::Vector3d xyz2(0.0, 1.0, 0.0);
    const double limit_lower1 = -1.0;
    const double limit_lower2 = -2.0;
    const double limit_upper1 = 2.0;
    const double limit_upper2 = 4.0;
    const double limit_effort1 = 1e2;
    const double limit_effort2 = 1e3;
    const double limit_velocity1 = 23;
    const double limit_velocity2 = 54;
    const double damping1 = 2e-2;
    const double damping2 = 4e-2;
    const double friction1 = 2e2;
    const double friction2 = 1e2;
    const bool useParentModelFrame1 = true;
    // don't set use_parent_model_frame for axis2
    // expect it to match sdformat default (false)
    const std::string gearbox_reference_body = "child_gearbox_link";
    const double gearbox_ratio = 0.2;

    msgs::Joint jointMsg;
    jointMsg.set_name(name);
    jointMsg.set_type(type);
    jointMsg.set_parent(parent);
    jointMsg.set_child(child);
    msgs::Set(jointMsg.mutable_pose(), pose);
    jointMsg.set_cfm(cfm);
    jointMsg.set_bounce(bounce);
    jointMsg.set_velocity(velocity);
    jointMsg.set_fudge_factor(fudge_factor);
    jointMsg.set_limit_cfm(limit_cfm);
    jointMsg.set_limit_erp(limit_erp);
    jointMsg.set_suspension_cfm(suspension_cfm);
    jointMsg.set_suspension_erp(suspension_erp);
    {
      auto axis1 = jointMsg.mutable_axis1();
      msgs::Set(axis1->mutable_xyz(), xyz1);
      axis1->set_limit_lower(limit_lower1);
      axis1->set_limit_upper(limit_upper1);
      axis1->set_limit_effort(limit_effort1);
      axis1->set_limit_velocity(limit_velocity1);
      axis1->set_damping(damping1);
      axis1->set_friction(friction1);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      axis1->set_use_parent_model_frame(useParentModelFrame1);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
    }
    {
      auto axis2 = jointMsg.mutable_axis2();
      msgs::Set(axis2->mutable_xyz(), xyz2);
      axis2->set_limit_lower(limit_lower2);
      axis2->set_limit_upper(limit_upper2);
      axis2->set_limit_effort(limit_effort2);
      axis2->set_limit_velocity(limit_velocity2);
      axis2->set_damping(damping2);
      axis2->set_friction(friction2);
    }
    msgs::Joint::Gearbox *gearboxMsg = jointMsg.mutable_gearbox();
    gearboxMsg->set_gearbox_reference_body(gearbox_reference_body);
    gearboxMsg->set_gearbox_ratio(gearbox_ratio);

    sdf::ElementPtr jointSDF = msgs::JointToSDF(jointMsg);
    EXPECT_TRUE(jointSDF->HasAttribute("name"));
    EXPECT_EQ(jointSDF->Get<std::string>("name"), name);
    EXPECT_TRUE(jointSDF->HasAttribute("type"));
    EXPECT_STREQ(jointSDF->Get<std::string>("type").c_str(), "gearbox");
    EXPECT_TRUE(jointSDF->HasElement("parent"));
    EXPECT_EQ(jointSDF->Get<std::string>("parent"), parent);
    EXPECT_TRUE(jointSDF->HasElement("child"));
    EXPECT_EQ(jointSDF->Get<std::string>("child"), child);
    EXPECT_TRUE(jointSDF->HasElement("pose"));
    EXPECT_EQ(pose, jointSDF->Get<ignition::math::Pose3d>("pose"));

    EXPECT_TRUE(jointSDF->HasElement("axis"));
    {
      auto axisElem = jointSDF->GetElement("axis");
      EXPECT_TRUE(axisElem->HasElement("xyz"));
      EXPECT_EQ(xyz1, axisElem->Get<ignition::math::Vector3d>("xyz"));
      EXPECT_TRUE(axisElem->GetElement("xyz")->HasAttribute("expressed_in"));
      EXPECT_EQ("__model__",
                axisElem->GetElement("xyz")->Get<std::string>("expressed_in"));

      EXPECT_TRUE(axisElem->HasElement("dynamics"));
      auto axisDynamics = axisElem->GetElement("dynamics");
      EXPECT_TRUE(axisDynamics->HasElement("damping"));
      EXPECT_DOUBLE_EQ(damping1, axisDynamics->Get<double>("damping"));
      EXPECT_TRUE(axisDynamics->HasElement("friction"));
      EXPECT_DOUBLE_EQ(friction1, axisDynamics->Get<double>("friction"));

      EXPECT_TRUE(axisElem->HasElement("limit"));
      auto axisLimit = axisElem->GetElement("limit");
      EXPECT_TRUE(axisLimit->HasElement("lower"));
      EXPECT_DOUBLE_EQ(limit_lower1, axisLimit->Get<double>("lower"));
      EXPECT_TRUE(axisLimit->HasElement("upper"));
      EXPECT_DOUBLE_EQ(limit_upper1, axisLimit->Get<double>("upper"));
      EXPECT_TRUE(axisLimit->HasElement("effort"));
      EXPECT_DOUBLE_EQ(limit_effort1, axisLimit->Get<double>("effort"));
      EXPECT_TRUE(axisLimit->HasElement("velocity"));
      EXPECT_DOUBLE_EQ(limit_velocity1, axisLimit->Get<double>("velocity"));
    }

    EXPECT_TRUE(jointSDF->HasElement("axis2"));
    {
      auto axisElem = jointSDF->GetElement("axis2");
      EXPECT_TRUE(axisElem->HasElement("xyz"));
      EXPECT_EQ(xyz2, axisElem->Get<ignition::math::Vector3d>("xyz"));
      // use_parent_model_frame is required in axis.proto, but since it is
      // false by default, the expressed_in attribute will empty. Actually, the
      // the attribute should not even be set, but it appears that libsdformat
      // always sets the attribute with a default value
      EXPECT_TRUE(axisElem->GetElement("xyz")->HasAttribute("expressed_in"));
      EXPECT_EQ("",
                axisElem->GetElement("xyz")->Get<std::string>("expressed_in"));

      EXPECT_TRUE(axisElem->HasElement("dynamics"));
      auto axisDynamics = axisElem->GetElement("dynamics");
      EXPECT_TRUE(axisDynamics->HasElement("damping"));
      EXPECT_DOUBLE_EQ(damping2, axisDynamics->Get<double>("damping"));
      EXPECT_TRUE(axisDynamics->HasElement("friction"));
      EXPECT_DOUBLE_EQ(friction2, axisDynamics->Get<double>("friction"));

      EXPECT_TRUE(axisElem->HasElement("limit"));
      auto axisLimit = axisElem->GetElement("limit");
      EXPECT_TRUE(axisLimit->HasElement("lower"));
      EXPECT_DOUBLE_EQ(limit_lower2, axisLimit->Get<double>("lower"));
      EXPECT_TRUE(axisLimit->HasElement("upper"));
      EXPECT_DOUBLE_EQ(limit_upper2, axisLimit->Get<double>("upper"));
      EXPECT_TRUE(axisLimit->HasElement("effort"));
      EXPECT_DOUBLE_EQ(limit_effort2, axisLimit->Get<double>("effort"));
      EXPECT_TRUE(axisLimit->HasElement("velocity"));
      EXPECT_DOUBLE_EQ(limit_velocity2, axisLimit->Get<double>("velocity"));
    }

    EXPECT_TRUE(jointSDF->HasElement("physics"));
    auto physicsElem = jointSDF->GetElement("physics");
    EXPECT_TRUE(physicsElem->HasElement("ode"));
    auto odePhysics = physicsElem->GetElement("ode");
    EXPECT_TRUE(odePhysics->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("cfm"), cfm);
    EXPECT_TRUE(odePhysics->HasElement("bounce"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("bounce"), bounce);
    EXPECT_TRUE(odePhysics->HasElement("velocity"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("velocity"), velocity);
    EXPECT_TRUE(odePhysics->HasElement("fudge_factor"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("fudge_factor"), fudge_factor);

    EXPECT_TRUE(odePhysics->HasElement("limit"));
    auto limitElem = odePhysics->GetElement("limit");
    EXPECT_TRUE(limitElem->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(limitElem->Get<double>("cfm"), limit_cfm);
    EXPECT_TRUE(limitElem->HasElement("erp"));
    EXPECT_DOUBLE_EQ(limitElem->Get<double>("erp"), limit_erp);

    EXPECT_TRUE(odePhysics->HasElement("suspension"));
    auto suspensionElem = odePhysics->GetElement("suspension");
    EXPECT_TRUE(suspensionElem->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(suspensionElem->Get<double>("cfm"), suspension_cfm);
    EXPECT_TRUE(suspensionElem->HasElement("erp"));
    EXPECT_DOUBLE_EQ(suspensionElem->Get<double>("erp"), suspension_erp);

    EXPECT_TRUE(jointSDF->HasElement("gearbox_reference_body"));
    EXPECT_EQ(jointSDF->Get<std::string>("gearbox_reference_body"),
        gearbox_reference_body);
    EXPECT_TRUE(jointSDF->HasElement("gearbox_ratio"));
    EXPECT_DOUBLE_EQ(jointSDF->Get<double>("gearbox_ratio"), gearbox_ratio);
  }

  // screw
  {
    const std::string name("test_screw_joint");
    const msgs::Joint::Type type = msgs::Joint::SCREW;
    const ignition::math::Pose3d pose(
        ignition::math::Vector3d(2, 1, 3),
        ignition::math::Quaterniond(0, 0, 0, 1));
    const std::string parent("parent_screw_link");
    const std::string child("child_screw_link");

    const double cfm = 0.15;
    const double bounce = 0.15;
    const double velocity = 0.25;
    const double fudge_factor = 0.45;
    const double limit_cfm = 0.25;
    const double limit_erp = 0.65;
    const double suspension_cfm = 0.76;
    const double suspension_erp = 0.46;
    const ignition::math::Vector3d xyz1(0.0, 1.0, 1.0);
    const double limit_lower1 = -1.2;
    const double limit_upper1 = 2.2;
    const double limit_effort1 = 1.2e2;
    const double limit_velocity1 = 22;
    const double damping1 = 2.1e-2;
    const double friction1 = 2.6e2;
    const bool useParentModelFrame1 = false;
    const double thread_pitch = 0.9;

    msgs::Joint jointMsg;
    jointMsg.set_name(name);
    jointMsg.set_type(type);
    jointMsg.set_parent(parent);
    jointMsg.set_child(child);
    msgs::Set(jointMsg.mutable_pose(), pose);
    jointMsg.set_cfm(cfm);
    jointMsg.set_bounce(bounce);
    jointMsg.set_velocity(velocity);
    jointMsg.set_fudge_factor(fudge_factor);
    jointMsg.set_limit_cfm(limit_cfm);
    jointMsg.set_limit_erp(limit_erp);
    jointMsg.set_suspension_cfm(suspension_cfm);
    jointMsg.set_suspension_erp(suspension_erp);
    {
      auto axis1 = jointMsg.mutable_axis1();
      msgs::Set(axis1->mutable_xyz(), xyz1);
      axis1->set_limit_lower(limit_lower1);
      axis1->set_limit_upper(limit_upper1);
      axis1->set_limit_effort(limit_effort1);
      axis1->set_limit_velocity(limit_velocity1);
      axis1->set_damping(damping1);
      axis1->set_friction(friction1);
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      axis1->set_use_parent_model_frame(useParentModelFrame1);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
    }

    msgs::Joint::Screw *screwMsg = jointMsg.mutable_screw();
    screwMsg->set_thread_pitch(thread_pitch);

    sdf::ElementPtr jointSDF = msgs::JointToSDF(jointMsg);
    EXPECT_TRUE(jointSDF->HasAttribute("name"));
    EXPECT_EQ(jointSDF->Get<std::string>("name"), name);
    EXPECT_TRUE(jointSDF->HasAttribute("type"));
    EXPECT_STREQ(jointSDF->Get<std::string>("type").c_str(), "screw");
    EXPECT_TRUE(jointSDF->HasElement("parent"));
    EXPECT_EQ(jointSDF->Get<std::string>("parent"), parent);
    EXPECT_TRUE(jointSDF->HasElement("child"));
    EXPECT_EQ(jointSDF->Get<std::string>("child"), child);
    EXPECT_TRUE(jointSDF->HasElement("pose"));
    EXPECT_EQ(pose, jointSDF->Get<ignition::math::Pose3d>("pose"));

    EXPECT_TRUE(jointSDF->HasElement("axis"));
    {
      auto axisElem = jointSDF->GetElement("axis");
      EXPECT_TRUE(axisElem->HasElement("xyz"));
      EXPECT_EQ(xyz1, axisElem->Get<ignition::math::Vector3d>("xyz"));
      EXPECT_TRUE(axisElem->GetElement("xyz")->HasAttribute("expressed_in"));
      EXPECT_EQ("",
                axisElem->GetElement("xyz")->Get<std::string>("expressed_in"));

      EXPECT_TRUE(axisElem->HasElement("dynamics"));
      auto axisDynamics = axisElem->GetElement("dynamics");
      EXPECT_TRUE(axisDynamics->HasElement("damping"));
      EXPECT_DOUBLE_EQ(damping1, axisDynamics->Get<double>("damping"));
      EXPECT_TRUE(axisDynamics->HasElement("friction"));
      EXPECT_DOUBLE_EQ(friction1, axisDynamics->Get<double>("friction"));

      EXPECT_TRUE(axisElem->HasElement("limit"));
      auto axisLimit = axisElem->GetElement("limit");
      EXPECT_TRUE(axisLimit->HasElement("lower"));
      EXPECT_DOUBLE_EQ(limit_lower1, axisLimit->Get<double>("lower"));
      EXPECT_TRUE(axisLimit->HasElement("upper"));
      EXPECT_DOUBLE_EQ(limit_upper1, axisLimit->Get<double>("upper"));
      EXPECT_TRUE(axisLimit->HasElement("effort"));
      EXPECT_DOUBLE_EQ(limit_effort1, axisLimit->Get<double>("effort"));
      EXPECT_TRUE(axisLimit->HasElement("velocity"));
      EXPECT_DOUBLE_EQ(limit_velocity1, axisLimit->Get<double>("velocity"));
    }

    EXPECT_TRUE(jointSDF->HasElement("physics"));
    auto physicsElem = jointSDF->GetElement("physics");
    EXPECT_TRUE(physicsElem->HasElement("ode"));
    auto odePhysics = physicsElem->GetElement("ode");
    EXPECT_TRUE(odePhysics->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("cfm"), cfm);
    EXPECT_TRUE(odePhysics->HasElement("bounce"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("bounce"), bounce);
    EXPECT_TRUE(odePhysics->HasElement("velocity"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("velocity"), velocity);
    EXPECT_TRUE(odePhysics->HasElement("fudge_factor"));
    EXPECT_DOUBLE_EQ(odePhysics->Get<double>("fudge_factor"), fudge_factor);

    EXPECT_TRUE(odePhysics->HasElement("limit"));
    auto limitElem = odePhysics->GetElement("limit");
    EXPECT_TRUE(limitElem->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(limitElem->Get<double>("cfm"), limit_cfm);
    EXPECT_TRUE(limitElem->HasElement("erp"));
    EXPECT_DOUBLE_EQ(limitElem->Get<double>("erp"), limit_erp);

    EXPECT_TRUE(odePhysics->HasElement("suspension"));
    auto suspensionElem = odePhysics->GetElement("suspension");
    EXPECT_TRUE(suspensionElem->HasElement("cfm"));
    EXPECT_DOUBLE_EQ(suspensionElem->Get<double>("cfm"), suspension_cfm);
    EXPECT_TRUE(suspensionElem->HasElement("erp"));
    EXPECT_DOUBLE_EQ(suspensionElem->Get<double>("erp"), suspension_erp);

    EXPECT_TRUE(jointSDF->HasElement("thread_pitch"));
    EXPECT_DOUBLE_EQ(jointSDF->Get<double>("thread_pitch"), thread_pitch);
  }
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, AddBoxLink)
{
  msgs::Model model;
  EXPECT_EQ(model.link_size(), 0);

  const double mass = 1.0;
  const ignition::math::Vector3d size(1, 1, 1);
  msgs::AddBoxLink(model, mass, size);
  EXPECT_EQ(model.link_size(), 1);
  {
    auto link = model.link(0);
    EXPECT_EQ(link.name(), std::string("link_1"));

    auto inertial = link.inertial();
    EXPECT_DOUBLE_EQ(inertial.mass(), mass);
    double ixx = inertial.ixx();
    double iyy = inertial.iyy();
    double izz = inertial.izz();
    double ixy = inertial.ixy();
    double ixz = inertial.ixz();
    double iyz = inertial.iyz();
    EXPECT_GT(ixx, 0.0);
    EXPECT_GT(iyy, 0.0);
    EXPECT_GT(izz, 0.0);
    EXPECT_DOUBLE_EQ(ixy, 0.0);
    EXPECT_DOUBLE_EQ(ixz, 0.0);
    EXPECT_DOUBLE_EQ(iyz, 0.0);
    // triangle inequality
    EXPECT_GE(ixx + iyy, izz);
    EXPECT_GE(iyy + izz, ixx);
    EXPECT_GE(izz + ixx, iyy);

    EXPECT_EQ(link.collision_size(), 1);
    {
      auto collision = link.collision(0);
      auto geometry = collision.geometry();
      EXPECT_EQ(geometry.type(), msgs::Geometry_Type_BOX);
      EXPECT_EQ(msgs::ConvertIgn(geometry.box().size()), size);
    }

    EXPECT_EQ(link.visual_size(), 1);
    {
      auto visual = link.visual(0);
      auto geometry = visual.geometry();
      EXPECT_EQ(geometry.type(), msgs::Geometry_Type_BOX);
      EXPECT_EQ(msgs::ConvertIgn(geometry.box().size()), size);
    }
  }

  const double massRatio = 2.0;
  msgs::AddBoxLink(model, mass*massRatio, size);
  EXPECT_EQ(model.link_size(), 2);
  {
    auto link1 = model.link(0);
    auto link2 = model.link(1);
    EXPECT_EQ(link2.name(), std::string("link_2"));

    auto inertial1 = link1.inertial();
    auto inertial2 = link2.inertial();

    EXPECT_NEAR(massRatio * inertial1.mass(), inertial2.mass(), 1e-6);
    EXPECT_NEAR(massRatio * inertial1.ixx(),  inertial2.ixx(),  1e-6);
    EXPECT_NEAR(massRatio * inertial1.iyy(),  inertial2.iyy(),  1e-6);
    EXPECT_NEAR(massRatio * inertial1.izz(),  inertial2.izz(),  1e-6);
  }
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, AddCylinderLink)
{
  msgs::Model model;
  EXPECT_EQ(model.link_size(), 0);

  const double mass = 1.0;
  const double radius = 0.5;
  const double length = 2.5;
  msgs::AddCylinderLink(model, mass, radius, length);
  EXPECT_EQ(model.link_size(), 1);
  {
    auto link = model.link(0);
    EXPECT_EQ(link.name(), std::string("link_1"));

    auto inertial = link.inertial();
    EXPECT_DOUBLE_EQ(inertial.mass(), mass);
    double ixx = inertial.ixx();
    double iyy = inertial.iyy();
    double izz = inertial.izz();
    double ixy = inertial.ixy();
    double ixz = inertial.ixz();
    double iyz = inertial.iyz();
    EXPECT_GT(ixx, 0.0);
    EXPECT_GT(iyy, 0.0);
    EXPECT_GT(izz, 0.0);
    EXPECT_DOUBLE_EQ(ixy, 0.0);
    EXPECT_DOUBLE_EQ(ixz, 0.0);
    EXPECT_DOUBLE_EQ(iyz, 0.0);
    // triangle inequality
    EXPECT_GE(ixx + iyy, izz);
    EXPECT_GE(iyy + izz, ixx);
    EXPECT_GE(izz + ixx, iyy);

    EXPECT_EQ(link.collision_size(), 1);
    {
      auto collision = link.collision(0);
      auto geometry = collision.geometry();
      EXPECT_EQ(geometry.type(), msgs::Geometry_Type_CYLINDER);
      EXPECT_DOUBLE_EQ(geometry.cylinder().radius(), radius);
      EXPECT_DOUBLE_EQ(geometry.cylinder().length(), length);
    }

    EXPECT_EQ(link.visual_size(), 1);
    {
      auto visual = link.visual(0);
      auto geometry = visual.geometry();
      EXPECT_EQ(geometry.type(), msgs::Geometry_Type_CYLINDER);
      EXPECT_DOUBLE_EQ(geometry.cylinder().radius(), radius);
      EXPECT_DOUBLE_EQ(geometry.cylinder().length(), length);
    }
  }

  const double massRatio = 2.0;
  msgs::AddCylinderLink(model, mass*massRatio, radius, length);
  EXPECT_EQ(model.link_size(), 2);
  {
    auto link1 = model.link(0);
    auto link2 = model.link(1);
    EXPECT_EQ(link2.name(), std::string("link_2"));

    auto inertial1 = link1.inertial();
    auto inertial2 = link2.inertial();

    EXPECT_NEAR(massRatio * inertial1.mass(), inertial2.mass(), 1e-6);
    EXPECT_NEAR(massRatio * inertial1.ixx(),  inertial2.ixx(),  1e-6);
    EXPECT_NEAR(massRatio * inertial1.iyy(),  inertial2.iyy(),  1e-6);
    EXPECT_NEAR(massRatio * inertial1.izz(),  inertial2.izz(),  1e-6);
  }
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, AddSphereLink)
{
  msgs::Model model;
  EXPECT_EQ(model.link_size(), 0);

  const double mass = 1.0;
  const double radius = 0.5;
  msgs::AddSphereLink(model, mass, radius);
  EXPECT_EQ(model.link_size(), 1);
  {
    auto link = model.link(0);
    EXPECT_EQ(link.name(), std::string("link_1"));

    auto inertial = link.inertial();
    EXPECT_DOUBLE_EQ(inertial.mass(), mass);
    double ixx = inertial.ixx();
    double iyy = inertial.iyy();
    double izz = inertial.izz();
    double ixy = inertial.ixy();
    double ixz = inertial.ixz();
    double iyz = inertial.iyz();
    EXPECT_GT(ixx, 0.0);
    EXPECT_GT(iyy, 0.0);
    EXPECT_GT(izz, 0.0);
    EXPECT_DOUBLE_EQ(ixy, 0.0);
    EXPECT_DOUBLE_EQ(ixz, 0.0);
    EXPECT_DOUBLE_EQ(iyz, 0.0);
    // triangle inequality
    EXPECT_GE(ixx + iyy, izz);
    EXPECT_GE(iyy + izz, ixx);
    EXPECT_GE(izz + ixx, iyy);

    EXPECT_EQ(link.collision_size(), 1);
    {
      auto collision = link.collision(0);
      auto geometry = collision.geometry();
      EXPECT_EQ(geometry.type(), msgs::Geometry_Type_SPHERE);
      EXPECT_DOUBLE_EQ(geometry.sphere().radius(), radius);
    }

    EXPECT_EQ(link.visual_size(), 1);
    {
      auto visual = link.visual(0);
      auto geometry = visual.geometry();
      EXPECT_EQ(geometry.type(), msgs::Geometry_Type_SPHERE);
      EXPECT_DOUBLE_EQ(geometry.sphere().radius(), radius);
    }
  }

  const double massRatio = 2.0;
  msgs::AddSphereLink(model, mass*massRatio, radius);
  EXPECT_EQ(model.link_size(), 2);
  {
    auto link1 = model.link(0);
    auto link2 = model.link(1);
    EXPECT_EQ(link2.name(), std::string("link_2"));

    auto inertial1 = link1.inertial();
    auto inertial2 = link2.inertial();

    EXPECT_NEAR(massRatio * inertial1.mass(), inertial2.mass(), 1e-6);
    EXPECT_NEAR(massRatio * inertial1.ixx(),  inertial2.ixx(),  1e-6);
    EXPECT_NEAR(massRatio * inertial1.iyy(),  inertial2.iyy(),  1e-6);
    EXPECT_NEAR(massRatio * inertial1.izz(),  inertial2.izz(),  1e-6);
  }
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, ModelToSDF)
{
  const std::string name("test_bicycle");
  const ignition::math::Pose3d pose(
      ignition::math::Vector3d(6, 1, 7),
      ignition::math::Quaterniond(0.5, 0.5, 0.5, 0.5));

  msgs::Model model;
  model.set_name(name);
  model.set_is_static(false);
  msgs::Set(model.mutable_pose(), pose);
  EXPECT_EQ(model.link_size(), 0);
  EXPECT_EQ(model.joint_size(), 0);

  // This will be a bicycle with two wheels.
  // The frame is a box.
  const double length = 1.5;
  const double height = 0.9;
  const double width = 0.1;
  const ignition::math::Vector3d boxSize(length, width, height);
  const double boxMass = 4.0;
  AddBoxLink(model, boxMass, boxSize);
  ASSERT_EQ(model.link_size(), 1);
  EXPECT_EQ(model.joint_size(), 0);
  model.mutable_link(0)->set_name("frame");

  // The rear wheel is a cylinder.
  const double radius = height / 2;
  AddCylinderLink(model, 0.5, radius, radius);
  ASSERT_EQ(model.link_size(), 2);
  EXPECT_EQ(model.joint_size(), 0);
  const ignition::math::Pose3d cylinderPose(
      -length/2, 0, -height/2, M_PI/2, 0, 0);
  {
    auto link = model.mutable_link(1);
    msgs::Set(link->mutable_pose(), cylinderPose);
    link->set_name("rear_wheel");
  }

  // The front wheel is a sphere.
  AddSphereLink(model, 0.5, radius);
  ASSERT_EQ(model.link_size(), 3);
  EXPECT_EQ(model.joint_size(), 0);
  const ignition::math::Pose3d spherePose(length/2, 0, -height/2, 0, 0, 0);
  {
    auto link = model.mutable_link(2);
    msgs::Set(link->mutable_pose(), spherePose);
    link->set_name("front_wheel");
  }

  // Add revolute joints for the wheels.
  // Front wheel joint
  model.add_joint();
  ASSERT_EQ(model.joint_size(), 1);
  auto frontJoint = model.mutable_joint(0);
  frontJoint->set_name("front_hinge");
  frontJoint->set_type(msgs::ConvertJointType("revolute"));
  frontJoint->set_parent("frame");
  frontJoint->set_child("front_wheel");
  const ignition::math::Vector3d frontAxis(0, 1, 0);
  msgs::Set(frontJoint->mutable_axis1()->mutable_xyz(), frontAxis);

  // Rear wheel joint
  model.add_joint();
  ASSERT_EQ(model.joint_size(), 2);
  auto rearJoint = model.mutable_joint(1);
  rearJoint->set_name("rear_hinge");
  rearJoint->set_type(msgs::ConvertJointType("revolute"));
  rearJoint->set_parent("frame");
  rearJoint->set_child("rear_wheel");
  const ignition::math::Vector3d rearAxis(0, 0, 1);
  msgs::Set(rearJoint->mutable_axis1()->mutable_xyz(), rearAxis);

  // Add plugin
  model.add_plugin();
  ASSERT_EQ(model.plugin_size(), 1);
  auto plugin = model.mutable_plugin(0);
  plugin->set_name("plugin_name");
  plugin->set_filename("plugin_filename");
  plugin->set_innerxml("<plugin_param>param</plugin_param>");

  sdf::ElementPtr modelSDF = msgs::ModelToSDF(model);
  EXPECT_EQ(modelSDF->Get<std::string>("name"), name);
  EXPECT_FALSE(modelSDF->Get<bool>("static"));
  EXPECT_EQ(pose, modelSDF->Get<ignition::math::Pose3d>("pose"));

  sdf::ElementPtr linkElem1 = modelSDF->GetElement("link");
  EXPECT_EQ(linkElem1->Get<std::string>("name"), "frame");
  EXPECT_EQ(linkElem1->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d());

  sdf::ElementPtr linkElem2 = linkElem1->GetNextElement("link");
  EXPECT_EQ(linkElem2->Get<std::string>("name"), "rear_wheel");
  EXPECT_EQ(linkElem2->Get<ignition::math::Pose3d>("pose"), cylinderPose);

  sdf::ElementPtr linkElem3 = linkElem2->GetNextElement("link");
  EXPECT_EQ(linkElem3->Get<std::string>("name"), "front_wheel");
  EXPECT_EQ(linkElem3->Get<ignition::math::Pose3d>("pose"), spherePose);

  sdf::ElementPtr jointElem1 = modelSDF->GetElement("joint");
  EXPECT_EQ(jointElem1->Get<std::string>("name"), "front_hinge");
  EXPECT_EQ(jointElem1->Get<std::string>("type"), "revolute");
  EXPECT_EQ(jointElem1->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d());

  sdf::ElementPtr jointElem2 = jointElem1->GetNextElement("joint");
  EXPECT_EQ(jointElem2->Get<std::string>("name"), "rear_hinge");
  EXPECT_EQ(jointElem2->Get<std::string>("type"), "revolute");
  EXPECT_EQ(jointElem2->Get<ignition::math::Pose3d>("pose"),
      ignition::math::Pose3d());

  sdf::ElementPtr pluginElem = modelSDF->GetElement("plugin");
  EXPECT_EQ(pluginElem->Get<std::string>("name"), "plugin_name");
  EXPECT_EQ(pluginElem->Get<std::string>("filename"), "plugin_filename");
  EXPECT_TRUE(pluginElem->HasElement("plugin_param"));
  EXPECT_EQ(pluginElem->Get<std::string>("plugin_param"), "param");
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, PluginToSDF)
{
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<plugin_param>param</plugin_param>";

  msgs::Plugin msg;
  msg.set_name(name);
  msg.set_filename(filename);
  msg.set_innerxml(innerxml);

  sdf::ElementPtr pluginSDF = msgs::PluginToSDF(msg);

  EXPECT_TRUE(pluginSDF->HasAttribute("name"));
  EXPECT_EQ(pluginSDF->Get<std::string>("name"), name);

  EXPECT_TRUE(pluginSDF->HasAttribute("filename"));
  EXPECT_EQ(pluginSDF->Get<std::string>("filename"), filename);

  EXPECT_TRUE(pluginSDF->HasElement("plugin_param"));
  EXPECT_EQ(pluginSDF->Get<std::string>("plugin_param"), "param");
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, PluginToFromSDF)
{
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<plugin_param>param</plugin_param>\n";

  // Create message
  msgs::Plugin msg1;
  msg1.set_name(name);
  msg1.set_filename(filename);
  msg1.set_innerxml(innerxml);

  // To SDF
  sdf::ElementPtr sdf1 = msgs::PluginToSDF(msg1);

  // Back to Msg
  msgs::Plugin msg2 = msgs::PluginFromSDF(sdf1);
  EXPECT_EQ(msg2.name(), name);
  EXPECT_EQ(msg2.filename(), filename);
  EXPECT_EQ(msg2.innerxml(), innerxml);

  // Back to SDF
  sdf::ElementPtr sdf2;
  sdf2.reset(new sdf::Element);
  sdf::initFile("plugin.sdf", sdf2);
  msgs::PluginToSDF(msg2, sdf2);
  EXPECT_TRUE(sdf2 != nullptr);
  EXPECT_EQ(sdf2->Get<std::string>("name"), name);
  EXPECT_EQ(sdf2->Get<std::string>("filename"), filename);
  EXPECT_TRUE(sdf2->HasElement("plugin_param"));
  EXPECT_EQ(sdf2->Get<std::string>("plugin_param"), "param");
}

/////////////////////////////////////////////////
TEST_F(MsgsTest, VisualPluginToFromSDF)
{
  const std::string visName("visual_name");
  const double radius = 3.3;
  std::string pluginName("plugin_name");
  std::string filename("plugin_filename");
  std::string innerxml("<plugin_param>param</plugin_param>\n");

  // Create SDF
  sdf::ElementPtr sdf1(new sdf::Element());
  sdf::initFile("visual.sdf", sdf1);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <visual name='" + visName + "'>\
           <geometry>\
             <sphere><radius>" + std::to_string(radius) + "</radius></sphere>\
           </geometry>\
           <plugin name='" + pluginName + "' filename='" + filename + "'>\
             " + innerxml + "\
           </plugin>\
         </visual>\
      </sdf>", sdf1));

  // To msg
  auto msg1 = msgs::VisualFromSDF(sdf1);

  // Back to SDF
  auto sdf2 = msgs::VisualToSDF(msg1);
  ASSERT_TRUE(sdf2 != nullptr);

  // Back to Msg
  auto msg2 = msgs::VisualFromSDF(sdf2);

  // Back to SDF
  auto sdf3 = msgs::VisualToSDF(msg2);
  ASSERT_TRUE(sdf3 != nullptr);

  ASSERT_TRUE(sdf3->HasAttribute("name"));
  EXPECT_EQ(sdf3->Get<std::string>("name"), visName);

  ASSERT_TRUE(sdf3->HasElement("geometry"));
  auto geomElem = sdf3->GetElement("geometry");
  ASSERT_TRUE(geomElem->HasElement("sphere"));
  auto sphereElem = geomElem->GetElement("sphere");
  EXPECT_TRUE(sphereElem->HasElement("radius"));
  EXPECT_DOUBLE_EQ(sphereElem->Get<double>("radius"), radius);

  ASSERT_TRUE(sdf3->HasElement("plugin"));
  auto pluginElem = sdf3->GetElement("plugin");
  EXPECT_EQ(pluginElem->Get<std::string>("name"), pluginName);
  EXPECT_EQ(pluginElem->Get<std::string>("filename"), filename);

  EXPECT_TRUE(pluginElem->HasElement("plugin_param"));
  EXPECT_EQ(pluginElem->Get<std::string>("plugin_param"), "param");
}

TEST_F(MsgsTest, ConvertIgnMsgColor)
{
  ignition::msgs::Color ignMsg;
  ignMsg.set_r(0.1);
  ignMsg.set_g(0.2);
  ignMsg.set_b(0.3);
  ignMsg.set_a(0.4);

  auto gzMsg = msgs::ConvertIgnMsg(ignMsg);
  auto ignMsg2 = msgs::ConvertIgnMsg(gzMsg);

  EXPECT_DOUBLE_EQ(ignMsg.r(), ignMsg2.r());
  EXPECT_DOUBLE_EQ(ignMsg.g(), ignMsg2.g());
  EXPECT_DOUBLE_EQ(ignMsg.b(), ignMsg2.b());
  EXPECT_DOUBLE_EQ(ignMsg.a(), ignMsg2.a());
}

TEST_F(MsgsTest, ConvertIgnMsgMaterialShaderType)
{
  auto ignMsg = ignition::msgs::Material::PIXEL;

  auto gzMsg = msgs::ConvertIgnMsg(ignMsg);
  auto ignMsg2 = msgs::ConvertIgnMsg(gzMsg);

  EXPECT_DOUBLE_EQ(ignMsg, ignMsg2);
}

TEST_F(MsgsTest, ConvertIgnMsgMaterialScript)
{
  ignition::msgs::Material::Script ignMsg;
  ignMsg.set_name("material_script");

  for (size_t i = 0; i < 3; ++i)
    ignMsg.add_uri("uri_" + std::to_string(i));

  auto gzMsg = msgs::ConvertIgnMsg(ignMsg);
  auto ignMsg2 = msgs::ConvertIgnMsg(gzMsg);

  EXPECT_EQ(ignMsg.name(), ignMsg2.name());
  ASSERT_EQ(ignMsg2.uri().size(), 3);

  for (size_t i = 0; i < 3; ++i)
    EXPECT_EQ(ignMsg.uri(i), ignMsg2.uri(i));
}

TEST_F(MsgsTest, ConvertIgnMsgMaterial)
{
  ignition::msgs::Material ignMsg;

  auto script = ignMsg.mutable_script();
  script->set_name("script");
  script->add_uri("uri");

  ignMsg.set_shader_type(ignition::msgs::Material::PIXEL);
  ignMsg.set_normal_map("normal_map");

  auto ambient = ignMsg.mutable_ambient();
  ambient->set_r(0.1);
  ambient->set_g(0.2);
  ambient->set_b(0.3);
  ambient->set_a(0.4);

  ignMsg.set_lighting(false);

  auto gzMsg = msgs::ConvertIgnMsg(ignMsg);
  auto ignMsg2 = msgs::ConvertIgnMsg(gzMsg);

  EXPECT_EQ(ignMsg.script().name(), ignMsg2.script().name());
  EXPECT_EQ(ignMsg.script().uri(0), ignMsg2.script().uri(0));
  EXPECT_EQ(ignMsg.normal_map(), ignMsg2.normal_map());
  EXPECT_DOUBLE_EQ(ignMsg.ambient().r(), ignMsg2.ambient().r());
  EXPECT_DOUBLE_EQ(ignMsg.ambient().g(), ignMsg2.ambient().g());
  EXPECT_DOUBLE_EQ(ignMsg.ambient().b(), ignMsg2.ambient().b());
  EXPECT_DOUBLE_EQ(ignMsg.ambient().a(), ignMsg2.ambient().a());
  EXPECT_EQ(ignMsg.lighting(), ignMsg2.lighting());
}
