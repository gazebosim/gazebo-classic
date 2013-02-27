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
#include "msgs/msgs.hh"

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

TEST(MsgTest, Request)
{
  msgs::Request *request = msgs::CreateRequest("help", "me");
  EXPECT_STREQ("help", request->request().c_str());
  EXPECT_STREQ("me", request->data().c_str());
  EXPECT_GT(request->id(), 0);
}

TEST(MsgTest, Time)
{
  common::Time t = common::Time::GetWallTime();
  msgs::Time msg;
  msgs::Stamp(&msg);
  EXPECT_EQ(t.sec, msg.sec());
  EXPECT_TRUE(t.nsec <= msg.nsec());
}


TEST(MsgTest, TimeFromHeader)
{
  common::Time t = common::Time::GetWallTime();
  msgs::Header msg;
  msgs::Stamp(&msg);
  EXPECT_EQ(t.sec, msg.stamp().sec());
  EXPECT_TRUE(t.nsec <= msg.stamp().nsec());
}


TEST(MsgTest, Packet)
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

TEST(MsgTest, BadPackage)
{
  msgs::GzString msg;
  EXPECT_THROW(msgs::Package("test_type", msg), common::Exception);
}

TEST(MsgTest, CovertMathVector3ToMsgs)
{
  msgs::Vector3d msg = msgs::Convert(math::Vector3(1, 2, 3));
  EXPECT_EQ(1, msg.x());
  EXPECT_EQ(2, msg.y());
  EXPECT_EQ(3, msg.z());
}

TEST(MsgTest, ConvertMsgsVector3dToMath)
{
  msgs::Vector3d msg = msgs::Convert(math::Vector3(1, 2, 3));
  math::Vector3 v    = msgs::Convert(msg);
  EXPECT_EQ(1, v.x);
  EXPECT_EQ(2, v.y);
  EXPECT_EQ(3, v.z);
}

TEST(MsgTest, ConvertMathQuaterionToMsgs)
{
  msgs::Quaternion msg =
    msgs::Convert(math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));

  EXPECT_TRUE(math::equal(msg.x(), -0.65328148243818818));
  EXPECT_TRUE(math::equal(msg.y(), 0.27059805007309856));
  EXPECT_TRUE(math::equal(msg.z(), 0.65328148243818829));
  EXPECT_TRUE(math::equal(msg.w(), 0.27059805007309851));
}

TEST(MsgTest, ConvertMsgsQuaterionToMath)
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

TEST(MsgTest, ConvertPoseMathToMsgs)
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
}

TEST(TestMsg, ConvertMsgPoseToMath)
{
  msgs::Pose msg = msgs::Convert(math::Pose(math::Vector3(1, 2, 3),
        math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI)));
  math::Pose v = msgs::Convert(msg);

  EXPECT_EQ(1, v.pos.x);
  EXPECT_EQ(2, v.pos.y);
  EXPECT_EQ(3, v.pos.z);
  EXPECT_TRUE(math::equal(v.rot.x, -0.65328148243818818));
  EXPECT_TRUE(math::equal(v.rot.y, 0.27059805007309856));
  EXPECT_TRUE(math::equal(v.rot.z, 0.65328148243818829));
  EXPECT_TRUE(math::equal(v.rot.w, 0.27059805007309851));
}

TET(MsgTest, ConvertCommonColorToMsgs)
{
  msgs::Color msg = msgs::Convert(common::Color(.1, .2, .3, 1.0));

  EXPECT_TRUE(math::equal(0.1f, msg.r()));
  EXPECT_TRUE(math::equal(0.2f, msg.g()));
  EXPECT_TRUE(math::equal(0.3f, msg.b()));
  EXPECT_TRUE(math::equal(1.0f, msg.a()));
}

TEST(MsgTest, ConvertMsgsColorToCommon)
{
  msgs::Color msg = msgs::Convert(common::Color(.1, .2, .3, 1.0));
  common::Color v = msgs::Convert(msg);

  EXPECT_TRUE(math::equal(0.1f, v.r));
  EXPECT_TRUE(math::equal(0.2f, v.g));
  EXPECT_TRUE(math::equal(0.3f, v.b));
  EXPECT_TRUE(math::equal(1.0f, v.a));
}

TEST(MsgTest, ConvertCommonTimeToMsgs)
{
  msgs::Time msg = msgs::Convert(common::Time(2, 123));
  EXPECT_EQ(2, msg.sec());
  EXPECT_EQ(123, msg.nsec());

  common::Time v = msgs::Convert(msg);
  EXPECT_EQ(2, v.sec);
  EXPECT_EQ(123, v.nsec);
}

TEST(MsgTest, ConvertMathPlaneToMsgs)
{
  msgs::PlaneGeom msg = msgs::Convert(math::Plane(math::Vector3(0, 0, 1),
        math::Vector2d(123, 456), 1.0));

  EXPECT_EQ(0, msg.normal().x());
  EXPECT_EQ(0, msg.normal().y());
  EXPECT_EQ(1, msg.normal().z());

  EXPECT_EQ(123, msg.size().x());
  EXPECT_EQ(456, msg.size().y());
}

TEST(MsgTest, ConvertMsgsPlaneToMath)
{
  msgs::PlaneGeom msg = msgs::Convert(math::Plane(math::Vector3(0, 0, 1),
        math::Vector2d(123, 456), 1.0));
  math::Plane v = msgs::Convert(msg);

  EXPECT_EQ(0, v.normal.x);
  EXPECT_EQ(0, v.normal.y);
  EXPECT_EQ(1, v.normal.z);

  EXPECT_EQ(123, v.size.x);
  EXPECT_EQ(456, v.size.y);

  EXPECT_TRUE(math::equal(1.0, v.d));
}

TEST(MsgTest, SetVector3)
{
  msgs::Vector3d msg;
  msgs::Set(&msg, math::Vector3(1, 2, 3));
  EXPECT_EQ(1, msg.x());
  EXPECT_EQ(2, msg.y());
  EXPECT_EQ(3, msg.z());
}

TEST(MsgTest, SetVector2d)
{
  msgs::Vector2d msg;
  msgs::Set(&msg, math::Vector2d(1, 2));
  EXPECT_EQ(1, msg.x());
  EXPECT_EQ(2, msg.y());
}

TEST(MsgTest, SetQuaternion)
{
  msgs::Quaternion msg;
  msgs::Set(&msg, math::Quaternion(M_PI * 0.25, M_PI * 0.5, M_PI));
  EXPECT_TRUE(math::equal(msg.x(), -0.65328148243818818));
  EXPECT_TRUE(math::equal(msg.y(), 0.27059805007309856));
  EXPECT_TRUE(math::equal(msg.z(), 0.65328148243818829));
  EXPECT_TRUE(math::equal(msg.w(), 0.27059805007309851));
}

TEST(MsgTest, SetPose)
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

TEST(MsgTest, SetColor)
{
  msgs::Color msg;
  msgs::Set(&msg, common::Color(.1, .2, .3, 1.0));
  EXPECT_TRUE(math::equal(0.1f, msg.r()));
  EXPECT_TRUE(math::equal(0.2f, msg.g()));
  EXPECT_TRUE(math::equal(0.3f, msg.b()));
  EXPECT_TRUE(math::equal(1.0f, msg.a()));
}

TEST(MsgTest, SetTime)
{
  msgs::Time msg;
  msgs::Set(&msg, common::Time(2, 123));
  EXPECT_EQ(2, msg.sec());
  EXPECT_EQ(123, msg.nsec());
}

TEST(MsgTest, SetPlane)
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
