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

#include "gazebo/math/Box.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/ConfigWidget_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ConfigWidget_TEST::EmptyMsgWidget()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;
  visualConfigWidget->Load(&visualMsg);
  gazebo::msgs::Visual *retVisualMsg =
      dynamic_cast<gazebo::msgs::Visual *>(visualConfigWidget->GetMsg());
  QVERIFY(retVisualMsg != NULL);
  delete visualConfigWidget;

  gazebo::gui::ConfigWidget *collisionConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Collision collisionMsg;
  collisionConfigWidget->Load(&collisionMsg);
  gazebo::msgs::Collision *retCollisionMsg =
      dynamic_cast<gazebo::msgs::Collision *>(collisionConfigWidget->GetMsg());
  QVERIFY(retCollisionMsg != NULL);
  delete collisionConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::JointMsgWidget()
{
  gazebo::gui::ConfigWidget *jointConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Joint jointMsg;

  {
    // joint
    jointMsg.set_name("test_joint");
    jointMsg.set_id(1122u);
    jointMsg.set_parent("test_joint_parent");
    jointMsg.set_parent_id(212121u);
    jointMsg.set_child("test_joint_child");
    jointMsg.set_child_id(454545u);

    // type
    jointMsg.set_type(gazebo::msgs::ConvertJointType("revolute"));

    // pose
    ignition::math::Vector3d pos(4.0, -1.0, 3.5);
    ignition::math::Quaterniond quat(0.0, 1.57, 0.0);
    gazebo::msgs::Set(jointMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // axis1
    gazebo::msgs::Axis *axisMsg = jointMsg.mutable_axis1();
    gazebo::msgs::Set(axisMsg->mutable_xyz(), ignition::math::Vector3d::UnitX);
    axisMsg->set_use_parent_model_frame(false);
    axisMsg->set_limit_lower(-999.0);
    axisMsg->set_limit_upper(999.0);
    axisMsg->set_limit_effort(-1.0);
    axisMsg->set_limit_velocity(-1.0);
    axisMsg->set_damping(0.0);

    // other joint physics properties
    jointMsg.set_cfm(0.2);
    jointMsg.set_bounce(0.3);
    jointMsg.set_velocity(0.4);
    jointMsg.set_fudge_factor(0.5);
    jointMsg.set_limit_cfm(0.6);
    jointMsg.set_limit_erp(0.7);
    jointMsg.set_suspension_cfm(0.8);
    jointMsg.set_suspension_erp(0.9);
  }
  jointConfigWidget->Load(&jointMsg);

  // retrieve the message from the config widget and
  // verify that all values have not been changed.
  {
    gazebo::msgs::Joint *retJointMsg =
        dynamic_cast<gazebo::msgs::Joint *>(jointConfigWidget->GetMsg());
    QVERIFY(retJointMsg != NULL);

    // joint
    QVERIFY(retJointMsg->name() == "test_joint");
    QCOMPARE(retJointMsg->id(), 1122u);
    QVERIFY(retJointMsg->parent() == "test_joint_parent");
    QCOMPARE(retJointMsg->parent_id(), 212121u);
    QVERIFY(retJointMsg->child() == "test_joint_child");
    QCOMPARE(retJointMsg->child_id(), 454545u);

    // type
    QCOMPARE(retJointMsg->type(), gazebo::msgs::ConvertJointType("revolute"));

    // pose
    const gazebo::msgs::Pose poseMsg = retJointMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), 4.0);
    QCOMPARE(posMsg.y(), -1.0);
    QCOMPARE(posMsg.z(), 3.5);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    gazebo::math::Quaternion quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.GetAsEuler().x, 0.0);
    QCOMPARE(quat.GetAsEuler().y, 1.57);
    QCOMPARE(quat.GetAsEuler().z, 0.0);

    // axis1
    gazebo::msgs::Axis *axisMsg = jointMsg.mutable_axis1();
    QCOMPARE(axisMsg->xyz().x(), 1.0);
    QCOMPARE(axisMsg->xyz().y(), 0.0);
    QCOMPARE(axisMsg->xyz().z(), 0.0);
    QCOMPARE(axisMsg->use_parent_model_frame(), false);
    QCOMPARE(axisMsg->limit_lower(), -999.0);
    QCOMPARE(axisMsg->limit_upper(), 999.0);
    QCOMPARE(axisMsg->limit_effort(), -1.0);
    QCOMPARE(axisMsg->limit_velocity(), -1.0);
    QCOMPARE(axisMsg->damping(), 0.0);

    // other joint physics properties
    QCOMPARE(retJointMsg->cfm(), 0.2);
    QCOMPARE(retJointMsg->bounce(), 0.3);
    QCOMPARE(retJointMsg->velocity(), 0.4);
    QCOMPARE(retJointMsg->fudge_factor(), 0.5);
    QCOMPARE(retJointMsg->limit_cfm(), 0.6);
    QCOMPARE(retJointMsg->limit_erp(), 0.7);
    QCOMPARE(retJointMsg->suspension_cfm(), 0.8);
    QCOMPARE(retJointMsg->suspension_erp(), 0.9);
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  // Joint type revolute -> universal
  {
    // joint
    jointConfigWidget->SetStringWidgetValue("name", "test_joint_updated");
    jointConfigWidget->SetUIntWidgetValue("id", 9999999u);
    jointConfigWidget->SetStringWidgetValue("parent",
        "test_joint_parent_updated");
    jointConfigWidget->SetUIntWidgetValue("parent_id", 1u);
    jointConfigWidget->SetStringWidgetValue("child",
        "test_joint_child_updated");
    jointConfigWidget->SetUIntWidgetValue("child_id", 2u);

    // type
    jointConfigWidget->SetEnumWidgetValue("type",
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_UNIVERSAL));

    // pose
    gazebo::math::Vector3 pos(2.0, 9.0, -4.0);
    gazebo::math::Quaternion quat(0.0, 0.0, 1.57);
    jointConfigWidget->SetPoseWidgetValue("pose",
        gazebo::math::Pose(pos, quat));

    // axis1
    jointConfigWidget->SetVector3WidgetValue("axis1::xyz",
        gazebo::math::Vector3::UnitY);
    jointConfigWidget->SetBoolWidgetValue("axis1::use_parent_model_frame",
        true);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_lower", -1.2);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_upper", -1.0);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_effort", 1.0);
    jointConfigWidget->SetDoubleWidgetValue("axis1::limit_velocity", 100.0);
    jointConfigWidget->SetDoubleWidgetValue("axis1::damping", 0.9);

    // axis2
    jointConfigWidget->SetVector3WidgetValue("axis2::xyz",
        gazebo::math::Vector3::UnitZ);
    jointConfigWidget->SetBoolWidgetValue("axis2::use_parent_model_frame",
        true);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_lower", -3.2);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_upper", -3.0);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_effort", 3.0);
    jointConfigWidget->SetDoubleWidgetValue("axis2::limit_velocity", 300.0);
    jointConfigWidget->SetDoubleWidgetValue("axis2::damping", 3.9);

    // other joint physics properties
    jointConfigWidget->SetDoubleWidgetValue("cfm", 0.9);
    jointConfigWidget->SetDoubleWidgetValue("bounce", 0.8);
    jointConfigWidget->SetDoubleWidgetValue("velocity", 0.7);
    jointConfigWidget->SetDoubleWidgetValue("fudge_factor", 0.6);
    jointConfigWidget->SetDoubleWidgetValue("limit_cfm", 0.5);
    jointConfigWidget->SetDoubleWidgetValue("limit_erp", 0.4);
    jointConfigWidget->SetDoubleWidgetValue("suspension_cfm", 0.3);
    jointConfigWidget->SetDoubleWidgetValue("suspension_erp", 0.2);
  }

  // verify widget values
  {
    // joint
    QVERIFY(jointConfigWidget->GetStringWidgetValue("name") ==
        "test_joint_updated");
    QCOMPARE(jointConfigWidget->GetUIntWidgetValue("id"), 9999999u);
    QVERIFY(jointConfigWidget->GetStringWidgetValue("parent") ==
        "test_joint_parent_updated");
    QCOMPARE(jointConfigWidget->GetUIntWidgetValue("parent_id"), 1u);
    QVERIFY(jointConfigWidget->GetStringWidgetValue("child") ==
        "test_joint_child_updated");
    QCOMPARE(jointConfigWidget->GetUIntWidgetValue("child_id"), 2u);

    // type
    QCOMPARE(jointConfigWidget->GetEnumWidgetValue("type"),
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_UNIVERSAL));

    // pose
    gazebo::math::Vector3 pos(2.0, 9.0, -4.0);
    gazebo::math::Quaternion quat(0.0, 0.0, 1.57);
    QCOMPARE(jointConfigWidget->GetPoseWidgetValue("pose"),
        gazebo::math::Pose(pos, quat));

    // axis1
    QCOMPARE(jointConfigWidget->GetVector3WidgetValue("axis1::xyz"),
        gazebo::math::Vector3::UnitY);
    QCOMPARE(jointConfigWidget->GetBoolWidgetValue(
        "axis1::use_parent_model_frame"), true);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis1::limit_lower"),
        -1.2);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis1::limit_upper"),
        -1.0);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis1::limit_effort"),
        1.0);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis1::limit_velocity"),
        100.0);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis1::damping"), 0.9);

    // axis2
    QCOMPARE(jointConfigWidget->GetVector3WidgetValue("axis2::xyz"),
        gazebo::math::Vector3::UnitZ);
    QCOMPARE(jointConfigWidget->GetBoolWidgetValue(
        "axis1::use_parent_model_frame"), true);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis2::limit_lower"),
        -3.2);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis2::limit_upper"),
        -3.0);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis2::limit_effort"),
        3.0);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis2::limit_velocity"),
        300.0);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("axis2::damping"), 3.9);

    // other joint physics properties
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("cfm"), 0.9);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("bounce"), 0.8);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("velocity"), 0.7);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("fudge_factor"), 0.6);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("limit_cfm"), 0.5);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("limit_erp"), 0.4);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("suspension_cfm"), 0.3);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("suspension_erp"), 0.2);
  }

  // verify updates in new msg
  {
    gazebo::msgs::Joint *retJointMsg =
        dynamic_cast<gazebo::msgs::Joint *>(jointConfigWidget->GetMsg());
    QVERIFY(retJointMsg != NULL);

    // joint
    QVERIFY(retJointMsg->name() == "test_joint_updated");
    QCOMPARE(retJointMsg->id(), 9999999u);
    QVERIFY(retJointMsg->parent() == "test_joint_parent_updated");
    QCOMPARE(retJointMsg->parent_id(), 1u);
    QVERIFY(retJointMsg->child() == "test_joint_child_updated");
    QCOMPARE(retJointMsg->child_id(), 2u);

    // type
    QCOMPARE(retJointMsg->type(), gazebo::msgs::ConvertJointType("universal"));

    // pose
    const gazebo::msgs::Pose poseMsg = retJointMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), 2.0);
    QCOMPARE(posMsg.y(), 9.0);
    QCOMPARE(posMsg.z(), -4.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    gazebo::math::Quaternion quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.GetAsEuler().x, 0.0);
    QCOMPARE(quat.GetAsEuler().y, 0.0);
    QCOMPARE(quat.GetAsEuler().z, 1.57);

    // axis1
    gazebo::msgs::Axis *axisMsg = retJointMsg->mutable_axis1();
    QCOMPARE(axisMsg->xyz().x(), 0.0);
    QCOMPARE(axisMsg->xyz().y(), 1.0);
    QCOMPARE(axisMsg->xyz().z(), 0.0);
    QCOMPARE(axisMsg->use_parent_model_frame(), true);
    QCOMPARE(axisMsg->limit_lower(), -1.2);
    QCOMPARE(axisMsg->limit_upper(), -1.0);
    QCOMPARE(axisMsg->limit_effort(), 1.0);
    QCOMPARE(axisMsg->limit_velocity(), 100.0);
    QCOMPARE(axisMsg->damping(), 0.9);

    // axis2
    gazebo::msgs::Axis *axis2Msg = retJointMsg->mutable_axis2();
    QCOMPARE(axis2Msg->xyz().x(), 0.0);
    QCOMPARE(axis2Msg->xyz().y(), 0.0);
    QCOMPARE(axis2Msg->xyz().z(), 1.0);
    QCOMPARE(axis2Msg->use_parent_model_frame(), true);
    QCOMPARE(axis2Msg->limit_lower(), -3.2);
    QCOMPARE(axis2Msg->limit_upper(), -3.0);
    QCOMPARE(axis2Msg->limit_effort(), 3.0);
    QCOMPARE(axis2Msg->limit_velocity(), 300.0);
    QCOMPARE(axis2Msg->damping(), 3.9);

    // other joint physics properties
    QCOMPARE(retJointMsg->cfm(), 0.9);
    QCOMPARE(retJointMsg->bounce(), 0.8);
    QCOMPARE(retJointMsg->velocity(), 0.7);
    QCOMPARE(retJointMsg->fudge_factor(), 0.6);
    QCOMPARE(retJointMsg->limit_cfm(), 0.5);
    QCOMPARE(retJointMsg->limit_erp(), 0.4);
    QCOMPARE(retJointMsg->suspension_cfm(), 0.3);
    QCOMPARE(retJointMsg->suspension_erp(), 0.2);
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  // Joint type universal -> ball
  {
    // joint
    jointConfigWidget->SetStringWidgetValue("name", "test_joint_updated2");
    jointConfigWidget->SetUIntWidgetValue("id", 2222222u);
    jointConfigWidget->SetStringWidgetValue("parent",
        "test_joint_parent_updated2");
    jointConfigWidget->SetUIntWidgetValue("parent_id", 10u);
    jointConfigWidget->SetStringWidgetValue("child",
        "test_joint_child_updated2");
    jointConfigWidget->SetUIntWidgetValue("child_id", 20u);

    // type
    jointConfigWidget->SetEnumWidgetValue("type",
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_BALL));

    // pose
    gazebo::math::Vector3 pos(-2.0, 1.0, 2.0);
    gazebo::math::Quaternion quat(0.0, 0.0, 0.0);
    jointConfigWidget->SetPoseWidgetValue("pose",
        gazebo::math::Pose(pos, quat));

    // other joint physics properties
    jointConfigWidget->SetDoubleWidgetValue("cfm", 0.19);
    jointConfigWidget->SetDoubleWidgetValue("bounce", 0.18);
    jointConfigWidget->SetDoubleWidgetValue("velocity", 2.7);
    jointConfigWidget->SetDoubleWidgetValue("fudge_factor", 0.26);
    jointConfigWidget->SetDoubleWidgetValue("limit_cfm", 0.15);
    jointConfigWidget->SetDoubleWidgetValue("limit_erp", 0.24);
    jointConfigWidget->SetDoubleWidgetValue("suspension_cfm", 0.13);
    jointConfigWidget->SetDoubleWidgetValue("suspension_erp", 0.12);
  }

  // verify widget values
  {
    // joint
    QVERIFY(jointConfigWidget->GetStringWidgetValue("name") ==
        "test_joint_updated2");
    QCOMPARE(jointConfigWidget->GetUIntWidgetValue("id"), 2222222u);
    QVERIFY(jointConfigWidget->GetStringWidgetValue("parent") ==
        "test_joint_parent_updated2");
    QCOMPARE(jointConfigWidget->GetUIntWidgetValue("parent_id"), 10u);
    QVERIFY(jointConfigWidget->GetStringWidgetValue("child") ==
        "test_joint_child_updated2");
    QCOMPARE(jointConfigWidget->GetUIntWidgetValue("child_id"), 20u);

    // type
    QCOMPARE(jointConfigWidget->GetEnumWidgetValue("type"),
        gazebo::msgs::Joint_Type_Name(
        gazebo::msgs::Joint_Type_BALL));

    // pose
    gazebo::math::Vector3 pos(-2.0, 1.0, 2.0);
    gazebo::math::Quaternion quat(0.0, 0.0, 0.0);
    QCOMPARE(jointConfigWidget->GetPoseWidgetValue("pose"),
        gazebo::math::Pose(pos, quat));

    // other joint physics properties
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("cfm"), 0.19);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("bounce"), 0.18);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("velocity"), 2.7);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("fudge_factor"), 0.26);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("limit_cfm"), 0.15);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("limit_erp"), 0.24);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("suspension_cfm"), 0.13);
    QCOMPARE(jointConfigWidget->GetDoubleWidgetValue("suspension_erp"), 0.12);
  }

  // verify updates in new msg
  {
    gazebo::msgs::Joint *retJointMsg =
        dynamic_cast<gazebo::msgs::Joint *>(jointConfigWidget->GetMsg());
    QVERIFY(retJointMsg != NULL);

    // joint
    QVERIFY(retJointMsg->name() == "test_joint_updated2");
    QCOMPARE(retJointMsg->id(), 2222222u);
    QVERIFY(retJointMsg->parent() == "test_joint_parent_updated2");
    QCOMPARE(retJointMsg->parent_id(), 10u);
    QVERIFY(retJointMsg->child() == "test_joint_child_updated2");
    QCOMPARE(retJointMsg->child_id(), 20u);

    // type
    QCOMPARE(retJointMsg->type(), gazebo::msgs::ConvertJointType("ball"));

    // pose
    const gazebo::msgs::Pose poseMsg = retJointMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), -2.0);
    QCOMPARE(posMsg.y(), 1.0);
    QCOMPARE(posMsg.z(), 2.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    gazebo::math::Quaternion quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.GetAsEuler().x, 0.0);
    QCOMPARE(quat.GetAsEuler().y, 0.0);
    QCOMPARE(quat.GetAsEuler().z, 0.0);

    // other joint physics properties
    QCOMPARE(retJointMsg->cfm(), 0.19);
    QCOMPARE(retJointMsg->bounce(), 0.18);
    QCOMPARE(retJointMsg->velocity(), 2.7);
    QCOMPARE(retJointMsg->fudge_factor(), 0.26);
    QCOMPARE(retJointMsg->limit_cfm(), 0.15);
    QCOMPARE(retJointMsg->limit_erp(), 0.24);
    QCOMPARE(retJointMsg->suspension_cfm(), 0.13);
    QCOMPARE(retJointMsg->suspension_erp(), 0.12);
  }
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::VisualMsgWidget()
{
  // create a visual message with test values
  // leave out plugin field for now

  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

  {
    // visual
    visualMsg.set_name("test_visual");
    visualMsg.set_id(12345u);
    visualMsg.set_parent_name("test_visual_parent");
    visualMsg.set_parent_id(54321u);
    visualMsg.set_cast_shadows(true);
    visualMsg.set_transparency(0.0);
    visualMsg.set_visible(true);
    visualMsg.set_delete_me(false);
    visualMsg.set_is_static(false);
    gazebo::msgs::Set(visualMsg.mutable_scale(),
        ignition::math::Vector3d(1.0, 1.0, 1.0));

    // pose
    ignition::math::Vector3d pos(2.0, 3.0, 4.0);
    ignition::math::Quaterniond quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // geometry
    gazebo::msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
    geometryMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    gazebo::msgs::CylinderGeom *cylinderGeomMsg =
        geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
    materialMsg->set_shader_type(
        gazebo::msgs::Material::Material::VERTEX);
    materialMsg->set_normal_map("test_normal_map");
    gazebo::msgs::Set(materialMsg->mutable_ambient(),
        gazebo::common::Color(0.0, 1.0, 0.0, 1.0));
    gazebo::msgs::Set(materialMsg->mutable_diffuse(),
        gazebo::common::Color(0.0, 1.0, 1.0, 0.4));
    gazebo::msgs::Set(materialMsg->mutable_specular(),
        gazebo::common::Color(1.0, 1.0, 1.0, 0.6));
    gazebo::msgs::Set(materialMsg->mutable_emissive(),
        gazebo::common::Color(0.0, 0.5, 0.2, 1.0));
    materialMsg->set_lighting(true);
    // material::script
    gazebo::msgs::Material::Script *scriptMsg = materialMsg->mutable_script();
    scriptMsg->add_uri("test_script_uri_0");
    scriptMsg->add_uri("test_script_uri_1");
    scriptMsg->set_name("test_script_name");
  }
  visualConfigWidget->Load(&visualMsg);

  // retrieve the message from the config widget and
  // verify that all values have not been changed.
  {
    gazebo::msgs::Visual *retVisualMsg =
        dynamic_cast<gazebo::msgs::Visual *>(visualConfigWidget->GetMsg());
    QVERIFY(retVisualMsg != NULL);

    // visual
    QVERIFY(retVisualMsg->name() == "test_visual");
    QCOMPARE(retVisualMsg->id(), 12345u);
    QVERIFY(retVisualMsg->parent_name() == "test_visual_parent");
    QCOMPARE(retVisualMsg->parent_id(), 54321u);
    QCOMPARE(retVisualMsg->cast_shadows(), true);
    QCOMPARE(retVisualMsg->transparency(), 0.0);
    QCOMPARE(retVisualMsg->visible(), true);
    QCOMPARE(retVisualMsg->delete_me(), false);
    QCOMPARE(retVisualMsg->is_static(), false);
    const gazebo::msgs::Vector3d scaleMsg = retVisualMsg->scale();
    QCOMPARE(scaleMsg.x(), 1.0);
    QCOMPARE(scaleMsg.y(), 1.0);
    QCOMPARE(scaleMsg.z(), 1.0);

    // pose
    const gazebo::msgs::Pose poseMsg = retVisualMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), 2.0);
    QCOMPARE(posMsg.y(), 3.0);
    QCOMPARE(posMsg.z(), 4.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    gazebo::math::Quaternion quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.GetAsEuler().x, 1.57);
    QCOMPARE(quat.GetAsEuler().y, 0.0);
    QCOMPARE(quat.GetAsEuler().z, 0.0);

    // geometry
    const gazebo::msgs::Geometry geometryMsg = retVisualMsg->geometry();
    QCOMPARE(geometryMsg.type(), gazebo::msgs::Geometry::CYLINDER);
    const gazebo::msgs::CylinderGeom cylinderGeomMsg = geometryMsg.cylinder();
    QCOMPARE(cylinderGeomMsg.radius(), 3.0);
    QCOMPARE(cylinderGeomMsg.length(), 0.2);

    // material
    const gazebo::msgs::Material materialMsg = retVisualMsg->material();
    QCOMPARE(materialMsg.shader_type(),
        gazebo::msgs::Material::Material::VERTEX);
    QVERIFY(materialMsg.normal_map() == "test_normal_map");
    const gazebo::msgs::Color ambientMsg = materialMsg.ambient();
    QCOMPARE(ambientMsg.r(), 0.0f);
    QCOMPARE(ambientMsg.g(), 1.0f);
    QCOMPARE(ambientMsg.b(), 0.0f);
    QCOMPARE(ambientMsg.a(), 1.0f);
    const gazebo::msgs::Color diffuseMsg = materialMsg.diffuse();
    QCOMPARE(diffuseMsg.r(), 0.0f);
    QCOMPARE(diffuseMsg.g(), 1.0f);
    QCOMPARE(diffuseMsg.b(), 1.0f);
    QCOMPARE(diffuseMsg.a(), 0.4f);
    const gazebo::msgs::Color specularMsg = materialMsg.specular();
    QCOMPARE(specularMsg.r(), 1.0f);
    QCOMPARE(specularMsg.g(), 1.0f);
    QCOMPARE(specularMsg.b(), 1.0f);
    QCOMPARE(specularMsg.a(), 0.6f);
    const gazebo::msgs::Color emissiveMsg = materialMsg.emissive();
    QCOMPARE(emissiveMsg.r(), 0.0f);
    QCOMPARE(emissiveMsg.g(), 0.5f);
    QCOMPARE(emissiveMsg.b(), 0.2f);
    QCOMPARE(emissiveMsg.a(), 1.0f);
    QCOMPARE(materialMsg.lighting(), true);
    // material::script
    const gazebo::msgs::Material::Script scriptMsg = materialMsg.script();
    QVERIFY(scriptMsg.uri(0) == "test_script_uri_0");
    QVERIFY(scriptMsg.uri(1) == "test_script_uri_1");
    QVERIFY(scriptMsg.name() == "test_script_name");
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  {
    // visual
    visualConfigWidget->SetStringWidgetValue("name", "test_visual_updated");
    visualConfigWidget->SetUIntWidgetValue("id", 11111u);
    visualConfigWidget->SetStringWidgetValue("parent_name",
        "test_visual_parent_updated");
    visualConfigWidget->SetUIntWidgetValue("parent_id", 55555u);
    visualConfigWidget->SetBoolWidgetValue("cast_shadows", false);
    visualConfigWidget->SetDoubleWidgetValue("transparency", 1.0);
    visualConfigWidget->SetBoolWidgetValue("visible", false);
    visualConfigWidget->SetBoolWidgetValue("delete_me", true);
    visualConfigWidget->SetBoolWidgetValue("is_static", true);
    visualConfigWidget->SetVector3WidgetValue("scale",
        gazebo::math::Vector3(2.0, 1.5, 0.5));

    // pose
    gazebo::math::Vector3 pos(-2.0, -3.0, -4.0);
    gazebo::math::Quaternion quat(0.0, 1.57, 0.0);
    visualConfigWidget->SetPoseWidgetValue("pose",
        gazebo::math::Pose(pos, quat));

    // geometry
    visualConfigWidget->SetGeometryWidgetValue("geometry", "box",
        gazebo::math::Vector3(5.0, 3.0, 4.0));

    // material
    visualConfigWidget->SetStringWidgetValue("material::normal_map",
        "test_normal_map_updated");
    visualConfigWidget->SetColorWidgetValue("material::ambient",
        gazebo::common::Color(0.2, 0.3, 0.4, 0.5));
    visualConfigWidget->SetColorWidgetValue("material::diffuse",
        gazebo::common::Color(0.1, 0.8, 0.6, 0.4));
    visualConfigWidget->SetColorWidgetValue("material::specular",
        gazebo::common::Color(0.5, 0.4, 0.3, 0.2));
    visualConfigWidget->SetColorWidgetValue("material::emissive",
        gazebo::common::Color(0.4, 0.6, 0.8, 0.1));
    visualConfigWidget->SetBoolWidgetValue("material::lighting", false);
    // material::script
    visualConfigWidget->SetStringWidgetValue("material::script::name",
        "test_script_name_updated");
  }

  // verify widget values
  {
    QVERIFY(visualConfigWidget->GetStringWidgetValue("name") ==
        "test_visual_updated");
    QCOMPARE(visualConfigWidget->GetUIntWidgetValue("id"), 11111u);
    QVERIFY(visualConfigWidget->GetStringWidgetValue("parent_name") ==
        "test_visual_parent_updated");
    QCOMPARE(visualConfigWidget->GetUIntWidgetValue("parent_id"), 55555u);
    QCOMPARE(visualConfigWidget->GetBoolWidgetValue("cast_shadows"), false);
    QCOMPARE(visualConfigWidget->GetDoubleWidgetValue("transparency"), 1.0);
    QCOMPARE(visualConfigWidget->GetBoolWidgetValue("visible"), false);
    QCOMPARE(visualConfigWidget->GetBoolWidgetValue("delete_me"), true);
    QCOMPARE(visualConfigWidget->GetBoolWidgetValue("is_static"), true);
    QCOMPARE(visualConfigWidget->GetVector3WidgetValue("scale"),
        gazebo::math::Vector3(2.0, 1.5, 0.5));

    // pose
    gazebo::math::Vector3 pos(-2.0, -3.0, -4.0);
    gazebo::math::Quaternion quat(0.0, 1.57, 0.0);
    QCOMPARE(visualConfigWidget->GetPoseWidgetValue("pose"),
        gazebo::math::Pose(pos, quat));

    // geometry
    ignition::math::Vector3d dimensions;
    std::string uri;
    QVERIFY(visualConfigWidget->GeometryWidgetValue("geometry", dimensions,
        uri) == "box");
    QCOMPARE(dimensions, ignition::math::Vector3d(5.0, 3.0, 4.0));

    // material
    QVERIFY(visualConfigWidget->GetStringWidgetValue("material::normal_map") ==
        "test_normal_map_updated");
    QCOMPARE(visualConfigWidget->GetColorWidgetValue("material::ambient"),
        gazebo::common::Color(0.2, 0.3, 0.4, 0.5));
    QCOMPARE(visualConfigWidget->GetColorWidgetValue("material::diffuse"),
        gazebo::common::Color(0.1, 0.8, 0.6, 0.4));
    QCOMPARE(visualConfigWidget->GetColorWidgetValue("material::specular"),
        gazebo::common::Color(0.5, 0.4, 0.3, 0.2));
    QCOMPARE(visualConfigWidget->GetColorWidgetValue("material::emissive"),
        gazebo::common::Color(0.4, 0.6, 0.8, 0.1));
    QCOMPARE(visualConfigWidget->GetBoolWidgetValue("material::lighting"),
        false);
    // material::script
    QVERIFY(visualConfigWidget->GetStringWidgetValue("material::script::name")
        == "test_script_name_updated");
  }

  // verify updates in new msg
  {
    gazebo::msgs::Visual *retVisualMsg =
        dynamic_cast<gazebo::msgs::Visual *>(visualConfigWidget->GetMsg());
    QVERIFY(retVisualMsg != NULL);

    // visual
    QVERIFY(retVisualMsg->name() == "test_visual_updated");
    QCOMPARE(retVisualMsg->id(), 11111u);
    QVERIFY(retVisualMsg->parent_name() == "test_visual_parent_updated");
    QCOMPARE(retVisualMsg->parent_id(), 55555u);
    QCOMPARE(retVisualMsg->cast_shadows(), false);
    QCOMPARE(retVisualMsg->transparency(), 1.0);
    QCOMPARE(retVisualMsg->visible(), false);
    QCOMPARE(retVisualMsg->delete_me(), true);
    QCOMPARE(retVisualMsg->is_static(), true);
    const gazebo::msgs::Vector3d scaleMsg = retVisualMsg->scale();
    QCOMPARE(scaleMsg.x(), 2.0);
    QCOMPARE(scaleMsg.y(), 1.5);
    QCOMPARE(scaleMsg.z(), 0.5);

    // pose
    const gazebo::msgs::Pose poseMsg = retVisualMsg->pose();
    const gazebo::msgs::Vector3d posMsg = poseMsg.position();
    QCOMPARE(posMsg.x(), -2.0);
    QCOMPARE(posMsg.y(), -3.0);
    QCOMPARE(posMsg.z(), -4.0);
    const gazebo::msgs::Quaternion quatMsg = poseMsg.orientation();
    gazebo::math::Quaternion quat(quatMsg.w(), quatMsg.x(), quatMsg.y(),
        quatMsg.z());
    QCOMPARE(quat.GetAsEuler().x, 0.0);
    QCOMPARE(quat.GetAsEuler().y, 1.57);
    QCOMPARE(quat.GetAsEuler().z, 0.0);

    // geometry
    const gazebo::msgs::Geometry geometryMsg = retVisualMsg->geometry();
    QCOMPARE(geometryMsg.type(), gazebo::msgs::Geometry::BOX);
    const gazebo::msgs::BoxGeom boxGeomMsg = geometryMsg.box();
    const gazebo::msgs::Vector3d boxGeomSizeMsg = boxGeomMsg.size();
    QCOMPARE(boxGeomSizeMsg.x(), 5.0);
    QCOMPARE(boxGeomSizeMsg.y(), 3.0);
    QCOMPARE(boxGeomSizeMsg.z(), 4.0);

    // material
    const gazebo::msgs::Material materialMsg = retVisualMsg->material();
    QCOMPARE(materialMsg.shader_type(),
        gazebo::msgs::Material::Material::VERTEX);
    QVERIFY(materialMsg.normal_map() == "test_normal_map_updated");
    const gazebo::msgs::Color ambientMsg = materialMsg.ambient();
    QCOMPARE(ambientMsg.r(), 0.2f);
    QCOMPARE(ambientMsg.g(), 0.3f);
    QCOMPARE(ambientMsg.b(), 0.4f);
    QCOMPARE(ambientMsg.a(), 0.5f);
    const gazebo::msgs::Color diffuseMsg = materialMsg.diffuse();
    QCOMPARE(diffuseMsg.r(), 0.1f);
    QCOMPARE(diffuseMsg.g(), 0.8f);
    QCOMPARE(diffuseMsg.b(), 0.6f);
    QCOMPARE(diffuseMsg.a(), 0.4f);
    const gazebo::msgs::Color specularMsg = materialMsg.specular();
    QCOMPARE(specularMsg.r(), 0.5f);
    QCOMPARE(specularMsg.g(), 0.4f);
    QCOMPARE(specularMsg.b(), 0.3f);
    QCOMPARE(specularMsg.a(), 0.2f);
    const gazebo::msgs::Color emissiveMsg = materialMsg.emissive();
    QCOMPARE(emissiveMsg.r(), 0.4f);
    QCOMPARE(emissiveMsg.g(), 0.6f);
    QCOMPARE(emissiveMsg.b(), 0.8f);
    QCOMPARE(emissiveMsg.a(), 0.1f);
    QCOMPARE(materialMsg.lighting(), false);
    // material::script
    const gazebo::msgs::Material::Script scriptMsg = materialMsg.script();
    QVERIFY(scriptMsg.uri(0) == "test_script_uri_0");
    QVERIFY(scriptMsg.uri(1) == "test_script_uri_1");
    QVERIFY(scriptMsg.name() == "test_script_name_updated");
  }

  delete visualConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::PluginMsgWidget()
{
  // create a plugin message with test values

  gazebo::gui::ConfigWidget *pluginConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Plugin pluginMsg;

  {
    // plugin
    pluginMsg.set_name("test_plugin");
    pluginMsg.set_filename("test_plugin_filename");
    pluginMsg.set_innerxml("<param>1</param>\n");
  }
  pluginConfigWidget->Load(&pluginMsg);

  // retrieve the message from the config widget and
  // verify that all values have not been changed.
  {
    gazebo::msgs::Plugin *retPluginMsg =
        dynamic_cast<gazebo::msgs::Plugin *>(pluginConfigWidget->GetMsg());
    QVERIFY(retPluginMsg != NULL);

    // plugin
    QVERIFY(retPluginMsg->name() == "test_plugin");
    QVERIFY(retPluginMsg->filename() == "test_plugin_filename");
    QVERIFY(retPluginMsg->innerxml() == "<param>1</param>\n");
  }

  // update fields in the config widget and
  // verify that the new message contains the updated values.
  {
    // plugin
    pluginConfigWidget->SetStringWidgetValue("name", "test_plugin_updated");
    pluginConfigWidget->SetStringWidgetValue("filename",
        "test_plugin_filename_updated");
    pluginConfigWidget->SetStringWidgetValue("innerxml",
        "<param2>new_param</param2>\n");
  }

  // verify widget values
  {
    QVERIFY(pluginConfigWidget->GetStringWidgetValue("name") ==
        "test_plugin_updated");
    QVERIFY(pluginConfigWidget->GetStringWidgetValue("filename") ==
        "test_plugin_filename_updated");
    QVERIFY(pluginConfigWidget->GetStringWidgetValue("innerxml") ==
        "<param2>new_param</param2>\n");
  }

  // verify updates in new msg
  {
    gazebo::msgs::Plugin *retPluginMsg =
        dynamic_cast<gazebo::msgs::Plugin *>(pluginConfigWidget->GetMsg());
    QVERIFY(retPluginMsg != NULL);

    // plugin
    QVERIFY(retPluginMsg->name() == "test_plugin_updated");
    QVERIFY(retPluginMsg->filename() == "test_plugin_filename_updated");
    QVERIFY(retPluginMsg->innerxml() == "<param2>new_param</param2>\n");
  }

  delete pluginConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ConfigWidgetVisible()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

  {
    // visual
    visualMsg.set_id(12345u);

    // pose
    ignition::math::Vector3d pos(2.0, 3.0, 4.0);
    ignition::math::Quaterniond quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // geometry
    gazebo::msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
    geometryMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    gazebo::msgs::CylinderGeom *cylinderGeomMsg =
        geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
    gazebo::msgs::Set(materialMsg->mutable_ambient(),
        gazebo::common::Color(0.0, 1.0, 0.0, 1.0));
    gazebo::msgs::Set(materialMsg->mutable_diffuse(),
        gazebo::common::Color(0.0, 1.0, 1.0, 0.4));

    // material::script
    gazebo::msgs::Material::Script *scriptMsg = materialMsg->mutable_script();
    scriptMsg->set_name("test_script_name");
  }
  visualConfigWidget->Load(&visualMsg);
  visualConfigWidget->show();

  // set different types of widgets to be not visibile
  {
    // primitive widget
    visualConfigWidget->SetWidgetVisible("id", false);
    // custom pose message widget
    visualConfigWidget->SetWidgetVisible("pose", false);
    // custom geometry message widget
    visualConfigWidget->SetWidgetVisible("geometry", false);
    // widget inside a group widget
    visualConfigWidget->SetWidgetVisible("material::diffuse", false);
    // widget two levels deep
    visualConfigWidget->SetWidgetVisible("material::script::name", false);
    // group widget
    visualConfigWidget->SetWidgetVisible("material", false);

    QCOMPARE(visualConfigWidget->GetWidgetVisible("id"), false);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("pose"), false);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("geometry"), false);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("material::diffuse"), false);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("material::script::name"),
        false);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("material"), false);
  }

  // set visible back to true
  {
    visualConfigWidget->SetWidgetVisible("id", true);
    visualConfigWidget->SetWidgetVisible("pose", true);
    visualConfigWidget->SetWidgetVisible("geometry", true);
    visualConfigWidget->SetWidgetVisible("material::diffuse", true);
    visualConfigWidget->SetWidgetVisible("material::script::name", true);
    visualConfigWidget->SetWidgetVisible("material", true);

    QCOMPARE(visualConfigWidget->GetWidgetVisible("id"), true);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("pose"), true);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("geometry"), true);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("material::diffuse"), true);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("material::script::name"),
        true);
    QCOMPARE(visualConfigWidget->GetWidgetVisible("material"), true);
  }

  delete visualConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::ConfigWidgetReadOnly()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

  {
    // visual
    visualMsg.set_id(12345u);

    // pose
    ignition::math::Vector3d pos(2.0, 3.0, 4.0);
    ignition::math::Quaterniond quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(),
        ignition::math::Pose3d(pos, quat));

    // geometry
    gazebo::msgs::Geometry *geometryMsg = visualMsg.mutable_geometry();
    geometryMsg->set_type(gazebo::msgs::Geometry::CYLINDER);
    gazebo::msgs::CylinderGeom *cylinderGeomMsg =
        geometryMsg->mutable_cylinder();
    cylinderGeomMsg->set_radius(3.0);
    cylinderGeomMsg->set_length(0.2);

    // material
    gazebo::msgs::Material *materialMsg = visualMsg.mutable_material();
    gazebo::msgs::Set(materialMsg->mutable_ambient(),
        gazebo::common::Color(0.0, 1.0, 0.0, 1.0));
    gazebo::msgs::Set(materialMsg->mutable_diffuse(),
        gazebo::common::Color(0.0, 1.0, 1.0, 0.4));

    // material::script
    gazebo::msgs::Material::Script *scriptMsg = materialMsg->mutable_script();
    scriptMsg->set_name("test_script_name");
  }
  visualConfigWidget->Load(&visualMsg);

  // set different types of widgets to be read-only
  {
    // primitive widget
    visualConfigWidget->SetWidgetReadOnly("id", true);
    // custom pose message widget
    visualConfigWidget->SetWidgetReadOnly("pose", true);
    // custom geometry message widget
    visualConfigWidget->SetWidgetReadOnly("geometry", true);
    // widget inside a group widget
    visualConfigWidget->SetWidgetReadOnly("material::diffuse", true);
    // widget two levels deep
    visualConfigWidget->SetWidgetReadOnly("material::script::name", true);
    // group widget
    visualConfigWidget->SetWidgetReadOnly("material", true);

    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("id"), true);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("pose"), true);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("geometry"), true);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("material::diffuse"), true);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("material::script::name"),
        true);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("material"), true);
  }

  // set read-only back to false
  {
    visualConfigWidget->SetWidgetReadOnly("id", false);
    visualConfigWidget->SetWidgetReadOnly("pose", false);
    visualConfigWidget->SetWidgetReadOnly("geometry", false);
    visualConfigWidget->SetWidgetReadOnly("material::diffuse", false);
    visualConfigWidget->SetWidgetReadOnly("material::script::name", false);
    visualConfigWidget->SetWidgetReadOnly("material", false);

    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("id"), false);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("pose"), false);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("geometry"), false);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("material::diffuse"), false);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("material::script::name"),
        false);
    QCOMPARE(visualConfigWidget->GetWidgetReadOnly("material"), false);
  }

  delete visualConfigWidget;
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::CreatedExternally()
{
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget;

  // Create predefined child widgets
  gazebo::gui::ConfigChildWidget *uintWidget =
      configWidget->CreateUIntWidget("uint", 0);
  gazebo::gui::ConfigChildWidget *intWidget =
      configWidget->CreateIntWidget("int", 0);
  gazebo::gui::ConfigChildWidget *doubleWidget =
      configWidget->CreateDoubleWidget("double", 1);
  gazebo::gui::ConfigChildWidget *stringWidget =
      configWidget->CreateStringWidget("string", 1);
  gazebo::gui::ConfigChildWidget *boolWidget =
      configWidget->CreateBoolWidget("bool", 2);
  gazebo::gui::ConfigChildWidget *vector3dWidget =
      configWidget->CreateVector3dWidget("vector3d", 2);
  gazebo::gui::ConfigChildWidget *colorWidget =
      configWidget->CreateColorWidget("color", 3);
  gazebo::gui::ConfigChildWidget *poseWidget =
      configWidget->CreatePoseWidget("pose", 3);

  std::vector<std::string> enumValues;
  enumValues.push_back("value1");
  enumValues.push_back("value2");
  enumValues.push_back("value3");
  gazebo::gui::ConfigChildWidget *enumWidget =
      configWidget->CreateEnumWidget("enum", enumValues, 4);

  QVERIFY(uintWidget != NULL);
  QVERIFY(intWidget != NULL);
  QVERIFY(doubleWidget != NULL);
  QVERIFY(stringWidget != NULL);
  QVERIFY(boolWidget != NULL);
  QVERIFY(vector3dWidget != NULL);
  QVERIFY(colorWidget != NULL);
  QVERIFY(poseWidget != NULL);
  QVERIFY(enumWidget != NULL);

  // Create a custom child widget
  QLabel *customLabel = new QLabel("custom label");
  QLineEdit *customLineEdit = new QLineEdit();
  QHBoxLayout *customLayout = new QHBoxLayout();
  customLayout->addWidget(customLabel);
  customLayout->addWidget(customLineEdit);

  gazebo::gui::ConfigChildWidget *customWidget =
      new gazebo::gui::ConfigChildWidget();
  customWidget->setLayout(customLayout);
  customWidget->widgets.push_back(customLineEdit);

  // Add child widgets to config widget
  QCOMPARE(configWidget->ConfigChildWidgetCount(), 0u);

  QVERIFY(configWidget->AddConfigChildWidget("uint", uintWidget));
  QVERIFY(configWidget->AddConfigChildWidget("int", intWidget));
  QVERIFY(configWidget->AddConfigChildWidget("double", doubleWidget));
  QVERIFY(configWidget->AddConfigChildWidget("string", stringWidget));
  QVERIFY(configWidget->AddConfigChildWidget("bool", boolWidget));
  QVERIFY(configWidget->AddConfigChildWidget("vector3d", vector3dWidget));
  QVERIFY(configWidget->AddConfigChildWidget("color", colorWidget));
  QVERIFY(configWidget->AddConfigChildWidget("pose", poseWidget));
  QVERIFY(configWidget->AddConfigChildWidget("enum", enumWidget));
  QVERIFY(configWidget->AddConfigChildWidget("custom", customWidget));

  QCOMPARE(configWidget->ConfigChildWidgetCount(), 10u);

  // Fail to add invalid children
  QCOMPARE(configWidget->AddConfigChildWidget("", uintWidget), false);
  QCOMPARE(configWidget->AddConfigChildWidget("validName", NULL), false);
  QCOMPARE(configWidget->AddConfigChildWidget("uint", intWidget), false);

  QCOMPARE(configWidget->ConfigChildWidgetCount(), 10u);

  // Check that checking visibility works
  QCOMPARE(configWidget->GetWidgetVisible("uint"), uintWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("int"), intWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("double"), doubleWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("string"), stringWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("bool"), boolWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("vector3d"),
      vector3dWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("color"), colorWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("pose"), poseWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("enum"), enumWidget->isVisible());
  QCOMPARE(configWidget->GetWidgetVisible("custom"), customWidget->isVisible());

  // Set widgets values
  unsigned int uintValue = 123;
  int intValue = -456;
  double doubleValue = 123.456;
  std::string stringValue("123");
  bool boolValue = true;
  ignition::math::Vector3d vector3dValue(1, 2, 3);
  gazebo::common::Color colorValue(0.1, 0.2, 0.3, 0.4);
  ignition::math::Pose3d poseValue(1, 2, 3, 0.1, 0.2, 0.3);
  std::string enumValue("value2");
  std::string customValue("123456789");

  QVERIFY(configWidget->SetUIntWidgetValue("uint", uintValue));
  QVERIFY(configWidget->SetIntWidgetValue("int", intValue));
  QVERIFY(configWidget->SetDoubleWidgetValue("double", doubleValue));
  QVERIFY(configWidget->SetStringWidgetValue("string", stringValue));
  QVERIFY(configWidget->SetBoolWidgetValue("bool", boolValue));
  QVERIFY(configWidget->SetVector3WidgetValue("vector3d",
      gazebo::math::Vector3(vector3dValue)));
  QVERIFY(configWidget->SetColorWidgetValue("color", colorValue));
  QVERIFY(configWidget->SetPoseWidgetValue("pose", poseValue));
  QVERIFY(configWidget->SetEnumWidgetValue("enum", enumValue));
  QVERIFY(configWidget->SetStringWidgetValue("custom", customValue));

  // Get widgets values
  QCOMPARE(configWidget->GetUIntWidgetValue("uint"), uintValue);
  QCOMPARE(configWidget->GetIntWidgetValue("int"), intValue);
  QCOMPARE(configWidget->GetDoubleWidgetValue("double"), doubleValue);
  QCOMPARE(configWidget->GetStringWidgetValue("string"), stringValue);
  QCOMPARE(configWidget->GetBoolWidgetValue("bool"), boolValue);
  QCOMPARE(configWidget->GetVector3WidgetValue("vector3d"),
      gazebo::math::Vector3(vector3dValue));
  QCOMPARE(configWidget->GetColorWidgetValue("color"), colorValue);
  QCOMPARE(configWidget->GetPoseWidgetValue("pose"),
      gazebo::math::Pose(poseValue));
  QCOMPARE(configWidget->GetEnumWidgetValue("enum"), enumValue);
  QCOMPARE(configWidget->GetStringWidgetValue("custom"), customValue);

  // Group some widgets
  QVBoxLayout *groupLayout = new QVBoxLayout();
  groupLayout->addWidget(uintWidget);
  groupLayout->addWidget(intWidget);
  groupLayout->addWidget(doubleWidget);

  QGroupBox *groupBox = new QGroupBox();
  groupBox->setLayout(groupLayout);

  QVBoxLayout *groupChildWidgetLayout = new QVBoxLayout();
  groupChildWidgetLayout->addWidget(groupBox);

  gazebo::gui::ConfigChildWidget *groupChildWidget =
      new gazebo::gui::ConfigChildWidget();
  groupChildWidget->setLayout(groupChildWidgetLayout);
  groupChildWidget->widgets.push_back(groupBox);

  gazebo::gui::GroupWidget *groupWidget =
      configWidget->CreateGroupWidget("groupWidget", groupChildWidget, 0);
  QVERIFY(groupWidget != NULL);
  QVERIFY(groupWidget->childWidget != NULL);
}

/////////////////////////////////////////////////
void ConfigWidget_TEST::EnumConfigWidget()
{
  // Create a parent widget
  gazebo::gui::ConfigWidget *configWidget = new gazebo::gui::ConfigWidget();
  QVERIFY(configWidget != NULL);

  // Create an enum child widget
  std::vector<std::string> enumValues;
  enumValues.push_back("value1");
  enumValues.push_back("value2");
  enumValues.push_back("value3");
  gazebo::gui::ConfigChildWidget *enumWidget =
      configWidget->CreateEnumWidget("Enum Label", enumValues);

  QVERIFY(enumWidget != NULL);

  // Add it to parent
  QVERIFY(configWidget->AddConfigChildWidget("enumWidgetName", enumWidget));

  // Check that all items can be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value1"));
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value2"));
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value3"));

  // Check that an inexistent item cannot be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value4")
      == false);

  // Check the number of items
  QComboBox *comboBox = enumWidget->findChild<QComboBox *>();
  QVERIFY(comboBox != NULL);
  QCOMPARE(comboBox->count(), 3);

  // Add an item and check count
  QVERIFY(configWidget->AddItemEnumWidget("enumWidgetName", "value4"));
  QCOMPARE(comboBox->count(), 4);

  // Check that the new item can be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value4"));

  // Remove an item and check count
  QVERIFY(configWidget->RemoveItemEnumWidget("enumWidgetName", "value2"));
  QCOMPARE(comboBox->count(), 3);

  // Check that the removed item cannot be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value2")
      == false);

  // Clear all items and check count
  QVERIFY(configWidget->ClearEnumWidget("enumWidgetName"));
  QCOMPARE(comboBox->count(), 0);

  // Check that none of the previous items can be selected
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value1")
      == false);
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value2")
      == false);
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value3")
      == false);
  QVERIFY(configWidget->SetEnumWidgetValue("enumWidgetName", "value4")
      == false);
}

// Generate a main function for the test
QTEST_MAIN(ConfigWidget_TEST)
