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
        gazebo::math::Vector3(1.0, 1.0, 1.0));

    // pose
    gazebo::math::Vector3 pos(2.0, 3.0, 4.0);
    gazebo::math::Quaternion quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(), gazebo::math::Pose(pos, quat));

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
    gazebo::math::Vector3 dimensions;
    std::string uri;
    QVERIFY(visualConfigWidget->GetGeometryWidgetValue("geometry", dimensions,
        uri) == "box");
    QCOMPARE(dimensions, gazebo::math::Vector3(5.0, 3.0, 4.0));

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
void ConfigWidget_TEST::ConfigWidgetVisible()
{
  gazebo::gui::ConfigWidget *visualConfigWidget =
      new gazebo::gui::ConfigWidget;
  gazebo::msgs::Visual visualMsg;

{
    // visual
    visualMsg.set_id(12345u);

    // pose
    gazebo::math::Vector3 pos(2.0, 3.0, 4.0);
    gazebo::math::Quaternion quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(), gazebo::math::Pose(pos, quat));

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
    gazebo::math::Vector3 pos(2.0, 3.0, 4.0);
    gazebo::math::Quaternion quat(1.57, 0.0, 0.0);
    gazebo::msgs::Set(visualMsg.mutable_pose(), gazebo::math::Pose(pos, quat));

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

// Generate a main function for the test
QTEST_MAIN(ConfigWidget_TEST)
