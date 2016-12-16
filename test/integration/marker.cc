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
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "marker.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void Marker_TEST::AddRemove()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty_bright.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);

  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);

  cam->SetCaptureData(true);

  auto camWidth = (int) cam->ImageWidth();

  gzmsg << "Camera width: " << camWidth << std::endl;

  // Create our node for communication
  ignition::transport::Node node;

  std::string topicName = "/marker";

  std::vector<std::string> serviceList;
  node.ServiceList(serviceList);

  QVERIFY(std::find(serviceList.begin(), serviceList.end(), topicName)
          != serviceList.end());

  // Publish to a Gazebo topic
  node.Advertise<ignition::msgs::Marker>(topicName);

  // Create the marker message
  ignition::msgs::Marker markerMsg;
  markerMsg.set_ns("default");
  markerMsg.set_id(0);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::SPHERE);

  gzmsg << "Add sphere" << std::endl;
  QVERIFY(node.Request(topicName, markerMsg));

  this->ProcessEventsAndDraw(mainWindow);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != nullptr);

  gazebo::rendering::VisualPtr vis =
    scene->GetVisual("__GZ_MARKER_VISUAL_default_0");
  QVERIFY(vis != nullptr);

  // Remove the shape
  gzmsg << "Remove sphere" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);

  // Add a box
  gzmsg << "Add box" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::BOX);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") != nullptr);

  // Add a cylinder
  gzmsg << "Add cylinder" << std::endl;
  markerMsg.set_ns("default");
  markerMsg.set_id(1);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(2, 0, .5, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") != nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") != nullptr);

  // Delete everything
  gzmsg << "Delete everything" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);

  // Draw a vertical line using LINE_LIST
  gzmsg << "Draw line list" << std::endl;
  markerMsg.set_id(2);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 0, -10));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 0, 10));
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") != nullptr);

  // Draw another vertical line using LINE_STRIP
  gzmsg << "Draw line strip" << std::endl;
  markerMsg.set_id(3);
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, -10));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 10));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 0, 10));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 0, -10));
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") != nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") != nullptr);

  // Delete everything
  gzmsg << "Delete everything" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);

  // Draw a bunch of points
  gzmsg << "Draw points" << std::endl;
  markerMsg.set_id(4);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::POINTS);
  markerMsg.clear_point();
  for (int i = 0; i < 100; ++i)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(
          ignition::math::Rand::DblUniform(-1, 1),
          ignition::math::Rand::DblUniform(-1, 1),
          ignition::math::Rand::DblUniform(-1, 1)));
  }
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") != nullptr);

  // Delete everything
  gzmsg << "Delete everything" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);

  // Draw text
  gzmsg << "Draw text" << std::endl;
  markerMsg.set_id(5);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TEXT);
  markerMsg.set_text("HELLO");
  ignition::msgs::Set(markerMsg.mutable_pose(),
                      ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") != nullptr);

  // Remove the text
  gzmsg << "Remove text" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") == nullptr);

  // Draw a triangle fan
  gzmsg << "Draw triangle fan" << std::endl;
  markerMsg.set_id(5);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.05));
  double radius = 2;
  for (double t = 0; t <= M_PI; t+= 0.01)
  {
    ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(radius * cos(t), radius * sin(t), 0.05));
  }
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") != nullptr);

  // Remove the triangle fan
  gzmsg << "Remove triangle fan" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") == nullptr);

  // Draw a triangle list
  gzmsg << "Draw triangle list" << std::endl;
  markerMsg.set_id(6);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.mutable_pose(),
                    ignition::math::Pose3d(0, -1.5, 0, 0, 0, 0));
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.5));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.5));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.5));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.5));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 1, 0.5));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(2, 2, 0.5));

  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_6") != nullptr);

  // Remove the triangle list
  gzmsg << "Remove triangle list" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_6") == nullptr);

  // Draw a triangle strip
  gzmsg << "Draw triangle strip" << std::endl;
  markerMsg.set_id(7);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
  markerMsg.clear_point();
  ignition::msgs::Set(markerMsg.add_point(),
        ignition::math::Vector3d(0, 0, 0.3));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 0, 0.3));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 1, 0.3));

  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 1, 0.3));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(0, 2, 0.3));
  ignition::msgs::Set(markerMsg.add_point(),
      ignition::math::Vector3d(1, 2, 0.3));
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_0") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_1") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_2") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_3") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_4") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_5") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_6") == nullptr);
  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_7") != nullptr);

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(Marker_TEST)
