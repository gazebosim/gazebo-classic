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

std::mutex mutex;
gazebo::common::Time simTime;

/////////////////////////////////////////////////
void SimTimeCallback(ConstWorldStatisticsPtr  &_msg)
{
  std::lock_guard<std::mutex> lock(mutex);
  simTime = gazebo::msgs::Convert(_msg->sim_time());
}

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

  // Delete everything
  gzmsg << "Delete everything" << std::endl;
  markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
  QVERIFY(node.Request(topicName, markerMsg));
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_7") == nullptr);

  // test lifetime
  // first get current sim time
  gazebo::transport::NodePtr gznode(new gazebo::transport::Node());
  gznode->Init();
  auto sub = gznode->Subscribe("~/world_stats", SimTimeCallback);
  for (unsigned int i = 0; i < 5; ++i)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (simTime == gazebo::common::Time::Zero)
      gazebo::common::Time::MSleep(300);
    else
      break;
  }
  sub.reset();
  gznode.reset();
  QVERIFY(simTime != gazebo::common::Time::Zero);

  // spawn sphere with lifetime
  markerMsg.set_ns("default");
  markerMsg.set_id(8);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::SPHERE);
  markerMsg.clear_point();
  // 5 sec life time
  int sec = 5;
  gazebo::common::Time lifeTime = simTime +
      gazebo::common::Time(sec, 0);
  ignition::msgs::Time *sphereLifeTime = markerMsg.mutable_lifetime();
  sphereLifeTime->set_sec(lifeTime.sec);
  sphereLifeTime->set_nsec(lifeTime.nsec);

  gzmsg << "Add sphere with life time" << std::endl;
  QVERIFY(node.Request(topicName, markerMsg));

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(scene->GetVisual("__GZ_MARKER_VISUAL_default_8") != nullptr);

  // sphere should be gone after life time is ended.
  for (int i = 0; i < 30; ++i)
  {
    vis = scene->GetVisual("__GZ_MARKER_VISUAL_default_8");
    if (!vis)
      break;
    this->ProcessEventsAndDraw(mainWindow, 10, 30);
  }
  QVERIFY(vis == nullptr);


  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void Marker_TEST::CornerCases()
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

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene();
  QVERIFY(scene != nullptr);

  // Create our node for communication
  ignition::transport::Node node;

  std::string markerTopic = "/marker";
  std::string listTopic = "/marker/list";

  // Advertise to topics
  node.Advertise<ignition::msgs::Marker>(markerTopic);
  node.Advertise<ignition::msgs::Marker_V>(listTopic);

  // Create marker without namespace or id
  {
    auto visCount = scene->VisualCount();

    ignition::msgs::Marker markerMsg;
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::SPHERE);

    QVERIFY(node.Request(markerTopic, markerMsg));

    this->ProcessEventsAndDraw(mainWindow);

    // Visual was added
    QCOMPARE(scene->VisualCount(), visCount + 1);
  }

  // Check topic list
  {
    ignition::msgs::Marker_V rep;
    bool result;

    QVERIFY(node.Request(listTopic, 5000u, rep, result));

    this->ProcessEventsAndDraw(mainWindow);

    QCOMPARE(rep.marker().size(), 1);

    QVERIFY(rep.marker(0).has_ns());
    QVERIFY(rep.marker(0).ns() == "");

    QVERIFY(rep.marker(0).has_id());
    QVERIFY(rep.marker(0).id() > 0);

    QVERIFY(rep.marker(0).type() == ignition::msgs::Marker::SPHERE);
  }

  // Attempt to clear inexistent namespace
  {
    auto visCount = scene->VisualCount();

    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns("bad_namespace");
    markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);

    QVERIFY(node.Request(markerTopic, markerMsg));

    this->ProcessEventsAndDraw(mainWindow);

    // No visuals were removed
    QCOMPARE(scene->VisualCount(), visCount);
  }

  // Clear all namespaces, by specifying no namespace
  {
    auto visCount = scene->VisualCount();

    ignition::msgs::Marker markerMsg;
    markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);

    QVERIFY(node.Request(markerTopic, markerMsg));

    this->ProcessEventsAndDraw(mainWindow);

    // Marker was removed
    QCOMPARE(scene->VisualCount(), visCount - 1);
  }

  // Check topic list
  {
    ignition::msgs::Marker_V rep;
    bool result;

    QVERIFY(node.Request(listTopic, 5000u, rep, result));

    this->ProcessEventsAndDraw(mainWindow);

    QCOMPARE(rep.marker().size(), 0);
  }

  // Create marker with namespace but no id
  {
    auto visCount = scene->VisualCount();

    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns("the_namespace");
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::SPHERE);

    QVERIFY(node.Request(markerTopic, markerMsg));

    this->ProcessEventsAndDraw(mainWindow);

    // Visual was added
    QCOMPARE(scene->VisualCount(), visCount + 1);
  }

  // Check topic list
  {
    ignition::msgs::Marker_V rep;
    bool result;

    QVERIFY(node.Request(listTopic, 5000u, rep, result));

    this->ProcessEventsAndDraw(mainWindow);

    QCOMPARE(rep.marker().size(), 1);

    QVERIFY(rep.marker(0).has_ns());
    QVERIFY(rep.marker(0).ns() == "the_namespace");

    QVERIFY(rep.marker(0).has_id());
    QVERIFY(rep.marker(0).id() > 0);

    QVERIFY(rep.marker(0).type() == ignition::msgs::Marker::SPHERE);
  }

  // Try to delete inexistent marker
  {
    auto visCount = scene->VisualCount();

    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns("the_namespace");
    markerMsg.set_id(0);
    markerMsg.set_action(ignition::msgs::Marker::DELETE_MARKER);

    QVERIFY(node.Request(markerTopic, markerMsg));

    this->ProcessEventsAndDraw(mainWindow);

    // No markers were removed
    QCOMPARE(scene->VisualCount(), visCount);
  }

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(Marker_TEST)
