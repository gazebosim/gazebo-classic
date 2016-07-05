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

#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gui/plot/Palette.hh"
#include "gazebo/gui/plot/Palette_TEST.hh"

/////////////////////////////////////////////////
void Palette_TEST::TopicsTab()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Count the number of topics being advertised
  int count = 0;
  auto msgTopics = gazebo::transport::getAdvertisedTopics();
  for (auto msgTopic : msgTopics)
  {
    for (auto topic : msgTopic.second)
      count++;
  }

  // Create a new plot window widget
  auto palette = new gazebo::gui::Palette(nullptr);
  QVERIFY(palette != nullptr);

  // Get the topics model
  auto topicsModel =
      palette->findChild<QStandardItemModel *>("plotTopicsModel");
  QVERIFY(topicsModel != nullptr);

  // Check the model has as many rows as there are topics
  QCOMPARE(topicsModel->rowCount(), count);

  delete palette;
}

/////////////////////////////////////////////////
void Palette_TEST::ModelsTab()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world");

  // Get the number of models in the world
  auto world = gazebo::physics::get_world("default");
  QVERIFY(world != nullptr);

  auto count = world->GetModelCount();

  // Create a new plot window widget
  auto palette = new gazebo::gui::Palette(nullptr);
  QVERIFY(palette != nullptr);

  // Get the models model
  auto modelsModel =
      palette->findChild<QStandardItemModel *>("plotModelsModel");
  QVERIFY(modelsModel != nullptr);

  // Check the model has as many rows as there are top level models,
  // plus the title
  QCOMPARE(modelsModel->rowCount(), static_cast<int>(count + 1));

  // Delete a model
  world->RemoveModel("sphere");

  this->ProcessEventsAndDraw();

  // Check it was deleted
  QCOMPARE(count - 1, world->GetModelCount());
  count--;

  // Get the new models model
  modelsModel =
      palette->findChild<QStandardItemModel *>("plotModelsModel");
  QVERIFY(modelsModel != nullptr);

  // Check the model has as many rows as there are top level models,
  // plus the title
  QCOMPARE(modelsModel->rowCount(), static_cast<int>(count + 1));

  delete palette;
}

// Generate a main function for the test
QTEST_MAIN(Palette_TEST)
