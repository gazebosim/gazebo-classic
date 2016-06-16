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

#include "gazebo/gui/plot/VariablePill.hh"
#include "gazebo/gui/plot/VariablePillContainer.hh"
#include "gazebo/gui/plot/VariablePillContainer_TEST.hh"

/////////////////////////////////////////////////
void VariablePillContainer_TEST::AddRemoveVariable()
{
  // create container
  gazebo::gui::VariablePillContainer *container01 =
      new gazebo::gui::VariablePillContainer(nullptr);
  QVERIFY(container01 != nullptr);
  QCOMPARE(container01->VariablePillCount(), 0u);

  // create variable pills
  gazebo::gui::VariablePill *var01 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var01 != nullptr);
  QCOMPARE(var01->VariablePillCount(), 0u);

  gazebo::gui::VariablePill *var02 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var02 != nullptr);
  QCOMPARE(var02->VariablePillCount(), 0u);

  gazebo::gui::VariablePill *var03 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var03 != nullptr);
  QCOMPARE(var03->VariablePillCount(), 0u);

  gazebo::gui::VariablePill *var04 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var04 != nullptr);
  QCOMPARE(var04->VariablePillCount(), 0u);

  gazebo::gui::VariablePill *var05 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var05 != nullptr);
  QCOMPARE(var05->VariablePillCount(), 0u);

  // add variable to container
  container01->AddVariablePill(var01);
  QCOMPARE(container01->VariablePillCount(), 1u);
  QCOMPARE(var01->Container(), container01);

  // add another variable to container
  container01->AddVariablePill(var02);
  QCOMPARE(container01->VariablePillCount(), 2u);
  QCOMPARE(var02->Container(), container01);

  // add variable to another variable - verify that containers can hold
  // multi-variables and report correct variable count
  var02->AddVariablePill(var03);
  QCOMPARE(container01->VariablePillCount(), 3u);
  QCOMPARE(var03->Container(), container01);
  QCOMPARE(var03->Parent(), var02);

  // make another multi-variable
  var01->AddVariablePill(var04);
  QCOMPARE(container01->VariablePillCount(), 4u);
  QCOMPARE(var04->Container(), container01);

  var04->AddVariablePill(var05);
  QCOMPARE(container01->VariablePillCount(), 5u);
  QCOMPARE(var05->Container(), container01);

  // remove variable
  container01->RemoveVariablePill(var01);
  QCOMPARE(container01->VariablePillCount(), 4u);

  // remove already removed variable
  container01->RemoveVariablePill(var01);
  QCOMPARE(container01->VariablePillCount(), 4u);

  // remove child of a multi-variable - check that the parent is not affected
  container01->RemoveVariablePill(var03);
  QCOMPARE(container01->VariablePillCount(), 3u);
  QCOMPARE(var02->Container(), container01);

  // remove multi-variable - check that it doesn't remove the child variable
  container01->RemoveVariablePill(var04);
  QCOMPARE(container01->VariablePillCount(), 2u);
  QVERIFY(var05->Parent() == nullptr);
  QCOMPARE(var05->Container(), container01);

  // remove remaining variables
  container01->RemoveVariablePill(var02);
  QCOMPARE(container01->VariablePillCount(), 1u);

  container01->RemoveVariablePill(var05);
  QCOMPARE(container01->VariablePillCount(), 0u);

  delete var01;
  delete var02;
  delete var03;
  delete var04;
  delete var05;
  delete container01;
}

/////////////////////////////////////////////////
void VariablePillContainer_TEST::MaxSize()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // create new container
  gazebo::gui::VariablePillContainer *container =
      new gazebo::gui::VariablePillContainer(nullptr);
  QVERIFY(container != nullptr);

  // set text
  std::string containerName = "container_test";
  container->SetText(containerName);
  QCOMPARE(container->Text(), containerName);

  // set max size
  QCOMPARE(container->MaxSize(), -1);
  int maxSize = 1;
  container->SetMaxSize(maxSize);
  QCOMPARE(container->MaxSize(), maxSize);

  // create variable pills
  gazebo::gui::VariablePill *var01 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var01 != nullptr);
  QCOMPARE(var01->VariablePillCount(), 0u);

  gazebo::gui::VariablePill *var02 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var02 != nullptr);
  QCOMPARE(var02->VariablePillCount(), 0u);

  // add variable to container - max size reached
  container->AddVariablePill(var01);
  QCOMPARE(container->VariablePillCount(), 1u);
  QCOMPARE(var01->Container(), container);

  // verify no more variables can be added
  container->AddVariablePill(var02);
  QCOMPARE(container->VariablePillCount(), 1u);
  QVERIFY(var02->Container() == nullptr);

  var01->AddVariablePill(var02);
  QCOMPARE(container->VariablePillCount(), 1u);
  QVERIFY(var02->Container() == nullptr);
  QVERIFY(var02->Parent() == nullptr);

  // remove variable and verify we can add a different variable now
  container->RemoveVariablePill(var01);
  QCOMPARE(container->VariablePillCount(), 0u);

  container->AddVariablePill(var02);
  QCOMPARE(container->VariablePillCount(), 1u);
  QCOMPARE(var02->Container(), container);

  delete var01;
  delete var02;
  delete container;
}

// Generate a main function for the test
QTEST_MAIN(VariablePillContainer_TEST)
