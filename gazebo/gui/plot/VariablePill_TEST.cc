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
#include "gazebo/gui/plot/VariablePill_TEST.hh"


/////////////////////////////////////////////////
void VariablePill_TEST::Variable()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // test various variable pill set/get functions
  gazebo::gui::VariablePill *var01 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var01 != nullptr);

  // name
  std::string varName = "var01name";
  var01->SetName(varName);
  QCOMPARE(var01->Name(), varName);

  // text
  std::string varText = "var01";
  var01->SetText(varText);
  QCOMPARE(var01->Text(), varText);

  // selected state
  QCOMPARE(var01->IsSelected(), false);

  var01->SetSelected(true);
  QCOMPARE(var01->IsSelected(), true);

  var01->SetSelected(false);
  QCOMPARE(var01->IsSelected(), false);

  // parent
  QVERIFY(var01->Parent() == nullptr);

  gazebo::gui::VariablePill *var02 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var02 != nullptr);

  var01->SetParent(var02);
  QCOMPARE(var01->Parent(), var02);

  delete var01;
  delete var02;
}

/////////////////////////////////////////////////
void VariablePill_TEST::VariableId()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // a set of unique variable ids
  std::set<unsigned int> ids;

  // Create new variable pills and verify they all have unique ids
  gazebo::gui::VariablePill *var01 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var01 != nullptr);
  unsigned int id = var01->Id();
  QVERIFY(id != gazebo::gui::VariablePill::EmptyVariable);
  QVERIFY(ids.count(id) == 0u);
  ids.insert(id);

  gazebo::gui::VariablePill *var02 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var02 != nullptr);
  id = var02->Id();
  QVERIFY(id != gazebo::gui::VariablePill::EmptyVariable);
  QVERIFY(ids.count(id) == 0u);
  ids.insert(id);

  gazebo::gui::VariablePill *var03 = new gazebo::gui::VariablePill(nullptr);
  QVERIFY(var03 != nullptr);
  id = var03->Id();
  QVERIFY(id != gazebo::gui::VariablePill::EmptyVariable);
  QVERIFY(ids.count(id) == 0u);
  ids.insert(id);

  delete var01;
  delete var02;
  delete var03;
}

/////////////////////////////////////////////////
void VariablePill_TEST::MultiVariable()
{
  // create 4 variable pills
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

  // add var02 to var01 - var01 becomes a multi-variable pill
  var01->AddVariablePill(var02);
  QCOMPARE(var01->VariablePillCount(), 1u);
  QVERIFY(var02->Parent() == var01);

  // verify we cannot add var03 to var02 because var02 already has a parent
  // this will implicitly add var03 to var02's parent
  var02->AddVariablePill(var03);
  QCOMPARE(var01->VariablePillCount(), 2u);
  QCOMPARE(var02->VariablePillCount(), 0u);
  QVERIFY(var02->Parent() == var01);
  QVERIFY(var03->Parent() == var01);

  // verify we can add var03 to var01 again
  // their the parent-child relationship should not change
  var01->AddVariablePill(var03);
  QCOMPARE(var01->VariablePillCount(), 2u);
  QVERIFY(var03->Parent() == var01);

  // move var03 from var01 to var04
  var04->AddVariablePill(var03);
  QCOMPARE(var01->VariablePillCount(), 1u);
  QCOMPARE(var04->VariablePillCount(), 1u);
  QVERIFY(var03->Parent() == var04);

  // remove var02 from var01
  var01->RemoveVariablePill(var02);
  QCOMPARE(var01->VariablePillCount(), 0u);
  QVERIFY(var02->Parent() == nullptr);

  // try remove var02 again - it should not do anything
  var01->RemoveVariablePill(var02);
  QCOMPARE(var01->VariablePillCount(), 0u);
  QVERIFY(var02->Parent() == nullptr);

  // remove var03 from var04
  var04->RemoveVariablePill(var03);
  QCOMPARE(var04->VariablePillCount(), 0u);
  QVERIFY(var03->Parent() == nullptr);

  delete var01;
  delete var02;
  delete var03;
  delete var04;
}

// Generate a main function for the test
QTEST_MAIN(VariablePill_TEST)
