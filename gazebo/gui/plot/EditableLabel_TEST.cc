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

#include "gazebo/gui/plot/EditableLabel.hh"
#include "gazebo/gui/plot/EditableLabel_TEST.hh"

/////////////////////////////////////////////////
void EditableLabel_TEST::Text()
{
  // Create a new EditableLabel widget
  gazebo::gui::EditableLabel *editableLabel =
      new gazebo::gui::EditableLabel("test_label", nullptr);

  QVERIFY(editableLabel != nullptr);

  QCOMPARE(editableLabel->Text(), std::string("test_label"));

  delete editableLabel;
}

/////////////////////////////////////////////////
void EditableLabel_TEST::Edit()
{
  // Create a new EditableLabel widget
  gazebo::gui::EditableLabel *editableLabel =
      new gazebo::gui::EditableLabel("test_label", nullptr);

  QVERIFY(editableLabel != nullptr);

  QLineEdit *lineEdit = editableLabel->findChild<QLineEdit *>();
  QVERIFY(lineEdit != nullptr);

  QCOMPARE(editableLabel->Text(), std::string("test_label"));
  editableLabel->show();

  // test editing value and hitting enter to save
  QPoint center(editableLabel->width()*0.5, editableLabel->height()*0.5);
  QTest::mouseMove(editableLabel, center);
  this->ProcessEventsAndDraw(nullptr);
  QTest::mouseDClick(editableLabel, Qt::LeftButton, Qt::NoModifier,
      center, 100);
  this->ProcessEventsAndDraw(nullptr);
  for (unsigned int i = 0; i < 3; ++i)
  {
    QTest::keyClick(lineEdit, Qt::Key_A);
    this->ProcessEventsAndDraw(nullptr);
  }
  QTest::keyClick(lineEdit, Qt::Key_Enter);
  this->ProcessEventsAndDraw(nullptr);
  QCOMPARE(editableLabel->Text(), std::string("aaa"));

  // test editing value and hitting escape to forget changes
  QTest::mouseDClick(editableLabel, Qt::LeftButton, Qt::NoModifier,
      center, 100);
  this->ProcessEventsAndDraw(nullptr);
  for (unsigned int i = 0; i < 3; ++i)
  {
    QTest::keyClick(lineEdit, Qt::Key_B);
    this->ProcessEventsAndDraw(nullptr);
  }
  QTest::keyClick(lineEdit, Qt::Key_Escape);
  QCOMPARE(editableLabel->Text(), std::string("aaa"));

  delete editableLabel;
}

// Generate a main function for the test
QTEST_MAIN(EditableLabel_TEST)
