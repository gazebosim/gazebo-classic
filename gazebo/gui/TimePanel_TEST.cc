/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/TimePanel_TEST.hh"

/////////////////////////////////////////////////
void TimePanel_TEST::SetPaused()
{
  this->Load("empty.world");

  // Create a new time panel widget
  gazebo::gui::TimePanel *timePanel = new gazebo::gui::TimePanel;
  QVERIFY(timePanel != NULL);

  // verify initial state
  QVERIFY(!timePanel->IsPaused());

  // set paused state and verify
  timePanel->SetPaused(true);
  QVERIFY(timePanel->IsPaused());

  timePanel->SetPaused(false);
  QVERIFY(!timePanel->IsPaused());
  delete timePanel;
}

// Generate a main function for the test
QTEST_MAIN(TimePanel_TEST)
