/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/LinkInspector.hh"
#include "gazebo/gui/model/LinkInspector_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void LinkInspector_TEST::RemoveButton()
{
/*
  this->dataPtr->connection =
      gui::model::Events::ConnectRequestLinkRemoval(
      boost::bind(&LinkInspector_TEST::RemoveLink, this, _1));

  // Create a link inspector
  gazebo::gui::LinkInspector *linkInspector =
      new gazebo::gui::LinkInspector(linkMaker);
  QVERIFY(linkInspector != NULL);

  // Get buttons
  QList<QToolButton *> toolButtons =
      linkInspector->findChildren<QToolButton *>();
  QVERIFY(toolButtons.size() == 1);

  // Trigger remove
  toolButtons[0]->click();

  delete linkInspector;
*/
}

// Generate a main function for the test
QTEST_MAIN(LinkInspector_TEST)
