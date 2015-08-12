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

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/model/ModelPluginInspector.hh"
#include "gazebo/gui/model/ModelPluginInspector_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelPluginInspector_TEST::Update()
{
  gazebo::gui::ModelPluginInspector *inspector =
      new gazebo::gui::ModelPluginInspector();
  QVERIFY(inspector != NULL);

  // Check fields
  QList<QLineEdit *> lineEdits = inspector->findChildren<QLineEdit *>();
  QList<QPlainTextEdit *> plainTextEdits =
      inspector->findChildren<QPlainTextEdit *>();
  QVERIFY(lineEdits.size() == 2u);
  QVERIFY(plainTextEdits.size() == 1u);

  QVERIFY(lineEdits[0]->text() == "");
  QVERIFY(lineEdits[1]->text() == "");
  QVERIFY(plainTextEdits[0]->toPlainText() == "");

  // Create a message
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<xml>1000</xml>\n";

  gazebo::msgs::Plugin pluginMsg;
  pluginMsg.set_name(name);
  pluginMsg.set_filename(filename);
  pluginMsg.set_innerxml(innerxml);
  gazebo::msgs::PluginPtr pluginPtr(new gazebo::msgs::Plugin);
  pluginPtr->CopyFrom(pluginMsg);

  // Update inspector
  inspector->Update(pluginPtr);

  // Check fields
  QVERIFY(lineEdits[0]->text() == QString::fromStdString(name));
  QVERIFY(lineEdits[1]->text() == QString::fromStdString(filename));
  QVERIFY(plainTextEdits[0]->toPlainText() == QString::fromStdString(innerxml));

  // Destructor
  delete inspector;
}

// Generate a main function for the test
QTEST_MAIN(ModelPluginInspector_TEST)
