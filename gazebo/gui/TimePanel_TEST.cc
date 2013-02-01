/*
 * Copyright 2011 Nate Koenig
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
void TimePanel_TEST::RecordButton()
{
  // Create a new data logger widget
  gazebo::gui::TimePanel *timePanel = new gazebo::gui::TimePanel;

  // Get the percent real time line
  QLineEdit *percentEdit = timePanel->findChild<QLineEdit*>(
      "timePanelPercentRealTime");

  // Get the sim time line
  QLineEdit *simTimeEdit = timePanel->findChild<QLineEdit*>(
      "timePanelSimTime");

  // Get the real time line
  QLineEdit *realTimeEdit = timePanel->findChild<QLineEdit*>(
      "timePanelRealTime");

  QVERIFY(percentEdit != NULL);
  QVERIFY(simTimeEdit != NULL);
  QVERIFY(realTimeEdit != NULL);

  gazebo::transport::NodePtr node;
  gazebo::transport::PublisherPtr pub;

  // Create a node from communication.
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  pub = node->Advertise<gazebo::msgs::LogControl>("~/world_stats");

  gazebo::msgs::WorldStatistics msg;
  gazebo::msgs::Set(msg.mutable_sim_time(), gazebo::common::Time(1, 2));
  gazebo::msgs::Set(msg.mutable_pause_time(), gazebo::common::Time(0, 5));
  gazebo::msgs::Set(msg.mutable_real_time(), gazebo::common::Time(123, 456));
  pub->Publish(msg);

  gazebo::common::Time::MSleep(100);
  QCoreApplication::processEvents();

  std::string txt;

  // Make sure the destination log file is correct.
  txt = realTimeEdit->text().toStdString();
  std::cout << "Real[" << txt << std::endl;
  // QVERIFY(txt.find("test/state.log") != std::string::npos);

  // Make sure the initial size is zero
  txt = simTimeEdit->text().toStdString();
  std::cout << "Sim[" << txt << std::endl;
  //QVERIFY(txt == "0.00 B");

  // Make sure the initial time is zero
  txt = percentEdit->text().toStdString();
  std::cout << "Percent[" << txt << std::endl;
  // QVERIFY(txt == "00:00:00.000");

  // Make sure the status label says "Recording"
  // txt = statusLabel->text().toStdString();
  //QVERIFY(txt == "Recording");

}
// Generate a main function for the test
QTEST_MAIN(TimePanel_TEST)
