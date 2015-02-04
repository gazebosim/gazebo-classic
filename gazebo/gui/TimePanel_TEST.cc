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
void TimePanel_TEST::ValidTimes()
{
  QBENCHMARK
  {
    this->Load("empty.world");

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

    // Wait a little bit so that time increases.
    for (unsigned int i = 0; i < 10; ++i)
    {
      gazebo::common::Time::MSleep(100);
      QCoreApplication::processEvents();
    }

    std::string txt;
    double value;

    // Make sure real time is greater than zero
    txt = realTimeEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
    QVERIFY(value > 0.0);

    // Make sure sim time is greater than zero
    txt = simTimeEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(txt.find(".")));
    QVERIFY(value > 0.0);

    // Make sure the percent real time is greater than zero
    txt = percentEdit->text().toStdString();
    value = boost::lexical_cast<double>(txt.substr(0, txt.find(" ")));
    QVERIFY(value > 0.0);
  }
}

// Generate a main function for the test
QTEST_MAIN(TimePanel_TEST)
