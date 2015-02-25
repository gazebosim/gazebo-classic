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

#include <boost/filesystem.hpp>
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/gui/DataLogger.hh"
#include "gazebo/gui/DataLogger_TEST.hh"
#include "gazebo/math/Rand.hh"

/////////////////////////////////////////////////
void DataLogger_TEST::RecordButton()
{
  QBENCHMARK
  {
    this->Load("worlds/empty.world");

    // Create a new data logger widget
    gazebo::gui::DataLogger *dataLogger = new gazebo::gui::DataLogger;
    dataLogger->show();
    QCoreApplication::processEvents();

    // Get the record button
    QToolButton *recordButton = dataLogger->findChild<QToolButton*>(
        "dataLoggerRecordButton");

    // Get the destination label
    QLineEdit *destPathLabel = dataLogger->findChild<QLineEdit*>(
        "dataLoggerDestnationPathLabel");

    // Get the time label
    QLabel *timeLabel = dataLogger->findChild<QLabel*>("dataLoggerTimeLabel");

    // Get the status label
    QLabel *statusLabel =
      dataLogger->findChild<QLabel*>("dataLoggerStatusLabel");

    // Get the size label
    QLabel *sizeLabel = dataLogger->findChild<QLabel*>("dataLoggerSizeLabel");

    QVERIFY(recordButton != NULL);
    QVERIFY(destPathLabel != NULL);
    QVERIFY(sizeLabel != NULL);
    QVERIFY(timeLabel != NULL);
    QVERIFY(statusLabel != NULL);

    // Toggle the record button, which starts logging.
    recordButton->toggle();

    // Wait for a log status return message
    while (destPathLabel->text().toStdString().empty())
    {
      // The following line tell QT to process its events. This is vital for
      // all tests, but it must be run in the main thread.
      QCoreApplication::processEvents();
      gazebo::common::Time::MSleep(100);
    }

    std::string txt;

    // Make sure the destination log file is correct.
    txt = destPathLabel->text().toStdString();
    QVERIFY(txt.find("test/state.log") != std::string::npos);

    // Make sure the initial size is zero
    txt = sizeLabel->text().toStdString();
    QVERIFY(txt == "0.00 B");

    // Make sure the initial time is zero
    txt = timeLabel->text().toStdString();
    QVERIFY(txt == "00:00:00.000");

    // Make sure the status label says "Recording"
    txt = statusLabel->text().toStdString();
    QVERIFY(txt == "Recording");


    // Toggle the record button, which stops logging.
    recordButton->toggle();

    // Make sure the initial size is zero
    txt = sizeLabel->text().toStdString();
    QVERIFY(txt == "0.00 B");

    // Make sure the initial time is zero
    txt = timeLabel->text().toStdString();
    QVERIFY(txt == "00:00:00.000");


    // Make sure the status label says "Ready"
    txt = statusLabel->text().toStdString();
    QVERIFY(txt == "Ready");

    dataLogger->hide();
  }
}

/////////////////////////////////////////////////
void DataLogger_TEST::StressTest()
{
  QBENCHMARK
  {
    gazebo::common::SystemPaths *paths =
        gazebo::common::SystemPaths::Instance();

    // Cleanup test directory.
    boost::filesystem::remove_all(paths->GetDefaultTestPath());

    this->Load("worlds/empty.world");

    // Cleanup test directory.
    boost::filesystem::remove_all(paths->GetDefaultTestPath());

    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr pub;

    // Create a node from communication.
    node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node->Init();
    pub = node->Advertise<gazebo::msgs::LogControl>("~/log/control");

    gazebo::msgs::LogControl msg;
    msg.set_base_path(paths->GetDefaultTestPath());
    pub->Publish(msg);

    // Create a new data logger widget
    gazebo::gui::DataLogger *dataLogger = new gazebo::gui::DataLogger;

    // Get the record button
    QToolButton *recordButton = dataLogger->findChild<QToolButton*>(
        "dataLoggerRecordButton");

    unsigned int count = 100;

    // Toggle the record button many times with sleeps
    for (unsigned int i = 0; i < count; ++i)
    {
      recordButton->toggle();

      // Sleep for random times
      gazebo::common::Time::MSleep(gazebo::math::Rand::GetIntUniform(10, 500));
    }

    // There should be (count * 0.5) log directories in $TMP/gazebo_test
    // due to the record button being toggled.
    unsigned int dirCount = 0;
    for (boost::filesystem::directory_iterator
          iter(paths->GetDefaultTestPath());
          iter != boost::filesystem::directory_iterator(); ++iter, ++dirCount)
    {
    }

    // Cleanup after ourselves.
    boost::filesystem::remove_all(paths->GetDefaultTestPath());

    QVERIFY(dirCount == count / 2);
  }
}

// Generate a main function for the test
QTEST_MAIN(DataLogger_TEST)
