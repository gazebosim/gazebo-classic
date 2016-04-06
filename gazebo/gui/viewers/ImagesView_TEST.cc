/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <memory>
#include <ignition/math/Rand.hh>

#include "gazebo/gui/viewers/ImagesView_TEST.hh"

/////////////////////////////////////////////////
void ImagesView_TEST::Construction()
{
  this->Load("worlds/empty.world");

  // Create a new data logger widget
  std::unique_ptr<gazebo::gui::ImagesView> view(
      new gazebo::gui::ImagesView(NULL));
  view->show();

  QCoreApplication::processEvents();

  // Get the frame that holds the images
  QFrame *frame = view->findChild<QFrame*>("blackBorderFrame");

  // The layout should be the only child of the frame on construction.
  QVERIFY(frame->children().size() == 1);

  view->hide();
}

/////////////////////////////////////////////////
void ImagesView_TEST::Switch()
{
  this->Load("worlds/multicamera_test.world");

  // Create a new data logger widget
  std::unique_ptr<gazebo::gui::ImagesView> view(
      new gazebo::gui::ImagesView(NULL));
  view->show();

  // Get the frame that holds the images
  QFrame *frame = view->findChild<QFrame *>("blackBorderFrame");
  QVERIFY(frame != NULL);

  // The layout should be the only child of the frame on construction.
  QVERIFY(frame->children().size() == 1);

  std::map<int, std::string> topicMap;
  topicMap[2] = "~/multicamera_1/link/cam1/images";
  topicMap[3] = "~/multicamera_2/link/cam2/images";
  topicMap[4] = "~/multicamera_3/link/cam3/images";
  topicMap[5] = "~/multicamera_4/link/cam4/images";
  topicMap[6] = "~/multicamera_5/link/cam5/images";
  topicMap[7] = "~/multicamera_6/link/cam6/images";

  gzmsg << "cam1 to cam6" << std::endl;
  for (int i = 2; i < 8; ++i)
    this->SetTopic(view.get(), topicMap[i], i);

  gzmsg << "cam6 to cam1" << std::endl;
  for (int i = 7; i > 1; --i)
    this->SetTopic(view.get(), topicMap[i], i);

  // Switch the topic 25 times
  for (unsigned int i = 0; i < 25; ++i)
  {
    // Get a random topic index.
    int index = ignition::math::Rand::IntUniform(1, 7);

    gzmsg << "Random switch [" << i + 1 << "] of 25. Number of images ["
        << index - 1 << "]" << std::endl;

    // Switch the topic
    view->SetTopic(topicMap[index]);

    // Spin the QT update loop for a while to process events.
    this->ProcessEventsAndDraw();
  }

  view->hide();
}

/////////////////////////////////////////////////
void ImagesView_TEST::SetTopic(gazebo::gui::ImagesView *_view,
    const std::string &_topicName, int _count)
{
  QFrame *frame = _view->findChild<QFrame*>("blackBorderFrame");
  QVERIFY(frame != NULL);

  _view->SetTopic(_topicName);

  int i = 0;

  // Make sure images were cleared and there's only one child left
  // Unless it has already loaded the new number of images
  for (i = 0; frame->children().size() != 1 && i < 100 &&
              frame->children().size() != _count; ++i)
  {
    gazebo::common::Time::MSleep(10);
    QCoreApplication::processEvents();
  }
  QVERIFY(frame->children().size() == _count
       || frame->children().size() == 1);

  // Wait a bit for the images to appear
  for (i = 0; frame->children().size() != _count && i < 100; ++i)
  {
    gazebo::common::Time::MSleep(10);
    QCoreApplication::processEvents();
  }

  // Make sure the number of images is correct.
  QCOMPARE(frame->children().size(), _count);
}

// Generate a main function for the test
QTEST_MAIN(ImagesView_TEST)
