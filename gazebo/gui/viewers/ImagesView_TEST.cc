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

#include "gazebo/math/Rand.hh"
#include "gazebo/gui/viewers/ImagesView_TEST.hh"

/////////////////////////////////////////////////
void ImagesView_TEST::Construction()
{
  this->Load("worlds/empty.world");

  // Create a new data logger widget
  gazebo::gui::ImagesView *view = new gazebo::gui::ImagesView(NULL);
  view->show();

  QCoreApplication::processEvents();

  // Get the frame that holds the images
  QFrame *frame = view->findChild<QFrame*>("blackBorderFrame");

  // The layout should be the only child of the frame on construction.
  QVERIFY(frame->children().size() == 1);

  view->hide();
  delete view;
}

/////////////////////////////////////////////////
void ImagesView_TEST::Switch()
{
  this->Load("worlds/multicamera_test.world");

  // Create a new data logger widget
  gazebo::gui::ImagesView *view = new gazebo::gui::ImagesView(NULL);
  view->show();

  // Get the frame that holds the images
  QFrame *frame = view->findChild<QFrame*>("blackBorderFrame");

  // The layout should be the only child of the frame on construction.
  QVERIFY(frame->children().size() == 1);

  this->SetTopic(view, "~/multicamera_1/link/cam/images", 2);
  this->SetTopic(view, "~/multicamera_2/link/cam/images", 3);
  this->SetTopic(view, "~/multicamera_3/link/cam/images", 4);
  this->SetTopic(view, "~/multicamera_4/link/cam/images", 5);
  this->SetTopic(view, "~/multicamera_5/link/cam/images", 6);
  this->SetTopic(view, "~/multicamera_6/link/cam/images", 7);

  this->SetTopic(view, "~/multicamera_6/link/cam/images", 7);
  this->SetTopic(view, "~/multicamera_5/link/cam/images", 6);
  this->SetTopic(view, "~/multicamera_4/link/cam/images", 5);
  this->SetTopic(view, "~/multicamera_3/link/cam/images", 4);
  this->SetTopic(view, "~/multicamera_2/link/cam/images", 3);
  this->SetTopic(view, "~/multicamera_1/link/cam/images", 2);

  view->hide();
  delete view;
}

/////////////////////////////////////////////////
void ImagesView_TEST::FastSwitch()
{
  this->Load("worlds/multicamera_test.world");

  // Create a new data logger widget
  gazebo::gui::ImagesView *view = new gazebo::gui::ImagesView(NULL);
  view->show();

  // Get the frame that holds the images
  QFrame *frame = view->findChild<QFrame*>("blackBorderFrame");

  // The layout should be the only child of the frame on construction.
  QVERIFY(frame->children().size() == 1);

  std::map<int, std::string> topicMap;
  topicMap[2] = "~/multicamera_1/link/cam/images";
  topicMap[3] = "~/multicamera_2/link/cam/images";
  topicMap[4] = "~/multicamera_3/link/cam/images";
  topicMap[5] = "~/multicamera_4/link/cam/images";
  topicMap[6] = "~/multicamera_5/link/cam/images";
  topicMap[7] = "~/multicamera_6/link/cam/images";

  // Switch the topic 100 times
  for (unsigned int i = 0; i < 100; ++i)
  {
    // Get a random topic index.
    int index = gazebo::math::Rand::GetIntUniform(1,7);

    // Switch the topic
    view->SetTopic(topicMap[index]);

    // Sping the QT update loop for a while to process events.
    for (int j = 0; j < 50; ++j)
    {
      gazebo::common::Time::MSleep(gazebo::math::Rand::GetIntUniform(10,50));
      QCoreApplication::processEvents();
    }
  }

  view->hide();
  delete view;
}

/////////////////////////////////////////////////
void ImagesView_TEST::SetTopic(gazebo::gui::ImagesView *_view,
    const std::string &_topicName, int _count)
{
  QFrame *frame = _view->findChild<QFrame*>("blackBorderFrame");

  _view->SetTopic(_topicName);

  int i = 0;

  for (i = 0; frame->children().size() != 1 && i < 1000; ++i)
  {
    gazebo::common::Time::MSleep(10);
    QCoreApplication::processEvents();
  }

  // Make sure the loop didn't exceed the maximum number of iterations
  QVERIFY(i < 1000);

  for (i = 0; frame->children().size() != _count && i < 1000; ++i)
  {
    gazebo::common::Time::MSleep(10);
    QCoreApplication::processEvents();
  }

  // Make sure the loop didn't exceed the maximum number of iterations
  QVERIFY(i < 1000);

  // Wait a bit for the images to appear
  // This is done for visual confirmation that the test works.
  for (i = 0; i < 100; ++i)
  {
    gazebo::common::Time::MSleep(10);
    QCoreApplication::processEvents();
  }

  // Make sure the number of images is correct.
  QVERIFY(frame->children().size() == _count);
}

// Generate a main function for the test
QTEST_MAIN(ImagesView_TEST)
