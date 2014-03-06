/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/math/Helpers.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gazebo/gui/qtpropertybrowser/qtvariantproperty.h"
#include "gazebo/gui/ModelListWidget.hh"
#include "gazebo/gui/ModelListWidget_TEST.hh"

#include "test_config.h"
/*
bool g_gotSetWireframe = false;
void OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "set_wireframe")
    g_gotSetWireframe = true;
}*/

/////////////////////////////////////////////////
void ModelListWidget_TEST::PropertyTree()
{
  QBENCHMARK
  {
    this->Load("worlds/empty.world");

    gazebo::gui::ModelListWidget *modelListWidget
        = new gazebo::gui::ModelListWidget;
    modelListWidget->show();
    QCoreApplication::processEvents();

    // Get tree widget
    QTreeWidget *modelTreeWidget = modelListWidget->findChild<QTreeWidget*>(
        "modelTreeWidget");

    QVERIFY(modelTreeWidget != NULL);

    QList<QTreeWidgetItem *> treeSceneItems =
        modelTreeWidget->findItems(tr("Scene"), Qt::MatchExactly);
    QCOMPARE(treeSceneItems.size(), 1);

    QList<QTreeWidgetItem *> treePhysicsItems =
      modelTreeWidget->findItems(tr("Physics"), Qt::MatchExactly);
    QCOMPARE(treePhysicsItems.size(), 1);

    QList<QTreeWidgetItem *> treeLightItems =
        modelTreeWidget->findItems(tr("Lights"), Qt::MatchExactly);
    QCOMPARE(treeLightItems.size(), 1);

    QList<QTreeWidgetItem *> treeModelItems =
        modelTreeWidget->findItems(tr("Models"), Qt::MatchExactly);
    QCOMPARE(treeModelItems.size(), 1);

    QTreeWidgetItem *modelsItem = treeModelItems.front();
    QVERIFY(modelsItem != NULL);


    modelListWidget->hide();
  }
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::OnResponse(ConstResponsePtr &_msg)
{
  gazebo::msgs::Model_V modelVMsg;
  if (_msg->has_type() && _msg->type() == modelVMsg.GetTypeName())
  {
    modelVMsg.ParseFromString(_msg->serialized_data());

    for (int i = 0; i < modelVMsg.models_size(); i++)
    {
      gazebo::gui::Events::modelUpdate(modelVMsg.models(i));
    }
  }
}


/////////////////////////////////////////////////
void ModelListWidget_TEST::CheckPoseProperty(QList<QtProperty *> _properties,
    const gazebo::math::Pose &_pose)
{

}

/////////////////////////////////////////////////
void ModelListWidget_TEST::ModelProperties()
{
  gazebo::gui::ModelListWidget *modelListWidget
      = new gazebo::gui::ModelListWidget;
  modelListWidget->show();
  QCoreApplication::processEvents();

  this->Load("worlds/shapes.world");

  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr sub;

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::PublisherPtr requestPub =
      node->Advertise<gazebo::msgs::Request>("~/request");
  gazebo::transport::SubscriberPtr responseSub = node->Subscribe("~/response",
      &ModelListWidget_TEST::OnResponse, this);

  gazebo::msgs::Request *requestMsg =
      gazebo::msgs::CreateRequest("entity_list");
  requestPub->Publish(*requestMsg);

  // Get tree widget
  QTreeWidget *modelTreeWidget = modelListWidget->findChild<QTreeWidget *>(
      "modelTreeWidget");

  QList<QTreeWidgetItem *> treeModelItems =
      modelTreeWidget->findItems(tr("Models"), Qt::MatchExactly);
  QCOMPARE(treeModelItems.size(), 1);

  QTreeWidgetItem *modelsItem = treeModelItems.front();
  QVERIFY(modelsItem != NULL);

  // verify that there are 4 models, ground plane, sphere, box, and cylinder
  int modelCount = 4;
  int maxSleep = 10;
  int sleep = 0;
  while (modelsItem->childCount() < modelCount && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    gazebo::common::Time::MSleep(500);
    sleep++;
  }
  QVERIFY(sleep < maxSleep);

  // process more events and make sure the child count remain the same
  sleep = 0;
  maxSleep = 5;
  while (sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    gazebo::common::Time::MSleep(500);
    QCOMPARE(modelsItem->childCount(), modelCount);
    sleep++;
  }

  // find all models in the tree
  QTreeWidgetItem *groundPlaneItem = NULL;
  QTreeWidgetItem *boxItem = NULL;
  QTreeWidgetItem *sphereItem = NULL;
  QTreeWidgetItem *cylinderItem = NULL;
  for (int i = 0; i < modelsItem->childCount(); ++i)
  {
    QTreeWidgetItem *item = modelsItem->child(i);
    if (item->text(0) == "ground_plane")
      groundPlaneItem = item;
    else if (item->text(0) == "box")
      boxItem = item;
    else if (item->text(0) == "sphere")
      sphereItem = item;
    else if (item->text(0) == "cylinder")
      cylinderItem = item;
  }

  // verify all models are present
  QVERIFY(groundPlaneItem != NULL);
  QVERIFY(boxItem != NULL);
  QVERIFY(sphereItem != NULL);
  QVERIFY(cylinderItem != NULL);

  // Get tree widget
  QtTreePropertyBrowser *propTreeBrowser =
    modelListWidget->findChild<QtTreePropertyBrowser *>(
      "propTreeBrowser");

  QVERIFY(propTreeBrowser != NULL);
  QCOMPARE(propTreeBrowser->properties().size(), 0);


  // select the model
  QRect modelsRect = modelTreeWidget->visualItemRect(modelsItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelsRect.center() );
  QCoreApplication::processEvents();
  // wait for the model to be selected
  sleep = 0;
  maxSleep = 5;
  while (!modelsItem->isSelected() && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(10);
    sleep++;
  }
  QVERIFY(modelsItem->isSelected());

  // select the box
  QRect boxRect = modelTreeWidget->visualItemRect(boxItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      boxRect.center() );
  QCoreApplication::processEvents();
  // wait for the box to be selected
  sleep = 0;
  maxSleep = 5;
  while (!boxItem->isSelected() && sleep < maxSleep)
  {
    gazebo::common::Time::MSleep(10);
    sleep++;
  }
  QVERIFY(boxItem->isSelected());


  // wait for the box properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    gazebo::common::Time::MSleep(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // check the box properties
  bool hasName = false;
  bool hasStatic = false;
  bool hasPose = false;
  QList<QtProperty *> modelProperties = propTreeBrowser->properties();
  for (int i = 0; i < modelProperties.size(); ++i)
  {
    QtProperty *property = modelProperties[i];
    if (property->propertyName() == tr("name"))
    {
      QCOMPARE(property->valueText(), tr("box"));
      hasName = true;
    }
    else if (property->propertyName() == tr("is_static"))
    {
      QCOMPARE(property->valueText(), tr("False"));
      hasStatic = true;
    }
    else if (property->propertyName() == tr("pose"))
    {
      QList<QtProperty *> modelPoseProperties = property->subProperties();
      this->CheckPoseProperty(modelPoseProperties,
          gazebo::math::Pose(0, 0, 0.5, 0, 0, 0));
      hasPose = true;
    }
    else if (property->propertyName() == tr("link"))
    {
    }
  }
  QVERIFY(hasName);
  QVERIFY(hasStatic);
  QVERIFY(hasPose);


  modelListWidget->hide();
  delete requestMsg;
  delete modelListWidget;
}

/*
/////////////////////////////////////////////////
void ModelListWidget_TEST::NonDefaultWorld()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  boost::filesystem::path path = TEST_PATH;
  path = path / "worlds" / "empty_different_name.world";
  this->Load(path.string(), false, false, true);

  // Create the main window.
  gazebo::gui::ModelListWidget *ModelListWidget = new gazebo::gui::ModelListWidget();
  QVERIFY(ModelListWidget != NULL);
  ModelListWidget->Load();
  ModelListWidget->Init();
  ModelListWidget->show();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    ModelListWidget->repaint();
  }

  // Get the user camera, and tell it to save frames
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();

  if (!cam)
    return;

  cam->SetCaptureData(true);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    ModelListWidget->repaint();
  }

  // Get the image data
  const unsigned char *image = cam->GetImageData();
  unsigned int height = cam->GetImageHeight();
  unsigned int width = cam->GetImageWidth();
  unsigned int depth = 3;

  unsigned int sum = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*depth; ++x)
    {
      unsigned int a = image[(y*width*depth)+x];
      sum += a;
    }
  }

  QVERIFY(sum > 0);

  cam->Fini();
  ModelListWidget->close();
  delete ModelListWidget;
}*/

// Generate a main function for the test
QTEST_MAIN(ModelListWidget_TEST)
