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
#include <boost/filesystem.hpp>
#include <memory>
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/qtpropertybrowser/qttreepropertybrowser.h"
#include "gazebo/gui/qtpropertybrowser/qtvariantproperty.h"
#include "gazebo/gui/ModelListWidget.hh"
#include "gazebo/gui/ModelListWidget_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelListWidget_TEST::TreeWidget()
{
  QBENCHMARK
  {
    this->Load("worlds/empty.world");

    gazebo::gui::ModelListWidget *modelListWidget
        = new gazebo::gui::ModelListWidget;
    QCoreApplication::processEvents();

    // Get tree widget
    QTreeWidget *modelTreeWidget = modelListWidget->findChild<QTreeWidget*>(
        "modelTreeWidget");

    QVERIFY(modelTreeWidget != nullptr);

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
    QVERIFY(modelsItem != nullptr);

    delete modelListWidget;
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
void ModelListWidget_TEST::CheckVector3Property(QList<QtProperty *> _properties,
    const ignition::math::Vector3d &_xyz)
{
  QVERIFY(_properties.size() >= 3);
  for (unsigned char c = 0; c < 3; ++c)
  {
    QtVariantProperty *property =
      static_cast<QtVariantProperty *>(_properties[c]);
    Q_ASSERT(property);
    std::string name(1, 'x' + c);
    QCOMPARE(property->propertyName(), tr(name.c_str()));
    QCOMPARE(property->value().toDouble(), _xyz[c]);
  }
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::CheckPoseProperty(QList<QtProperty *> _properties,
    const ignition::math::Pose3d &_pose)
{
  QCOMPARE(_properties.size(), 6);
  this->CheckVector3Property(_properties, _pose.Pos());
  QtVariantProperty *property;
  property = static_cast<QtVariantProperty *>(_properties[3]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("roll"));
  QCOMPARE(property->value().toDouble(), _pose.Rot().Euler().X());
  property = static_cast<QtVariantProperty *>(_properties[4]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("pitch"));
  QCOMPARE(property->value().toDouble(), _pose.Rot().Euler().Y());
  property = static_cast<QtVariantProperty *>(_properties[5]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("yaw"));
  QCOMPARE(property->value().toDouble(), _pose.Rot().Euler().Z());
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::SetVector3Property(
    QtTreePropertyBrowser *_propTreeBrowser,
    QList<QtProperty *> _properties,
    const ignition::math::Vector3d &_xyz)
{
  QVERIFY(_properties.size() >= 3);
  for (unsigned char c = 0; c < 3; ++c)
  {
    QtVariantProperty *property =
      static_cast<QtVariantProperty *>(_properties[c]);
    Q_ASSERT(property);
    std::string name(1, 'x' + c);
    QCOMPARE(property->propertyName(), tr(name.c_str()));
    QVERIFY(_propTreeBrowser->items(property).size() == 1);
    _propTreeBrowser->setCurrentItem(_propTreeBrowser->items(property)[0]);
    property->setValue(_xyz[c]);
  }
  QTest::qWait(500);
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::SetPoseProperty(
    QtTreePropertyBrowser *_propTreeBrowser,
    QList<QtProperty *> _properties,
    const ignition::math::Pose3d &_pose)
{
  QCOMPARE(_properties.size(), 6);
  this->SetVector3Property(_propTreeBrowser, _properties, _pose.Pos());
  QtVariantProperty *property;
  property = static_cast<QtVariantProperty *>(_properties[3]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("roll"));
  QVERIFY(_propTreeBrowser->items(property).size() == 1);
  _propTreeBrowser->setCurrentItem(_propTreeBrowser->items(property)[0]);
  property->setValue(_pose.Rot().Euler().X());
  property = static_cast<QtVariantProperty *>(_properties[4]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("pitch"));
  QVERIFY(_propTreeBrowser->items(property).size() == 1);
  _propTreeBrowser->setCurrentItem(_propTreeBrowser->items(property)[0]);
  property->setValue(_pose.Rot().Euler().Y());
  property = static_cast<QtVariantProperty *>(_properties[5]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("yaw"));
  QVERIFY(_propTreeBrowser->items(property).size() == 1);
  _propTreeBrowser->setCurrentItem(_propTreeBrowser->items(property)[0]);
  property->setValue(_pose.Rot().Euler().Z());
  QTest::qWait(500);
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::CheckLinkProperty(QList<QtProperty *> _properties,
    const std::string &_name, bool _selfCollide, bool _gravity, bool _kinematic,
    bool _canonical, bool _enableWind, const ignition::math::Pose3d &_pose)
{
  // ignore checking link id in _properties[0]
  QtVariantProperty *property =
      static_cast<QtVariantProperty *>(_properties[1]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("name"));
  QCOMPARE(property->valueText(), tr(_name.c_str()));
  property = static_cast<QtVariantProperty *>(_properties[2]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("self_collide"));
  QCOMPARE(property->value().toBool(), _selfCollide);
  property = static_cast<QtVariantProperty *>(_properties[3]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("gravity"));
  QCOMPARE(property->value().toBool(), _gravity);
  property = static_cast<QtVariantProperty *>(_properties[4]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("kinematic"));
  QCOMPARE(property->value().toBool(), _kinematic);
  property = static_cast<QtVariantProperty *>(_properties[5]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("canonical"));
  QCOMPARE(property->value().toBool(), _canonical);
  property = static_cast<QtVariantProperty *>(_properties[6]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("enable_wind"));
  QCOMPARE(property->value().toBool(), _enableWind);
  property = static_cast<QtVariantProperty *>(_properties[7]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("pose"));
  CheckPoseProperty(property->subProperties(), _pose);
  // pose settings for canonical links should be disabled
  QCOMPARE(property->isEnabled(), !_canonical);

  /// TODO check inertial, collision, visual properties
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::SetLinkProperty(
    QtTreePropertyBrowser *propTreeBrowser, QList<QtProperty *> _properties,
    const std::string &_name, bool _selfCollide, bool _gravity, bool _kinematic,
    bool _canonical, bool _enableWind, const ignition::math::Pose3d &_pose)
{
  QtVariantProperty *property =
      static_cast<QtVariantProperty *>(_properties[1]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("name"));
  QVERIFY(propTreeBrowser->items(property).size() == 1);
  propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
  property->setValue(tr(_name.c_str()));
  property = static_cast<QtVariantProperty *>(_properties[2]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("self_collide"));
  QVERIFY(propTreeBrowser->items(property).size() == 1);
  propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
  property->setValue(_selfCollide);
  property = static_cast<QtVariantProperty *>(_properties[3]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("gravity"));
  QVERIFY(propTreeBrowser->items(property).size() == 1);
  propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
  property->setValue(_gravity);
  property = static_cast<QtVariantProperty *>(_properties[4]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("kinematic"));
  QVERIFY(propTreeBrowser->items(property).size() == 1);
  propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
  property->setValue(_kinematic);
  property = static_cast<QtVariantProperty *>(_properties[5]);
  Q_ASSERT(property);
  // only set the pose for non-canonical links
  QCOMPARE(property->propertyName(), tr("canonical"));
  QCOMPARE(property->value().toBool(), _canonical);
  if (!property->value().toBool())
  {
    property = static_cast<QtVariantProperty *>(_properties[7]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("pose"));
    QVERIFY(propTreeBrowser->items(property).size() == 1);
    propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
    propTreeBrowser->setExpanded(propTreeBrowser->topLevelItem(property), true);
    this->SetPoseProperty(propTreeBrowser, property->subProperties(), _pose);
  }
  property = static_cast<QtVariantProperty *>(_properties[6]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("enable_wind"));
  QVERIFY(propTreeBrowser->items(property).size() == 1);
  propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
  property->setValue(_enableWind);

  QTest::qWait(1000);
  /// TODO set inertial, collision, visual properties
}


/////////////////////////////////////////////////
void ModelListWidget_TEST::CheckPluginProperty(QList<QtProperty *> _properties,
    const std::string &_name, const std::string &_filename,
    const std::string &_innerxml)
{
  // Name
  auto property = static_cast<QtVariantProperty *>(_properties[0]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("name"));
  QCOMPARE(property->valueText(), tr(_name.c_str()));

  // Filename
  property = static_cast<QtVariantProperty *>(_properties[1]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("filename"));
  QCOMPARE(property->valueText(), tr(_filename.c_str()));

  // Inner XML
  property = static_cast<QtVariantProperty *>(_properties[2]);
  Q_ASSERT(property);
  QCOMPARE(property->propertyName(), tr("innerxml"));
  QCOMPARE(property->valueText(), tr(_innerxml.c_str()));
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::ModelsTree()
{
  gazebo::gui::ModelListWidget *modelListWidget
      = new gazebo::gui::ModelListWidget;
  QCoreApplication::processEvents();

  this->Load("worlds/shapes.world");

  gazebo::transport::NodePtr node;
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
  QVERIFY(modelsItem != nullptr);

  // verify that there are 4 models, ground plane, sphere, box, and cylinder
  int modelCount = 4;
  int maxSleep = 10;
  int sleep = 0;
  while (modelsItem->childCount() < modelCount && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(sleep < maxSleep);

  // process more events and make sure the child count remain the same
  sleep = 0;
  maxSleep = 5;
  while (sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    QCOMPARE(modelsItem->childCount(), modelCount);
    sleep++;
  }

  // find all models in the tree
  QTreeWidgetItem *groundPlaneItem = nullptr;
  QTreeWidgetItem *boxItem = nullptr;
  QTreeWidgetItem *sphereItem = nullptr;
  QTreeWidgetItem *cylinderItem = nullptr;
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
  QVERIFY(groundPlaneItem != nullptr);
  QVERIFY(boxItem != nullptr);
  QVERIFY(sphereItem != nullptr);
  QVERIFY(cylinderItem != nullptr);

  node.reset();
  delete requestMsg;
  delete modelListWidget;
  modelListWidget = nullptr;
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::ModelProperties()
{
  gazebo::gui::ModelListWidget *modelListWidget
      = new gazebo::gui::ModelListWidget;
  modelListWidget->show();
  modelListWidget->setGeometry(0, 0, 400, 800);
  QCoreApplication::processEvents();

  this->Load("worlds/multilink_shape.world");

  gazebo::transport::NodePtr node;
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
  QVERIFY(modelsItem != nullptr);

  // verify that there is only 1 model
  int modelCount = 1;
  int maxSleep = 10;
  int sleep = 0;
  while (modelsItem->childCount() < modelCount && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(sleep < maxSleep);

  // Get the model item
  QTreeWidgetItem *modelItem = modelsItem->child(0);
  QVERIFY(modelItem != nullptr);
  std::string modelName = "multilink";
  QCOMPARE(modelItem->text(0), tr(modelName.c_str()));

  // Get propery browser widget
  QObject *propTreeObj =
    modelListWidget->findChild<QObject *>("propTreeBrowser");
  QtTreePropertyBrowser *propTreeBrowser =
    dynamic_cast<QtTreePropertyBrowser *>(propTreeObj);

  QVERIFY(propTreeBrowser != nullptr);
  QCOMPARE(propTreeBrowser->properties().size(), 0);

  // select the models item
  QRect modelsRect = modelTreeWidget->visualItemRect(modelsItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelsRect.center() );
  QCoreApplication::processEvents();
  // wait for the model to be selected
  sleep = 0;
  maxSleep = 5;
  while (!modelsItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelsItem->isSelected());

  // select the multi-link model
  QRect modelRect = modelTreeWidget->visualItemRect(modelItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelRect.center());
  QCoreApplication::processEvents();
  sleep = 0;
  maxSleep = 5;
  while (!modelItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelItem->isSelected());

  // wait for the model properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // check the model properties
  bool hasName = false;
  bool hasStatic = false;
  bool hasPose = false;
  int numLinks = 0;
  int nameIndex = 1;
  QList<QtProperty *> modelProperties = propTreeBrowser->properties();
  for (int i = 0; i < modelProperties.size(); ++i)
  {
    QtVariantProperty *property =
      static_cast<QtVariantProperty *>(modelProperties[i]);
    if (property->propertyName() == tr("name"))
    {
      QCOMPARE(property->valueText(), tr(modelName.c_str()));
      hasName = true;
    }
    else if (property->propertyName() == tr("is_static"))
    {
      QCOMPARE(property->value().toBool(), false);
      hasStatic = true;
    }
    else if (property->propertyName() == tr("pose"))
    {
      this->CheckPoseProperty(property->subProperties(),
          ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
      hasPose = true;
    }
    else if (property->propertyName() == tr("link"))
    {
      QVERIFY(property->subProperties().size() > 0);
      if (property->subProperties()[nameIndex]->valueText().toStdString().
          find("box_link") != std::string::npos)
      {
        this->CheckLinkProperty(property->subProperties(),
          modelName + "::box_link", false, true, false, true, false,
          ignition::math::Pose3d(1.0, 0, 0, 0, 0, 0));
        numLinks++;
      }
      else if (property->subProperties()[nameIndex]->valueText().toStdString().
        find("sphere_link") != std::string::npos)
      {
        this->CheckLinkProperty(property->subProperties(),
            modelName + "::sphere_link", false, true, false, false, false,
            ignition::math::Pose3d(-1.5, 0, 0, 0, 0, 1.57));
        numLinks++;
      }
    }
  }
  QVERIFY(hasName);
  QVERIFY(hasStatic);
  QVERIFY(hasPose);
  // there should be 2 links
  QCOMPARE(numLinks, 2);

  numLinks = 0;
  // change model properties
  for (int i = 0; i < modelProperties.size(); ++i)
  {
    QtVariantProperty *property =
      static_cast<QtVariantProperty *>(modelProperties[i]);

    // TODO changing model name currently fails
    // if (property->propertyName() == tr("name"))
    // {
    //  propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
    //  property->setValue(true);
    // }
    if (property->propertyName() == tr("is_static"))
    {
      propTreeBrowser->setCurrentItem(propTreeBrowser->items(property)[0]);
      property->setValue(true);
    }
    else if (property->propertyName() == tr("pose"))
    {
      this->SetPoseProperty(propTreeBrowser, property->subProperties(),
          ignition::math::Pose3d(0, 0, 1.0, 0, 0, 0));
    }
    else if (property->propertyName() == tr("link"))
    {
      QVERIFY(property->subProperties().size() > 0);
      if (property->subProperties()[nameIndex]->valueText().toStdString().
          find("box_link") != std::string::npos)
      {
        this->SetLinkProperty(propTreeBrowser, property->subProperties(),
            modelName + "::box_link", true, false, true, true, false,
            ignition::math::Pose3d(1.5, 2.0, 3.2, 0.6, 0.7, 0.8));
        numLinks++;
      }
      else if (property->subProperties()[nameIndex]->valueText().toStdString().
        find("sphere_link") != std::string::npos)
      {
        this->SetLinkProperty(propTreeBrowser, property->subProperties(),
            modelName + "::sphere_link", true, false, true, false, false,
            ignition::math::Pose3d(-2.0, 0.5, 1.0, 3.14, 0, 0));
        numLinks++;
      }
    }
  }
  QCOMPARE(numLinks, 2);

  // select the multi-link model again to refresh the property browser
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelRect.center() );
  QCoreApplication::processEvents();
  QTest::qWait(100);
  sleep = 0;
  maxSleep = 5;
  while (!modelItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelItem->isSelected());
  // wait for the model properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  numLinks = 0;
  // verify the model properties are sucessfully set
  for (int i = 0; i < modelProperties.size(); ++i)
  {
    QtVariantProperty *property =
      static_cast<QtVariantProperty *>(modelProperties[i]);

    if (property->propertyName() == tr("is_static"))
    {
      QCOMPARE(property->value().toBool(), true);
    }
    else if (property->propertyName() == tr("pose"))
    {
      this->CheckPoseProperty(property->subProperties(),
          ignition::math::Pose3d(0, 0, 1.0, 0, 0, 0));
    }
    else if (property->propertyName() == tr("link"))
    {
      QVERIFY(property->subProperties().size() > 0);
      if (property->subProperties()[nameIndex]->valueText().toStdString().
          find("box_link") != std::string::npos)
      {
        // as this is the canonical link, the pose properties should remain
        // unchanged
        this->CheckLinkProperty(property->subProperties(),
            modelName + "::box_link", true, false, true, true, false,
            ignition::math::Pose3d(1.0, 0, 0, 0, 0, 0));
        numLinks++;
      }
      else if (property->subProperties()[nameIndex]->valueText().toStdString().
        find("sphere_link") != std::string::npos)
      {
        this->CheckLinkProperty(property->subProperties(),
            modelName + "::sphere_link", true, false, true, false, false,
            ignition::math::Pose3d(-2.0, 0.5, 1.0, 3.14, 0, 0));
        numLinks++;
      }
    }
  }
  QCOMPARE(numLinks, 2);

  modelListWidget->hide();
  node.reset();
  delete requestMsg;
  delete modelListWidget;
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::LinkProperties()
{
  gazebo::gui::ModelListWidget *modelListWidget
      = new gazebo::gui::ModelListWidget;
  modelListWidget->show();
  modelListWidget->setGeometry(0, 0, 400, 800);
  QCoreApplication::processEvents();

  this->Load("worlds/multilink_shape.world");

  gazebo::transport::NodePtr node;
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
  QVERIFY(modelsItem != nullptr);

  // verify that there is only 1 model
  int modelCount = 1;
  int maxSleep = 10;
  int sleep = 0;
  while (modelsItem->childCount() < modelCount && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(sleep < maxSleep);

  // Get the model item
  QTreeWidgetItem *modelItem = modelsItem->child(0);
  QVERIFY(modelItem != nullptr);
  std::string modelName = "multilink";
  QCOMPARE(modelItem->text(0), tr(modelName.c_str()));

  // Get propery browser widget
  QObject *propTreeObj =
    modelListWidget->findChild<QObject *>("propTreeBrowser");
  QtTreePropertyBrowser *propTreeBrowser =
    dynamic_cast<QtTreePropertyBrowser *>(propTreeObj);

  QVERIFY(propTreeBrowser != nullptr);
  QCOMPARE(propTreeBrowser->properties().size(), 0);

  // select the models item
  QRect modelsRect = modelTreeWidget->visualItemRect(modelsItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelsRect.center() );
  QCoreApplication::processEvents();
  // wait for the models item to be selected
  sleep = 0;
  maxSleep = 5;
  while (!modelsItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelsItem->isSelected());

  // select the multi-link model
  QRect modelRect = modelTreeWidget->visualItemRect(modelItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelRect.center());
  QCoreApplication::processEvents();
  sleep = 0;
  maxSleep = 5;
  while (!modelItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelItem->isSelected());

  // wait for the model properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // select the box link
  QTreeWidgetItem *boxLinkItem = modelItem->child(1);
  QVERIFY(boxLinkItem != nullptr);

  std::string boxLinkName = "box_link";
  QCOMPARE(boxLinkItem->text(0), tr(boxLinkName.c_str()));

  QRect boxLinkRect = modelTreeWidget->visualItemRect(boxLinkItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      boxLinkRect.center() );

  QCoreApplication::processEvents();
  sleep = 0;
  maxSleep = 5;
  while (!boxLinkItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(boxLinkItem->isSelected());

  // wait for the box link properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // check the box link properties
  this->CheckLinkProperty(propTreeBrowser->properties(),
      modelName + "::" + boxLinkName, false, true, false, true, false,
      ignition::math::Pose3d(1.0, 0, 0, 0, 0, 0));

  // change box link properties
  // TODO changing link name currently fails.
  this->SetLinkProperty(propTreeBrowser, propTreeBrowser->properties(),
      modelName + "::" + boxLinkName, true, false, true, true, false,
      ignition::math::Pose3d(2.5, 1.0, 4.2, 0.8, 0.5, 0.1));

  // select the box link again to refresh the property browser
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      boxLinkRect.center() );

  QCoreApplication::processEvents();
  QTest::qWait(100);
  sleep = 0;
  maxSleep = 5;
  while (!boxLinkItem->isSelected() && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(boxLinkItem->isSelected());
  // wait for the box link properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // verify the link properties are sucessfully set
  // the link is canonical so the pose should remain the same
  this->CheckLinkProperty(propTreeBrowser->properties(),
      modelName + "::" + boxLinkName, true, false, true, true, false,
      ignition::math::Pose3d(1.0, 0, 0, 0, 0, 0));

  // select the sphere link
  QTreeWidgetItem *sphereLinkItem = modelItem->child(2);
  QVERIFY(sphereLinkItem != nullptr);
  std::string sphereLinkName = "sphere_link";
  QCOMPARE(sphereLinkItem->text(0), tr(sphereLinkName.c_str()));

  QRect sphereLinkRect = modelTreeWidget->visualItemRect(sphereLinkItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      sphereLinkRect.center() );
  QCoreApplication::processEvents();
  QTest::qWait(100);
  sleep = 0;
  maxSleep = 5;
  while (!sphereLinkItem->isSelected() && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(sphereLinkItem->isSelected());

  // wait for the sphere link properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // check the sphere link properties
  this->CheckLinkProperty(propTreeBrowser->properties(),
      modelName + "::" + sphereLinkName, false, true, false, false, false,
      ignition::math::Pose3d(-1.5, 0, 0, 0, 0, 1.57));

  // change sphere link properties
  // TODO changing link name currently fails.
  this->SetLinkProperty(propTreeBrowser, propTreeBrowser->properties(),
      modelName + "::" + sphereLinkName, true, false, true, false, false,
      ignition::math::Pose3d(-2.0, 0.1, -1.2, 0, 1.57, 0));

  // select the sphere link again to refresh the property browser
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      sphereLinkRect.center() );
  QCoreApplication::processEvents();
  QTest::qWait(100);
  sleep = 0;
  maxSleep = 5;
  while (!sphereLinkItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(sphereLinkItem->isSelected());
  // wait for the sphere link properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  // verify the link properties are sucessfully set
  this->CheckLinkProperty(propTreeBrowser->properties(),
      modelName + "::" + sphereLinkName, true, false, true, false, false,
      ignition::math::Pose3d(-2.0, 0.1, -1.2, 0, 1.57, 0));

  modelListWidget->hide();
  node.reset();
  delete requestMsg;
  delete modelListWidget;
}


/////////////////////////////////////////////////
void ModelListWidget_TEST::PluginProperties()
{
  gazebo::gui::ModelListWidget *modelListWidget
      = new gazebo::gui::ModelListWidget;
  modelListWidget->show();
  modelListWidget->setGeometry(0, 0, 400, 2000);
  QCoreApplication::processEvents();

  this->Load("worlds/underwater.world");

  // Initialize transport
  gazebo::transport::NodePtr node;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::PublisherPtr requestPub =
      node->Advertise<gazebo::msgs::Request>("~/request");
  gazebo::transport::SubscriberPtr responseSub = node->Subscribe("~/response",
      &ModelListWidget_TEST::OnResponse, this);

  // Request list of entities
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
  QVERIFY(modelsItem != nullptr);

  // Verify that there are 5 models
  int modelCount = 5;
  int maxSleep = 10;
  int sleep = 0;
  while (modelsItem->childCount() < modelCount && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QCOMPARE(modelsItem->childCount(), modelCount);

  // Get propery browser widget
  QObject *propTreeObj =
    modelListWidget->findChild<QObject *>("propTreeBrowser");
  QtTreePropertyBrowser *propTreeBrowser =
    dynamic_cast<QtTreePropertyBrowser *>(propTreeObj);
  QVERIFY(propTreeBrowser != nullptr);

  // Check there are no properties yet
  QCOMPARE(propTreeBrowser->properties().size(), 0);

  // Select the models item
  QRect modelsRect = modelTreeWidget->visualItemRect(modelsItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelsRect.center() );
  QCoreApplication::processEvents();

  // Wait for the models item to be selected
  sleep = 0;
  while (!modelsItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelsItem->isSelected());

  // Get the submarine model item
  QTreeWidgetItem *modelItem = modelsItem->child(3);
  QVERIFY(modelItem != nullptr);
  std::string modelName = "submarine_buoyant";
  QCOMPARE(modelItem->text(0), tr(modelName.c_str()));

  // Select the submarine model
  QRect modelRect = modelTreeWidget->visualItemRect(modelItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      modelRect.center());
  QCoreApplication::processEvents();
  sleep = 0;
  maxSleep = 5;
  while (!modelItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(modelItem->isSelected());

  // Get the buoyancy plugin
  QTreeWidgetItem *pluginItem = modelItem->child(6);
  QVERIFY(pluginItem != nullptr);
  std::string pluginName = "buoyancy";
  QCOMPARE(pluginItem->text(0), tr(pluginName.c_str()));

  // Select the buoyancy plugin
  QRect pluginRect = modelTreeWidget->visualItemRect(pluginItem);
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      pluginRect.center() );
  QCoreApplication::processEvents();

  // Wait for the models plugin to be selected
  sleep = 0;
  maxSleep = 5;
  while (!pluginItem->isSelected() && sleep < maxSleep)
  {
    QTest::qWait(10);
    sleep++;
  }
  QVERIFY(pluginItem->isSelected());

  // Wait for the plugin properties to appear
  sleep = 0;
  maxSleep = 10;
  while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
  {
    QCoreApplication::processEvents();
    QTest::qWait(500);
    sleep++;
  }
  QVERIFY(propTreeBrowser->properties().size() > 0);

  std::string pluginFilename = "libBuoyancyPlugin.so";
  std::string plugininnerxml = "<fluid_density>999.1026</fluid_density> ";

  // check the buoyancy properties
  this->CheckPluginProperty(propTreeBrowser->properties(), pluginName,
      pluginFilename, plugininnerxml);

  modelListWidget->hide();
  node->Fini();
  delete requestMsg;
  delete modelListWidget;
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::PhysicsProperties()
{
  std::unique_ptr<gazebo::gui::ModelListWidget> modelListWidget(
      new gazebo::gui::ModelListWidget);
  modelListWidget->show();
  modelListWidget->setGeometry(0, 0, 400, 800);
  QCoreApplication::processEvents();

  this->Load("worlds/empty.world");

  // Verify that property browser widget is initially empty
  QObject *propTreeObj =
    modelListWidget->findChild<QObject *>("propTreeBrowser");
  QtTreePropertyBrowser *propTreeBrowser =
    dynamic_cast<QtTreePropertyBrowser *>(propTreeObj);
  QVERIFY(propTreeBrowser != nullptr);
  QCOMPARE(propTreeBrowser->properties().size(), 0);

  // Get the physics item from the model tree
  QTreeWidget *modelTreeWidget = modelListWidget->findChild<QTreeWidget *>(
      "modelTreeWidget");
  QList<QTreeWidgetItem *> treePhysicsItems =
    modelTreeWidget->findItems(tr("Physics"), Qt::MatchExactly);
  QCOMPARE(treePhysicsItems.size(), 1);
  QTreeWidgetItem *physicsItem = treePhysicsItems.front();
  QVERIFY(physicsItem != nullptr);

  // select the physics item after giving it time to be rendered
  QTest::qWait(10);
  QRect physicsRect;
  {
    physicsRect = modelTreeWidget->visualItemRect(physicsItem);
    QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
        physicsRect.center() );
    QCoreApplication::processEvents();
    // wait for the physics item to be selected
    int sleep = 0;
    int maxSleep = 5;
    while (!physicsItem->isSelected() && sleep < maxSleep)
    {
      QTest::qWait(10);
      sleep++;
    }
    QVERIFY(physicsItem->isSelected());
  }

  // wait for the physics properties to appear
  {
    int sleep = 0;
    int maxSleep = 10;
    while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
    {
      QCoreApplication::processEvents();
      QTest::qWait(500);
      sleep++;
    }
    QVERIFY(propTreeBrowser->properties().size() > 0);
  }

  // verify that there are 8 physics properties
  QCOMPARE(propTreeBrowser->properties().size(), 8);

  // check default values of each parameter
  // physics engine type: ODE
  {
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[0]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("physics engine"));
    QCOMPARE(property->valueText(), tr("ODE"));
  }

  // physics enabled: true
  {
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[1]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("enable physics"));
    QCOMPARE(property->value().toBool(), true);
  }

  // real time update rate: 1000
  {
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[2]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("real time update rate"));
    QCOMPARE(property->value().toDouble(), 1e3);
  }

  // max step size:  0.001
  {
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[3]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("max step size"));
    QCOMPARE(property->value().toDouble(), 1e-3);
  }

  // gravity: 0 0 -9.8
  {
    const ignition::math::Vector3d gravity(0, 0, -9.8);
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[4]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("gravity"));
    this->CheckVector3Property(property->subProperties(), gravity);
    // set gravity to 0 0 0
    this->SetVector3Property(propTreeBrowser, property->subProperties(),
        ignition::math::Vector3d::Zero);
  }

  // magnetic field: 6e-6 2.3e-5 -4.2e-5
  {
    const ignition::math::Vector3d xyz(6e-6, 2.3e-5, -4.2e-5);
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[5]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("magnetic field"));
    CheckVector3Property(property->subProperties(), xyz);
  }

  // select the box link again to refresh the property browser
  QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
      physicsRect.center() );

  {
    QCoreApplication::processEvents();
    QTest::qWait(100);
    int sleep = 0;
    int maxSleep = 5;
    while (!physicsItem->isSelected() && sleep < maxSleep)
    {
      QCoreApplication::processEvents();
      QTest::qWait(10);
      sleep++;
    }
    QVERIFY(physicsItem->isSelected());
  }

  // wait for the box link properties to appear
  {
    int sleep = 0;
    int maxSleep = 10;
    while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
    {
      QCoreApplication::processEvents();
      QTest::qWait(500);
      sleep++;
    }
    QVERIFY(propTreeBrowser->properties().size() > 0);
  }

  // check gravity: 0 0 0
  {
    auto properties = propTreeBrowser->properties();
    QtVariantProperty *property =
        static_cast<QtVariantProperty *>(properties[4]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("gravity"));
    this->CheckVector3Property(property->subProperties(),
        ignition::math::Vector3d::Zero);
  }

  modelListWidget->hide();
}

/////////////////////////////////////////////////
void ModelListWidget_TEST::GUIProperties()
{
  this->Load("worlds/empty.world", false, false, true);

  auto scene = gazebo::rendering::get_scene();
  scene->SetGrid(true);
  auto camera = scene->CreateUserCamera("gzclient_camera");
  gazebo::gui::set_active_camera(camera);

  // Get the model list widget
  std::unique_ptr<gazebo::gui::ModelListWidget> modelListWidget(
      new gazebo::gui::ModelListWidget);
  modelListWidget->show();
  modelListWidget->setGeometry(0, 0, 400, 800);
  QCoreApplication::processEvents();

  // Verify that property browser widget is initially empty
  auto propTreeObj = modelListWidget->findChild<QObject *>("propTreeBrowser");
  auto propTreeBrowser = dynamic_cast<QtTreePropertyBrowser *>(propTreeObj);
  QVERIFY(propTreeBrowser != nullptr);
  QCOMPARE(propTreeBrowser->properties().size(), 0);

  // Get the gui item from the model tree
  auto modelTreeWidget = modelListWidget->findChild<QTreeWidget *>(
      "modelTreeWidget");
  auto treeGUIItems = modelTreeWidget->findItems(tr("GUI"), Qt::MatchExactly);
  QCOMPARE(treeGUIItems.size(), 1);
  QTreeWidgetItem *guiItem = treeGUIItems.front();
  QVERIFY(guiItem != nullptr);

  // select the gui item after giving it time to be rendered
  QTest::qWait(10);
  QRect guiRect;
  {
    guiRect = modelTreeWidget->visualItemRect(guiItem);
    QTest::mouseClick(modelTreeWidget->viewport(), Qt::LeftButton, 0,
        guiRect.center() );
    QCoreApplication::processEvents();

    // wait for the gui item to be selected
    int sleep = 0;
    int maxSleep = 5;
    while (!guiItem->isSelected() && sleep < maxSleep)
    {
      QTest::qWait(10);
      sleep++;
    }
    QVERIFY(guiItem->isSelected());
  }

  // Wait for the gui properties to appear
  {
    int sleep = 0;
    int maxSleep = 10;
    while (propTreeBrowser->properties().size() == 0 && sleep < maxSleep)
    {
      QCoreApplication::processEvents();
      QTest::qWait(500);
      sleep++;
    }
    QVERIFY(propTreeBrowser->properties().size() > 0);
  }

  // Verify that there are 2 gui properties
  auto properties = propTreeBrowser->properties();
  QCOMPARE(properties.size(), 2);

  // Check default values of each parameter
  // Camera
  {
    auto property = static_cast<QtVariantProperty *>(properties[0]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("camera"));

    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[0]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("name"));
      QCOMPARE(prop->valueText(), QString("gzclient_camera"));
    }
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[1]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("clip"));
    }
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[2]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("pose"));
    }
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[3]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("track_visual"));
    }
  }

  // Grid
  {
    auto property = static_cast<QtVariantProperty *>(properties[1]);
    Q_ASSERT(property);
    QCOMPARE(property->propertyName(), tr("grid"));
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[0]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("cell count"));
      QCOMPARE(prop->value().toInt(), 20);
    }
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[1]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("cell size"));
      QCOMPARE(prop->value().toDouble(), 1.0);
    }
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[2]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("normal cell count"));
      QCOMPARE(prop->value().toInt(), 0);
    }
    {
      auto prop =
          static_cast<QtVariantProperty *>(property->subProperties()[3]);
      Q_ASSERT(prop);
      QCOMPARE(prop->propertyName(), tr("height offset"));
      QVERIFY(fabs(prop->value().toDouble() - 0.015) < 1e-6);
    }
  }

  modelListWidget->hide();
}

// Generate a main function for the test
QTEST_MAIN(ModelListWidget_TEST)
