/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/math/Box.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelAlign.hh"

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/gui/ModelAlign_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXMin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "min", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetMinX = modelVisuals[0]->GetWorldPose().pos.x +
      centerOffsets[0].x - targetBbox.GetXLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double minX = vis->GetWorldPose().pos.x + centerOffsets[i].x -
        bbox.GetXLength()/2.0;
    QVERIFY(gazebo::math::equal(minX, targetMinX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXCenter()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "center", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetCenterX = modelVisuals[0]->GetWorldPose().pos.x +
      centerOffsets[0].x;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double centerX = vis->GetWorldPose().pos.x + centerOffsets[i].x;
    QVERIFY(gazebo::math::equal(centerX, targetCenterX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignXMax()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "max", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetMaxX = modelVisuals[0]->GetWorldPose().pos.x +
      centerOffsets[0].x + targetBbox.GetXLength()/2.0;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double maxX = vis->GetWorldPose().pos.x + centerOffsets[i].x
        + bbox.GetXLength()/2.0;
    QVERIFY(gazebo::math::equal(maxX, targetMaxX, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYMin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "min", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetMinY = modelVisuals[0]->GetWorldPose().pos.y +
      centerOffsets[0].y - targetBbox.GetYLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double minY = vis->GetWorldPose().pos.y + centerOffsets[i].y -
        bbox.GetYLength()/2.0;
    QVERIFY(gazebo::math::equal(minY, targetMinY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYCenter()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "center", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetCenterY = modelVisuals[0]->GetWorldPose().pos.y +
      centerOffsets[0].y;


  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double centerY = vis->GetWorldPose().pos.y + centerOffsets[i].y;
    QVERIFY(gazebo::math::equal(centerY, targetCenterY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignYMax()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "y", "max", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetMaxY = modelVisuals[0]->GetWorldPose().pos.y +
      centerOffsets[0].y + targetBbox.GetYLength()/2.0;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double maxY = vis->GetWorldPose().pos.y + centerOffsets[i].y
        + bbox.GetYLength()/2.0;
    QVERIFY(gazebo::math::equal(maxY, targetMaxY, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZMin()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "min", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetMinZ = modelVisuals[0]->GetWorldPose().pos.z +
      centerOffsets[0].z - targetBbox.GetZLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double minZ = vis->GetWorldPose().pos.z + centerOffsets[i].z -
        bbox.GetZLength()/2.0;
    QVERIFY(gazebo::math::equal(minZ, targetMinZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZCenter()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "center", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetCenterZ = modelVisuals[0]->GetWorldPose().pos.z +
      centerOffsets[0].z;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double centerZ = vis->GetWorldPose().pos.z + centerOffsets[i].z;
    QVERIFY(gazebo::math::equal(centerZ, targetCenterZ, 1e-5));
  }
}

/////////////////////////////////////////////////
void ModelAlign_TEST::AlignZMax()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/align.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("Dumpster");
  modelNames.push_back("bookshelf");
  modelNames.push_back("table");
  modelNames.push_back("jersey_barrier");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "z", "max", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();

  double targetMaxZ = modelVisuals[0]->GetWorldPose().pos.z +
      centerOffsets[0].z + targetBbox.GetZLength()/2.0;

  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();

    double maxZ = vis->GetWorldPose().pos.z + centerOffsets[i].z
        + bbox.GetZLength()/2.0;
    QVERIFY(gazebo::math::equal(maxZ, targetMaxZ, 1e-5));
  }
}


/////////////////////////////////////////////////
void ModelAlign_TEST::AlignScale()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, true);

  gazebo::rendering::ScenePtr scene;
  scene = gazebo::rendering::get_scene("default");
  QVERIFY(scene != NULL);

  std::vector<std::string> modelNames;
  modelNames.push_back("box");
  modelNames.push_back("cylinder");
  modelNames.push_back("sphere");

  gazebo::event::Events::preRender();

  int sleep  = 0;
  int maxSleep = 200;
  while (!scene->GetInitialized() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  sleep = 0;
  unsigned int modelVisualCount = 0;
  while (modelVisualCount != modelNames.size() && sleep < maxSleep)
  {
    gazebo::event::Events::preRender();
    modelVisualCount = 0;
    for (unsigned int i = 0; i < modelNames.size(); ++i)
    {
      if (scene->GetVisual(modelNames[i]))
        modelVisualCount++;
    }
    gazebo::common::Time::MSleep(30);
    sleep++;
  }

  std::vector<gazebo::rendering::VisualPtr> modelVisuals;
  std::vector<gazebo::math::Vector3> centerOffsets;
  for (unsigned int i = 0; i < modelNames.size(); ++i)
  {
    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelNames[i]);
    QVERIFY(modelVis != NULL);
    gazebo::math::Vector3 modelCenterOffset =
        modelVis->GetBoundingBox().GetCenter();
    modelVisuals.push_back(modelVis);
    centerOffsets.push_back(modelCenterOffset);
  }

  // manually change scale of model visual and verify
  gazebo::rendering::VisualPtr targetVis = modelVisuals[0];
  targetVis->SetScale(gazebo::math::Vector3(1.5, 1, 1));
  QVERIFY(targetVis->GetScale() == gazebo::math::Vector3(1.5, 1, 1));

  gazebo::gui::ModelAlign::Instance()->Init();
  gazebo::gui::ModelAlign::Instance()->AlignVisuals(
      modelVisuals, "x", "min", "first");

  gazebo::math::Box targetBbox = modelVisuals[0]->GetBoundingBox();
  gazebo::math::Vector3 targetScale = modelVisuals[0]->GetScale();

  // verify other models align at minx of the scaled target model
  double targetMinX = modelVisuals[0]->GetWorldPose().pos.x +
      centerOffsets[0].x - targetScale.x * targetBbox.GetXLength()/2.0;
  for (unsigned int i = 1; i < modelVisuals.size(); ++i)
  {
    gazebo::rendering::VisualPtr vis = modelVisuals[i];
    gazebo::math::Box bbox = vis->GetBoundingBox();
    gazebo::math::Vector3 visScale = vis->GetScale();

    double minX = vis->GetWorldPose().pos.x + centerOffsets[i].x -
        visScale.x * bbox.GetXLength()/2.0;
    QVERIFY(gazebo::math::equal(minX, targetMinX, 1e-5));
  }
}

// Generate a main function for the test
QTEST_MAIN(ModelAlign_TEST)
