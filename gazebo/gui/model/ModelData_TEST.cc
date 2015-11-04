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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelData.hh"
#include "gazebo/gui/model/ModelData_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void ModelData_TEST::Clone()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  gui::LinkData *link = new gui::LinkData();

  double mass = 1.0;
  ignition::math::Vector3d size = ignition::math::Vector3d::One;

  // create a link
  msgs::Model model;
  msgs::AddBoxLink(model, mass, size);
  link->Load(msgs::LinkToSDF(model.link(0)));
  rendering::VisualPtr linkVis(new rendering::Visual("model::box_link",
      scene->GetWorldVisual()));
  link->linkVisual = linkVis;

  // add a visual
  rendering::VisualPtr vis(
      new rendering::Visual("model::box_link::visual", linkVis));
  vis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
  link->AddVisual(vis);

  // add a collision visual
  rendering::VisualPtr collisionVis(
      new rendering::Visual("model::box_link::collision", linkVis));
  collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
  link->AddCollision(collisionVis);

  // verify clone link
  std::string cloneLinkName = "box_link_clone";
  gui::LinkData *cloneLink = link->Clone(cloneLinkName);
  QCOMPARE(cloneLink->GetName(), cloneLinkName);
  QCOMPARE(cloneLink->Scale(), ignition::math::Vector3d::One);
  QCOMPARE(cloneLink->Pose(), ignition::math::Pose3d::Zero);
  QVERIFY(cloneLink->linkVisual != NULL);
  QCOMPARE(cloneLink->linkVisual->GetName(), "model::" + cloneLinkName);
  QVERIFY(cloneLink->Scale() == ignition::math::Vector3d::One);

  // verify clone link visual
  QCOMPARE(cloneLink->visuals.size(), link->visuals.size());
  QVERIFY(cloneLink->visuals.size() == 1u);
  rendering::VisualPtr cloneVis = cloneLink->visuals.begin()->first;
  QVERIFY(cloneVis != NULL);
  QCOMPARE(cloneVis->GetName(), "model::" + cloneLinkName + "::visual");
  QCOMPARE(cloneVis->GetGeometryType(), std::string("box"));
  QCOMPARE(cloneVis->GetGeometrySize(), size);

  // verify clone link collision
  QCOMPARE(cloneLink->collisions.size(), link->collisions.size());
  QVERIFY(cloneLink->collisions.size() == 1u);
  rendering::VisualPtr cloneCol = cloneLink->collisions.begin()->first;
  QVERIFY(cloneCol != NULL);
  QCOMPARE(cloneCol->GetName(), "model::" + cloneLinkName + "::collision");
  QCOMPARE(cloneCol->GetGeometryType(), std::string("box"));
  QCOMPARE(cloneCol->GetGeometrySize(), size);

  // verify link sdf
  sdf::ElementPtr linkSDF = cloneLink->linkSDF;
  QVERIFY(linkSDF->HasElement("inertial"));
  sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
  QVERIFY(inertialElem->HasElement("inertia"));
  sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
  QVERIFY(inertialElem->HasElement("mass"));
  sdf::ElementPtr massElem = inertialElem->GetElement("mass");
  QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));

  msgs::Inertial inertialMsg = model.link(0).inertial();
  double ixx = inertialMsg.ixx();
  double iyy = inertialMsg.iyy();
  double izz = inertialMsg.izz();
  double ixy = inertialMsg.ixy();
  double ixz = inertialMsg.ixz();
  double iyz = inertialMsg.iyz();

  QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
  QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
  QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
  QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
  QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
  QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));

  // verify visual sdf
  sdf::ElementPtr visualElem = cloneVis->GetSDF();
  QCOMPARE(visualElem->Get<std::string>("name"), std::string("visual"));
  sdf::ElementPtr visualGeomElem = visualElem->GetElement("geometry");
  QVERIFY(visualGeomElem->HasElement("box"));
  sdf::ElementPtr visualGeomBoxElem = visualGeomElem->GetElement("box");
  QCOMPARE(visualGeomBoxElem->Get<ignition::math::Vector3d>("size"),
     size);

  // verify collision sdf
  sdf::ElementPtr colElem = cloneCol->GetSDF();
  QCOMPARE(colElem->Get<std::string>("name"), std::string("collision"));
  sdf::ElementPtr colGeomElem = colElem->GetElement("geometry");
  QVERIFY(colGeomElem ->HasElement("box"));
  sdf::ElementPtr colGeomBoxElem = colGeomElem->GetElement("box");
  QCOMPARE(colGeomBoxElem->Get<ignition::math::Vector3d>("size"),
     size);

  delete link;

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelData_TEST::LinkScale()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // box
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 1.0;
    ignition::math::Vector3d size = ignition::math::Vector3d::One;

    // create a link
    msgs::Model model;
    msgs::AddBoxLink(model, mass, size);
    link->Load(msgs::LinkToSDF(model.link(0)));
    rendering::VisualPtr linkVis(new rendering::Visual("box_link", scene));
    link->SetLinkVisual(linkVis);

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("box_link::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->Scale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(3.0, 2.0, 1.0);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (size.X() * size.Y() * size.Z());
      ignition::math::Vector3d newSize = dScale * size;
      double newMass = density * (newSize.X() * newSize.Y() * newSize.Z());
      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddBoxLink to help us compute the expected inertia values.
      msgs::AddBoxLink(model, newMass, newScale);
      msgs::Inertial newInertialMsg = model.link(1).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));

      scale = newScale;
      mass = newMass;
      size = newSize;
    }

    // set another scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(1.2, 3.8, 2.5);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (size.X() * size.Y() * size.Z());
      ignition::math::Vector3d newSize = dScale * size;
      double newMass = density * (newSize.X() * newSize.Y() * newSize.Z());

      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddBoxLink to help us compute the expected inertia values.
      msgs::AddBoxLink(model, newMass, newScale);
      msgs::Inertial newInertialMsg = model.link(2).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));
    }
    delete link;
  }

  // cylinder
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 1.0;
    double radius = 0.5;
    double length = 1.0;

    msgs::Model model;
    // set reasonable inertial values based on geometry
    msgs::AddCylinderLink(model, mass, radius, length);
    link->Load(msgs::LinkToSDF(model.link(0)));
    rendering::VisualPtr linkVis(new rendering::Visual("cylinder_link", scene));
    link->SetLinkVisual(linkVis);

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("cylinder_link::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->Scale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(8.5, 8.5, 1.5);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (M_PI*radius*radius*length);
      double newRadius = radius * dScale.X();
      double newLength = length * dScale.Z();
      double newMass = density * (M_PI*newRadius*newRadius*newLength);

      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddCylinderLink to help us compute the
      // expected inertia values.
      msgs::AddCylinderLink(model, newMass, newRadius, newLength);
      msgs::Inertial newInertialMsg = model.link(1).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));

      // update variables for next scale operation
      scale = newScale;
      mass = newMass;
      radius = newRadius;
      length = newLength;
    }

    // set another scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(1.2, 1.2, 3.4);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (M_PI*radius*radius*length);
      double newRadius = radius * dScale.X();
      double newLength = length * dScale.Z();
      double newMass = density * (M_PI*newRadius*newRadius*newLength);
      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddCylinderLink to help us compute
      // the expected inertia values.
      msgs::AddCylinderLink(model, newMass, newRadius, newLength);
      msgs::Inertial newInertialMsg = model.link(2).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));
    }
    delete link;
  }

  // sphere
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 1.0;
    double radius = 0.5;

    msgs::Model model;
    // set reasonable inertial values based on geometry
    msgs::AddSphereLink(model, mass, radius);
    link->Load(msgs::LinkToSDF(model.link(0)));
    rendering::VisualPtr linkVis(new rendering::Visual("sphere_link", scene));
    link->SetLinkVisual(linkVis);

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("sphere_link::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->Scale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(2.5, 2.5, 2.5);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (4/3*M_PI*radius*radius*radius);
      double newRadius = radius * dScale.X();
      double newMass = density * (4/3*M_PI*newRadius*newRadius*newRadius);
      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddSphereLink to help us compute the expected inertia values.
      msgs::AddSphereLink(model, newMass, newRadius);
      msgs::Inertial newInertialMsg = model.link(1).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));

      // update variables for next scale operation
      scale = newScale;
      mass = newMass;
      radius = newRadius;
    }

    // set another scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(3.1, 3.1, 3.1);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (4/3*M_PI*radius*radius*radius);
      double newRadius = radius * dScale.X();
      double newMass = density * (4/3*M_PI*newRadius*newRadius*newRadius);
      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddSphereLink to help us compute the expected inertia values.
      msgs::AddSphereLink(model, newMass, newRadius);
      msgs::Inertial newInertialMsg = model.link(2).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));
    }
    delete link;
  }

  // non-unit sphere
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 2.8;
    double radius = 1.2;

    msgs::Model model;
    // set reasonable inertial values based on geometry
    msgs::AddSphereLink(model, mass, radius);
    link->Load(msgs::LinkToSDF(model.link(0)));
    rendering::VisualPtr linkVis(new rendering::Visual("sphere_link2", scene));
    link->SetLinkVisual(linkVis);

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("sphere_link2::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->Scale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(2.0, 2.0, 2.0);
      collisionVis->SetScale(newScale *
          ignition::math::Vector3d(radius*2, radius*2, radius*2));
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->Scale() == newScale);

      // change in scale
      ignition::math::Vector3d dScale = newScale / scale;

      // verify new mass
      double density = mass / (4/3*M_PI*radius*radius*radius);
      double newRadius = radius * dScale.X();
      double newMass = density * (4/3*M_PI*newRadius*newRadius*newRadius);

      QVERIFY(ignition::math::equal(massElem->Get<double>(), newMass));

      // verify new inertia values
      // use msgs::AddSphereLink to help us compute the expected inertia values.
      msgs::AddSphereLink(model, newMass, newRadius);
      msgs::Inertial newInertialMsg = model.link(1).inertial();
      double newIxx = newInertialMsg.ixx();
      double newIyy = newInertialMsg.iyy();
      double newIzz = newInertialMsg.izz();
      double newIxy = newInertialMsg.ixy();
      double newIxz = newInertialMsg.ixz();
      double newIyz = newInertialMsg.iyz();

      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixx"), newIxx, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyy"), newIyy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("izz"), newIzz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixy"), newIxy, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("ixz"), newIxz, 1e-3));
      QVERIFY(ignition::math::equal(
          inertiaElem->Get<double>("iyz"), newIyz, 1e-3));
    }
    delete link;
  }

  // precision test: scale down and back up
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 1.0;
    double radius = 0.5;

    msgs::Model model;
    // set reasonable inertial values based on geometry
    msgs::AddSphereLink(model, mass, radius);
    link->Load(msgs::LinkToSDF(model.link(0)));
    rendering::VisualPtr linkVis(new rendering::Visual("sphere_link3", scene));
    link->SetLinkVisual(linkVis);

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("sphere_link3::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->Scale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));

    // scale down
    double scaleFactor = 1;
    for (unsigned int i = 10; i <= 1e6; i = i*10)
    {
      // set scale
      scaleFactor =  1.0/static_cast<double>(i);
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(scaleFactor, scaleFactor, scaleFactor);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);
      // verify new scale
      QVERIFY(link->Scale() == newScale);
    }
    // scale up
    for (unsigned int i = 1e5; i >= 1; i = i/10)
    {
      // set scale
      scaleFactor =  1.0/static_cast<double>(i);
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(scaleFactor, scaleFactor, scaleFactor);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);
      // verify new scale
      QVERIFY(link->Scale() == newScale);
    }
    // verify against original mass and inertia values
    QVERIFY(ignition::math::equal(massElem->Get<double>(), mass));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz));
    delete link;
  }

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

/////////////////////////////////////////////////
void ModelData_TEST::LinkVolume()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world");

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  for (int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  //  Verify box volume calcs
  {
    gui::LinkData *link = new gui::LinkData();

    ignition::math::Vector3d size(3, 4, 5);
    double mass = 1.0;
    double expectedVolume = 60.0;

    msgs::Model model;
    msgs::AddBoxLink(model, mass, size);

    link->Load(msgs::LinkToSDF(model.link(0)));

    rendering::VisualPtr linkVis(new rendering::Visual("box_link", scene));
    link->SetLinkVisual(linkVis);

    rendering::VisualPtr collisionVis(
        new rendering::Visual("box_link::collision", linkVis));

    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    double volume = link->ComputeVolume();
    QVERIFY(fabs(expectedVolume - volume) < 1e-3);

    delete link;
  }

  // Verify sphere volume calcs
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 1.0;
    double radius = 1.5;
    double expectedVolume = 14.137;

    msgs::Model model;
    msgs::AddSphereLink(model, mass, radius);

    link->Load(msgs::LinkToSDF(model.link(0)));

    rendering::VisualPtr linkVis(new rendering::Visual("sphere_link", scene));
    link->SetLinkVisual(linkVis);

    rendering::VisualPtr collisionVis(
        new rendering::Visual("sphere_link::collision", linkVis));

    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    double volume = link->ComputeVolume();
    QVERIFY(fabs(expectedVolume - volume) < 1e-3);

    delete link;
  }

  // Verify cylinder volume calcs
  {
    gui::LinkData *link = new gui::LinkData();

    double mass = 1.0;
    double radius = 1.5;
    double length = 5.0;
    double expectedVolume = 35.343;

    msgs::Model model;
    msgs::AddCylinderLink(model, mass, radius, length);

    link->Load(msgs::LinkToSDF(model.link(0)));

    rendering::VisualPtr linkVis(new rendering::Visual("cylinder_link", scene));
    link->SetLinkVisual(linkVis);

    rendering::VisualPtr collisionVis(
        new rendering::Visual("cylinder_link::collision", linkVis));

    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    double volume = link->ComputeVolume();
    QVERIFY(fabs(expectedVolume - volume) < 1e-3);

    delete link;
  }
}

/////////////////////////////////////////////////
void ModelData_TEST::BoxVolume()
{
  const double l = 3, w = 4, h = 5;
  const double expectedVolume = 60;

  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(l);
  size->set_y(w);
  size->set_z(h);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_BOX);
  geo->set_allocated_box(box);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  double volume = gui::LinkData::ComputeVolume(*col);
  QVERIFY(fabs(expectedVolume - volume) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::CylinderVolume()
{
  const double r = 1.5, l = 5.0;
  const double expectedVolume = 35.343;

  msgs::CylinderGeom *cyl = new msgs::CylinderGeom();
  cyl->set_radius(r);
  cyl->set_length(l);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_CYLINDER);
  geo->set_allocated_cylinder(cyl);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  double volume = gui::LinkData::ComputeVolume(*col);
  QVERIFY(fabs(expectedVolume - volume) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::SphereVolume()
{
  const double r = 1.5;
  const double expectedVolume = 14.137;

  msgs::SphereGeom *sphere = new msgs::SphereGeom();
  sphere->set_radius(r);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_SPHERE);
  geo->set_allocated_sphere(sphere);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  double volume = gui::LinkData::ComputeVolume(*col);
  QVERIFY(fabs(expectedVolume - volume) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::MeshVolume()
{
  const double l = 3, w = 4, h = 5;
  const double expectedVolume = 60;

  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(l);
  size->set_y(w);
  size->set_z(h);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_MESH);
  geo->set_allocated_box(box);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  double volume = gui::LinkData::ComputeVolume(*col);
  QVERIFY(fabs(expectedVolume - volume) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::PolylineVolume()
{
  const double l = 3, w = 4, h = 5;
  const double expectedVolume = 60;

  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(l);
  size->set_y(w);
  size->set_z(h);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_POLYLINE);
  geo->set_allocated_box(box);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  double volume = gui::LinkData::ComputeVolume(*col);
  QVERIFY(fabs(expectedVolume - volume) < 1e-3);

  delete col;
}

void ModelData_TEST::SphereMomentOfInertia()
{
  const double r = 1.5;
  const double m = 1.0;
  const double expectedI = 0.9;

  msgs::SphereGeom *sphere = new msgs::SphereGeom();
  sphere->set_radius(r);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_SPHERE);
  geo->set_allocated_sphere(sphere);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  ignition::math::Vector3d I = gui::LinkData::ComputeMomentOfInertia(*col, m);
  QVERIFY(fabs(expectedI - I.X()) < 1e-3);
  QVERIFY(fabs(expectedI - I.Y()) < 1e-3);
  QVERIFY(fabs(expectedI - I.Z()) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::CylinderMomentOfInertia()
{
  const double r = 1.5, l = 5.0, m = 1.0;
  const double expectedIx = 2.64583;
  const double expectedIy = 2.64583;
  const double expectedIz = 1.125;

  msgs::CylinderGeom *cyl = new msgs::CylinderGeom();
  cyl->set_radius(r);
  cyl->set_length(l);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_CYLINDER);
  geo->set_allocated_cylinder(cyl);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  ignition::math::Vector3d I = gui::LinkData::ComputeMomentOfInertia(*col, m);

  QVERIFY(fabs(expectedIx - I.X()) < 1e-3);
  QVERIFY(fabs(expectedIy - I.Y()) < 1e-3);
  QVERIFY(fabs(expectedIz - I.Z()) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::BoxMomentOfInertia()
{
  const double l = 3, w = 4, h = 5, m = 1.0;
  const double expectedIx = 3.41667;
  const double expectedIy = 2.83333;
  const double expectedIz = 2.08333;

  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(l);
  size->set_y(w);
  size->set_z(h);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_BOX);
  geo->set_allocated_box(box);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  ignition::math::Vector3d I = gui::LinkData::ComputeMomentOfInertia(*col, m);

  QVERIFY(fabs(expectedIx - I.X()) < 1e-3);
  QVERIFY(fabs(expectedIy - I.Y()) < 1e-3);
  QVERIFY(fabs(expectedIz - I.Z()) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::MeshMomentOfInertia()
{
  const double l = 3, w = 4, h = 5, m = 1.0;
  const double expectedIx = 3.41667;
  const double expectedIy = 2.83333;
  const double expectedIz = 2.08333;

  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(l);
  size->set_y(w);
  size->set_z(h);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_MESH);
  geo->set_allocated_box(box);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  ignition::math::Vector3d I = gui::LinkData::ComputeMomentOfInertia(*col, m);

  QVERIFY(fabs(expectedIx - I.X()) < 1e-3);
  QVERIFY(fabs(expectedIy - I.Y()) < 1e-3);
  QVERIFY(fabs(expectedIz - I.Z()) < 1e-3);

  delete col;
}

/////////////////////////////////////////////////
void ModelData_TEST::PolylineMomentOfInertia()
{
  const double l = 3, w = 4, h = 5, m = 1.0;
  const double expectedIx = 3.41667;
  const double expectedIy = 2.83333;
  const double expectedIz = 2.08333;

  msgs::Vector3d *size = new msgs::Vector3d();
  size->set_x(l);
  size->set_y(w);
  size->set_z(h);

  msgs::BoxGeom *box = new msgs::BoxGeom();
  box->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_POLYLINE);
  geo->set_allocated_box(box);

  msgs::Collision *col = new msgs::Collision();
  col->set_allocated_geometry(geo);

  ignition::math::Vector3d I = gui::LinkData::ComputeMomentOfInertia(*col, m);

  QVERIFY(fabs(expectedIx - I.X()) < 1e-3);
  QVERIFY(fabs(expectedIy - I.Y()) < 1e-3);
  QVERIFY(fabs(expectedIz - I.Z()) < 1e-3);

  delete col;
}

// Generate a main function for the test
QTEST_MAIN(ModelData_TEST)
