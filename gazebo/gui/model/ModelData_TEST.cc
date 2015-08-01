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
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("box_link::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->GetScale() == scale);

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

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz, 1e-3));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(3.0, 2.0, 1.0);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

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
      QVERIFY(link->GetScale() == newScale);

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
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("cylinder_link::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->GetScale() == scale);

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

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz, 1e-3));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(8.5, 8.5, 1.5);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

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
      QVERIFY(link->GetScale() == newScale);

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
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("sphere_link::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->GetScale() == scale);

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

    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixx"), ixx, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz, 1e-3));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(2.5, 2.5, 2.5);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

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
      QVERIFY(link->GetScale() == newScale);

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
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("sphere_link2::collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    ignition::math::Vector3d scale = ignition::math::Vector3d::One;
    QVERIFY(link->GetScale() == scale);

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
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyy"), iyy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("izz"), izz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixy"), ixy, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("ixz"), ixz, 1e-3));
    QVERIFY(ignition::math::equal(inertiaElem->Get<double>("iyz"), iyz, 1e-3));

    // set new scale and verify inertial values
    {
      // set scale
      ignition::math::Vector3d newScale =
          ignition::math::Vector3d(2.0, 2.0, 2.0);
      collisionVis->SetScale(newScale);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

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
    delete link;
  }

  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelData_TEST)
