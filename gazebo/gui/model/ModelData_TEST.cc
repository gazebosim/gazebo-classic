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
    math::Vector3 size = math::Vector3::One;

    // create a link
    msgs::Model model;
    msgs::AddBoxLink(model, mass, size);
    link->Load(msgs::LinkToSDF(model.link(0)));
    rendering::VisualPtr linkVis(new rendering::Visual("link", scene));
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    math::Vector3 scale = math::Vector3::One;
    QVERIFY(link->GetScale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    qFuzzyCompare(massElem->Get<double>(), mass);

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();
    qFuzzyCompare(inertiaElem->Get<double>("ixx"), ixx);
    qFuzzyCompare(inertiaElem->Get<double>("iyy"), iyy);
    qFuzzyCompare(inertiaElem->Get<double>("izz"), izz);
    qFuzzyCompare(inertiaElem->Get<double>("ixy"), ixy);
    qFuzzyCompare(inertiaElem->Get<double>("ixz"), ixz);
    qFuzzyCompare(inertiaElem->Get<double>("iyz"), iyz);

    // set new scale and verify inertial values
    {
      // set scale
      math::Vector3 newScale = math::Vector3(3.0, 2.0, 1.0);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

      // change in scale
      math::Vector3 dScale = newScale / scale;

      // verify new mass
      double density = mass / (size.x * size.y * size.z);
      math::Vector3 newSize = dScale * size;
      double newMass = density * (newSize.x * newSize.y * newSize.z) * mass;
      qFuzzyCompare(massElem->Get<double>(), newMass);

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
      qFuzzyCompare(inertiaElem->Get<double>("ixx"), newIxx);
      qFuzzyCompare(inertiaElem->Get<double>("iyy"), newIyy);
      qFuzzyCompare(inertiaElem->Get<double>("izz"), newIzz);
      qFuzzyCompare(inertiaElem->Get<double>("ixy"), newIxy);
      qFuzzyCompare(inertiaElem->Get<double>("ixz"), newIxz);
      qFuzzyCompare(inertiaElem->Get<double>("iyz"), newIyz);

      scale = newScale;
      mass = newMass;
      size = newSize;
    }

    // set another scale and verify inertial values
    {
      // set scale
      math::Vector3 newScale = math::Vector3(1.2, 3.8, 2.5);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

      // change in scale
      math::Vector3 dScale = newScale / scale;

      // verify new mass
      double density = mass / (size.x * size.y * size.z);
      math::Vector3 newSize = dScale * size;
      double newMass = density * (newSize.x * newSize.y * newSize.z) * mass;
      qFuzzyCompare(massElem->Get<double>(), newMass);

      std::cerr << " == box mass " << mass << ", " << newMass << ", " << massElem->Get<double>() << std::endl;
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
      qFuzzyCompare(inertiaElem->Get<double>("ixx"), newIxx);
      qFuzzyCompare(inertiaElem->Get<double>("iyy"), newIyy);
      qFuzzyCompare(inertiaElem->Get<double>("izz"), newIzz);
      qFuzzyCompare(inertiaElem->Get<double>("ixy"), newIxy);
      qFuzzyCompare(inertiaElem->Get<double>("ixz"), newIxz);
      qFuzzyCompare(inertiaElem->Get<double>("iyz"), newIyz);
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
    rendering::VisualPtr linkVis(new rendering::Visual("link", scene));
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    math::Vector3 scale = math::Vector3::One;
    QVERIFY(link->GetScale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    qFuzzyCompare(massElem->Get<double>(), mass);

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();
    qFuzzyCompare(inertiaElem->Get<double>("ixx"), ixx);
    qFuzzyCompare(inertiaElem->Get<double>("iyy"), iyy);
    qFuzzyCompare(inertiaElem->Get<double>("izz"), izz);
    qFuzzyCompare(inertiaElem->Get<double>("ixy"), ixy);
    qFuzzyCompare(inertiaElem->Get<double>("ixz"), ixz);
    qFuzzyCompare(inertiaElem->Get<double>("iyz"), iyz);

    // set new scale and verify inertial values
    {
      // set scale
      math::Vector3 newScale = math::Vector3(8.5, 8.5, 1.5);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

      // change in scale
      math::Vector3 dScale = newScale / scale;

      // verify new mass
      double density = mass / (M_PI*radius*radius*length);
      double newRadius = radius * dScale.x;
      double newLength = length * dScale.z;
      double newMass = density * (M_PI*newRadius*newRadius*newLength);

      std::cerr << " cylinder mass radius length" << mass << ", " << radius << ", " << length << std::endl;
      std::cerr << " cylinder density " << density << std::endl;
      std::cerr << " cylinder scale " << scale << std::endl;
      std::cerr << " cylinder mass " << mass << ", " << newMass << ", " << massElem->Get<double>() << std::endl;

      qFuzzyCompare(massElem->Get<double>(), newMass);

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
      qFuzzyCompare(inertiaElem->Get<double>("ixx"), newIxx);
      qFuzzyCompare(inertiaElem->Get<double>("iyy"), newIyy);
      qFuzzyCompare(inertiaElem->Get<double>("izz"), newIzz);
      qFuzzyCompare(inertiaElem->Get<double>("ixy"), newIxy);
      qFuzzyCompare(inertiaElem->Get<double>("ixz"), newIxz);
      qFuzzyCompare(inertiaElem->Get<double>("iyz"), newIyz);

      // update variables for next scale operation
      scale = newScale;
      mass = newMass;
      radius = newRadius;
      length = newLength;
    }

    // set another scale and verify inertial values
    {
      // set scale
      math::Vector3 newScale = math::Vector3(1.2, 1.2, 3.4);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

      // change in scale
      math::Vector3 dScale = newScale / scale;

      // verify new mass
      double density = mass / (M_PI*radius*radius*length);
      double newRadius = radius * dScale.x;
      double newLength = length * dScale.z;
      double newMass = density * (M_PI*newRadius*newRadius*newLength);
      qFuzzyCompare(massElem->Get<double>(), newMass);

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
      qFuzzyCompare(inertiaElem->Get<double>("ixx"), newIxx);
      qFuzzyCompare(inertiaElem->Get<double>("iyy"), newIyy);
      qFuzzyCompare(inertiaElem->Get<double>("izz"), newIzz);
      qFuzzyCompare(inertiaElem->Get<double>("ixy"), newIxy);
      qFuzzyCompare(inertiaElem->Get<double>("ixz"), newIxz);
      qFuzzyCompare(inertiaElem->Get<double>("iyz"), newIyz);
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
    rendering::VisualPtr linkVis(new rendering::Visual("link", scene));
    link->linkVisual = linkVis;

    // add a collision visual
    rendering::VisualPtr collisionVis(
        new rendering::Visual("collision", linkVis));
    collisionVis->Load(msgs::VisualToSDF(model.link(0).visual(0)));
    link->AddCollision(collisionVis);

    // verify scale
    math::Vector3 scale = math::Vector3::One;
    QVERIFY(link->GetScale() == scale);

    sdf::ElementPtr linkSDF = link->linkSDF;
    QVERIFY(linkSDF->HasElement("inertial"));
    sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
    QVERIFY(inertialElem->HasElement("inertia"));
    sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
    QVERIFY(inertialElem->HasElement("mass"));
    sdf::ElementPtr massElem = inertialElem->GetElement("mass");

    // verify mass
    qFuzzyCompare(massElem->Get<double>(), mass);

    // verify inertia values
    msgs::Inertial inertialMsg = model.link(0).inertial();
    double ixx = inertialMsg.ixx();
    double iyy = inertialMsg.iyy();
    double izz = inertialMsg.izz();
    double ixy = inertialMsg.ixy();
    double ixz = inertialMsg.ixz();
    double iyz = inertialMsg.iyz();
    qFuzzyCompare(inertiaElem->Get<double>("ixx"), ixx);
    qFuzzyCompare(inertiaElem->Get<double>("iyy"), iyy);
    qFuzzyCompare(inertiaElem->Get<double>("izz"), izz);
    qFuzzyCompare(inertiaElem->Get<double>("ixy"), ixy);
    qFuzzyCompare(inertiaElem->Get<double>("ixz"), ixz);
    qFuzzyCompare(inertiaElem->Get<double>("iyz"), iyz);

    // set new scale and verify inertial values
    {
      // set scale
      math::Vector3 newScale = math::Vector3(2.5, 2.5, 2.5);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

      // change in scale
      math::Vector3 dScale = newScale / scale;

      // verify new mass
      double density = mass / (4/3*M_PI*radius*radius*radius);
      double newRadius = radius * dScale.x;
      double newMass = density * (4/3*M_PI*newRadius*newRadius*newRadius);
      qFuzzyCompare(massElem->Get<double>(), newMass);

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
      qFuzzyCompare(inertiaElem->Get<double>("ixx"), newIxx);
      qFuzzyCompare(inertiaElem->Get<double>("iyy"), newIyy);
      qFuzzyCompare(inertiaElem->Get<double>("izz"), newIzz);
      qFuzzyCompare(inertiaElem->Get<double>("ixy"), newIxy);
      qFuzzyCompare(inertiaElem->Get<double>("ixz"), newIxz);
      qFuzzyCompare(inertiaElem->Get<double>("iyz"), newIyz);

      // update variables for next scale operation
      scale = newScale;
      mass = newMass;
      radius = newRadius;
    }

    // set another scale and verify inertial values
    {
      // set scale
      math::Vector3 newScale = math::Vector3(2.5, 2.5, 2.5);
      link->SetScale(newScale);

      // verify new scale
      QVERIFY(link->GetScale() == newScale);

      // change in scale
      math::Vector3 dScale = newScale / scale;

      // verify new mass
      double density = mass / (4/3*M_PI*radius*radius*radius);
      double newRadius = radius * dScale.x;
      double newMass = density * (4/3*M_PI*newRadius*newRadius*newRadius);
      qFuzzyCompare(massElem->Get<double>(), newMass);

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
      qFuzzyCompare(inertiaElem->Get<double>("ixx"), newIxx);
      qFuzzyCompare(inertiaElem->Get<double>("iyy"), newIyy);
      qFuzzyCompare(inertiaElem->Get<double>("izz"), newIzz);
      qFuzzyCompare(inertiaElem->Get<double>("ixy"), newIxy);
      qFuzzyCompare(inertiaElem->Get<double>("ixz"), newIxz);
      qFuzzyCompare(inertiaElem->Get<double>("iyz"), newIyz);
    }
    delete link;
  }
}


// Generate a main function for the test
QTEST_MAIN(ModelData_TEST)
