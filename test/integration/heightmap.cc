/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <string.h>
#include <ignition/math/Vector3.hh>

// required for HAVE_DART_BULLET define
#include <gazebo/gazebo_config.h>

#include "gazebo/common/SystemPaths.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "heights_cmp.h"
#include "gazebo/test/helper_physics_generator.hh"
#include "images_cmp.h"
#include "gazebo/test/ServerFixture.hh"

#ifdef HAVE_DART
#ifdef __MACTYPES__
#undef nil
#endif
#include "gazebo/physics/dart/DARTPhysics.hh"
#endif

using namespace gazebo;

std::mutex mutex;
unsigned char* img = NULL;

/////////////////////////////////////////////////
void OnNewCameraFrame(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  std::lock_guard<std::mutex> lock(mutex);
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

class HeightmapTest : public ServerFixture,
                      public testing::WithParamInterface<const char*>
{
  public: void PhysicsLoad(const std::string &_physicsEngine);
  public: void WhiteAlpha(const std::string &_physicsEngine);
  public: void WhiteNoAlpha(const std::string &_physicsEngine);
  public: void Volume(const std::string &_physicsEngine);
  public: void LoadDEM(const std::string &_physicsEngine);
  public: void Material(const std::string &_worldName,
      const std::string &_physicsEngine);
  // \brief Test dropping a spheres on terrain
  // \param[in] _physicsEngine the physics engine to test
  // \param[in] _dartCollision only if \e _physicsEngine is "dart", this
  //  is the collision detector to use in DART. Can be fcl, dart, bullet or ode.
  public: void TerrainCollision(const std::string &_physicsEngine,
                                const std::string &_dartCollision = "");
  // \brief Test dropping boxes on asymmetric terrain
  // \param[in] _physics the physics engine to test
  public: void TerrainCollisionAsymmetric(const std::string &_physics);

  /// \brief Test loading a heightmap that has no visuals
  public: void NoVisual();

  /// \brief Test loading a heightmap with an LOD visual plugin
  public: void LODVisualPlugin();

  /// \brief Test loading a heightmap with an LOD visual plugin that has
  /// different parameters for the Server and GUI.
  public: void LODVisualPluginServerGUI();

/// \brief Test loading a heightmap and verify cache files are created
  public: void HeightmapCache();

  public: void NotSquareImage();
  public: void InvalidSizeImage();
  // public: void Heights(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void HeightmapTest::PhysicsLoad(const std::string &_physicsEngine)
{
  Load("worlds/heightmap_test.world", true, _physicsEngine);

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::WorldPtr w = physics::get_world();
    ASSERT_NE(w, nullptr);
    physics::PhysicsEnginePtr engine = w->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run heights test\n";
    return;
  }

  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model != NULL);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::dynamic_pointer_cast<physics::HeightmapShape>(
        collision->GetShape());

  EXPECT_TRUE(shape != NULL);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  EXPECT_TRUE(shape->Pos() == ignition::math::Vector3d::Zero);
  EXPECT_TRUE(shape->Size() == ignition::math::Vector3d(129, 129, 10));

  common::Image trueImage("media/materials/textures/heightmap_bowl.png");
  common::Image testImage = shape->GetImage();

  common::SystemPaths *paths = common::SystemPaths::Instance();
  testImage.SavePNG(paths->TmpPath() + "/test_shape.png");

  EXPECT_EQ(trueImage.GetWidth(), testImage.GetWidth());
  EXPECT_EQ(trueImage.GetHeight(), testImage.GetHeight());

  // Debug output
  // Compare the true image to the image generated by the heightmap shape
  // for (uint16_t y = 0; y < testImage.GetHeight(); y += 1.0)
  // {
  //   for (uint16_t x = 0; x < testImage.GetWidth(); x += 1.0)
  //   {
  //     // EXPECT_NEAR(trueImage.GetPixel(x, y).r,
  //     //             testImage.GetPixel(x, y).r, 0.008);
  //     // if (fabs(trueImage.GetPixel(x, y).r - testImage.GetPixel(x, y).r)
  //     // > 0.008)
  //     {
  //       printf("XY[%d %d] True[%f] Test[%f]\n", x, y,
  //           trueImage.GetPixel(x, y).r, testImage.GetPixel(x, y).r);
  //     }
  //   }
  // }
}

/////////////////////////////////////////////////
void HeightmapTest::WhiteAlpha(const std::string &_physicsEngine)
{
  Load("worlds/white_alpha_heightmap.world", true, _physicsEngine);

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::WorldPtr w = physics::get_world();
    ASSERT_NE(w, nullptr);
    physics::PhysicsEnginePtr engine = w->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }

#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model != NULL);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());

  EXPECT_TRUE(shape != NULL);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  int x, y;
  for (y = 0; y < shape->VertexCount().Y(); ++y)
  {
    for (x = 0; x < shape->VertexCount().X(); ++x)
    {
      EXPECT_NEAR(shape->GetHeight(x, y), 10.0, 1e-4);
    }
  }
}

/////////////////////////////////////////////////
void HeightmapTest::WhiteNoAlpha(const std::string &_physicsEngine)
{
  Load("worlds/white_no_alpha_heightmap.world", true, _physicsEngine);

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::WorldPtr w = physics::get_world();
    ASSERT_NE(w, nullptr);
    physics::PhysicsEnginePtr engine = w->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model != NULL);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());

  EXPECT_TRUE(shape != NULL);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  int x, y;
  for (y = 0; y < shape->VertexCount().Y(); ++y)
  {
    for (x = 0; x < shape->VertexCount().X(); ++x)
    {
      EXPECT_EQ(shape->GetHeight(x, y), 10.0);
    }
  }
}

/////////////////////////////////////////////////
void HeightmapTest::NotSquareImage()
{
  common::SystemPaths::Instance()->AddGazeboPaths(
      TEST_INTEGRATION_PATH);

  this->server = new Server();
  this->server->PreLoad();
  // EXPECT_THROW(this->server->LoadFile("worlds/not_square_heightmap.world"),
  //            common::Exception);

  this->server->Fini();
  delete this->server;
}

/////////////////////////////////////////////////
void HeightmapTest::InvalidSizeImage()
{
  common::SystemPaths::Instance()->AddGazeboPaths(
      TEST_INTEGRATION_PATH);

  this->server = new Server();
  this->server->PreLoad();
  // EXPECT_THROW(this->server->LoadFile("worlds/invalid_size_heightmap.world"),
  //             common::Exception);

  this->server->Fini();
  delete this->server;
}

/////////////////////////////////////////////////
void HeightmapTest::Volume(const std::string &_physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    // SimbodyHeightmapShape unimplemented. ComputeVolume actually returns 0 as
    // an error code, which is the correct answer, but we'll skip it for now.
    gzerr << "Aborting test for "
          << _physicsEngine
          << std::endl;
    return;
  }

  Load("worlds/heightmap_test.world", true, _physicsEngine);

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::WorldPtr w = physics::get_world();
    ASSERT_NE(w, nullptr);
    physics::PhysicsEnginePtr engine = w->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model != NULL);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::dynamic_pointer_cast<physics::HeightmapShape>(
        collision->GetShape());

  EXPECT_DOUBLE_EQ(shape->ComputeVolume(), 0);
}

/////////////////////////////////////////////////
void HeightmapTest::LoadDEM(const std::string &_physicsEngine)
{
#ifdef HAVE_GDAL
  if (_physicsEngine == "bullet" || _physicsEngine == "simbody")
  {
    gzerr << "Aborting test for " << _physicsEngine <<
        ", negative elevations are not working yet." << std::endl;
    return;
  }

  Load("worlds/dem_neg.world", true, _physicsEngine);

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::WorldPtr w = physics::get_world();
    ASSERT_NE(w, nullptr);
    physics::PhysicsEnginePtr engine = w->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::ModelPtr boxModel = GetModel("box");
  ASSERT_NE(boxModel, nullptr);

  ignition::math::Pose3d boxInitPose(0, 0, -207, 0, 0, 0);
  EXPECT_EQ(boxModel->WorldPose(), boxInitPose);

  physics::ModelPtr model = GetModel("heightmap");
  ASSERT_NE(model, nullptr);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::dynamic_pointer_cast<physics::HeightmapShape>(
        collision->GetShape());

  ASSERT_NE(shape, nullptr);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  EXPECT_TRUE(shape->Pos() == ignition::math::Vector3d::Zero);

  double maxHeight = shape->GetMaxHeight();
  double minHeight = shape->GetMinHeight();
  EXPECT_GE(maxHeight, minHeight);
  EXPECT_GE(boxInitPose.Pos().Z(), minHeight);

  // step the world
  // let the box fall onto the heightmap and wait for it to rest.
  // ODE and Bullet seem to be fine with 1000 iterations, but DART needs more.
  world->Step(1200);

  ignition::math::Pose3d boxRestPose = boxModel->WorldPose();
  EXPECT_NE(boxRestPose, boxInitPose);
  EXPECT_GE(boxInitPose.Pos().Z(), minHeight);

  // step the world and verify the box is at rest
  world->Step(100);

  ignition::math::Pose3d boxNewRestPose = boxModel->WorldPose();
  EXPECT_EQ(boxNewRestPose, boxRestPose);
#else
  // prevent unused variable warning
  (void)(_physicsEngine);
#endif
}

/*
void HeightmapTest::Heights(const std::string &_physicsEngine)
{
  Load("worlds/heightmap_test.world", _physicsEngine);

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run heights test\n";
    return;
  }

  // Make sure we can get a valid pointer to the scene.
  rendering::ScenePtr scene = GetScene();
  ASSERT_TRUE(scene);

  rendering::Heightmap *heightmap = NULL;

  // Wait for the heightmap to get loaded by the scene.
  {
    int i = 0;
    while (i < 20 && (heightmap = scene->GetHeightmap()) == NULL)
    {
      common::Time::MSleep(100);
      i++;
    }

    if (i >= 20)
      gzthrow("Unable to get heightmap");
  }

  physics::ModelPtr model = GetModel("heightmap");
  EXPECT_TRUE(model);

  physics::CollisionPtr collision =
    model->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr shape =
    boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());

  EXPECT_TRUE(shape);
  EXPECT_TRUE(shape->HasType(physics::Base::HEIGHTMAP_SHAPE));

  EXPECT_TRUE(shape->GetPos() == ignition::math::Vector3d::Zero);
  EXPECT_TRUE(shape->GetSize() == ignition::math::Vector3d(129, 129, 10));

  std::vector<float> physicsTest;
  std::vector<float> renderTest;

  float x, y;

  for (y = 0; y < shape->GetSize().y && y < .3; y += 0.2)
  {
    for (x = 0; x < shape->GetSize().x && x < 1; x += 0.2)
    {
      // Compute the proper physics test point.
      int xi = rint(x);
      if (xi >= shape->GetSize().x)
        xi = shape->GetSize().x - 1.0;

      int yi = rint(y);
      if (yi >= shape->GetSize().y)
        yi = shape->GetSize().y - 1.0;

      // Compute the proper render test point.
      double xd = xi - (shape->GetSize().x) * 0.5;
      double yd = (shape->GetSize().y) * 0.5 - yi;

      // The shape->GetHeight function requires a point relative to the
      // bottom left of the heightmap image
      physicsTest.push_back(shape->GetHeight(xi, yi));

      // The render test requires a point relative to the center of the
      // heightmap.
      renderTest.push_back(heightmap->GetHeight(xd, yd));

      // Debug output
      if (fabs(physicsTest.back() - renderTest.back()) >= 0.04)
      {
        std::cout << "Render XY[" << xd << " " << yd << "] Physics XY[" << xi
          << " " << yi << "] R[" << renderTest.back() << "] P["
          << physicsTest.back() << "] D["
          << fabs(renderTest.back() - physicsTest.back()) << "]\n";
      }

      // Test to see if the physics height is equal to the render engine
      // height.
      EXPECT_NEAR(physicsTest.back(), renderTest.back(), 0.04);
    }
  }

  float diffMax, diffSum, diffAvg;
  FloatCompare(&physicsTest[0], &renderTest[0], physicsTest.size(),
      diffMax, diffSum, diffAvg);

  EXPECT_LT(diffMax, 0.04);
  EXPECT_LT(diffSum, 0.2);
  EXPECT_LT(diffAvg, 0.02);

  printf("Max[%f] Sum[%f] Avg[%f]\n", diffMax, diffSum, diffAvg);

  // This will print the heights
  // printf("static float __heights[] = {");
  // unsigned int i=0;
  // for (y = 0; y < shape->GetVertexCount().y; ++y)
  // {
  //   for (x = 0; x < shape->GetVertexCount().x; ++x)
  //   {
  //     if (y == shape->GetVertexCount().y && x == shape->GetVertexCount().x)
  //       break;

  //     if (i % 7 == 0)
  //       printf("\n");
  //     else
  //       printf(" ");
  //     printf("%f,", shape->GetHeight(x, y));
  //     i++;
  //   }
  // }
  // printf(" %f};\nstatic float *heights = __heights;\n",
  // shape->GetHeight(x,y));
}
*/

/////////////////////////////////////////////////
void HeightmapTest::Material(const std::string &_worldName,
    const std::string &_physicsEngine)
{
  // load a heightmap with red material
  Load(_worldName, false, _physicsEngine);

  if (_physicsEngine == "simbody")
  {
    // SimbodyHeightmapShape unimplemented.
    gzerr << "Aborting test for simbody" << std::endl;
    return;
  }

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::WorldPtr w = physics::get_world();
    ASSERT_NE(w, nullptr);
    physics::PhysicsEnginePtr engine = w->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  physics::ModelPtr heightmap = GetModel("heightmap");
  ASSERT_NE(heightmap, nullptr);

  // spawn camera sensor to capture an image of heightmap
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 100),
      ignition::math::Quaterniond(0, 1.57, 0));
  SpawnCamera(modelName, cameraName, testPose.Pos(),
      testPose.Rot().Euler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  int imageCount = 0;
  img = new unsigned char[width*height*3];
  event::ConnectionPtr c =
      camSensor->Camera()->ConnectNewImageFrame(
      std::bind(&::OnNewCameraFrame, &imageCount, img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  // grab some images
  int sleep = 0;
  int maxSleep = 500;
  int total_images = 10;
  while (imageCount < total_images && sleep++ < maxSleep )
    common::Time::MSleep(10);
  EXPECT_GE(imageCount, total_images);

  c.reset();

  unsigned int rSum = 0;
  unsigned int gSum = 0;
  unsigned int bSum = 0;
  for (unsigned int i = 0; i < height*width*3; i+=3)
  {
    unsigned int r = img[i];
    unsigned int g = img[i+1];
    unsigned int b = img[i+2];
    rSum += r;
    gSum += g;
    bSum += b;
  }

  // verify that red is the dominant color in the image
  EXPECT_GT(rSum, gSum);
  EXPECT_GT(rSum, bSum);

  delete [] img;
}

/////////////////////////////////////////////////
void HeightmapTest::NoVisual()
{
  // load a heightmap with no visual
  Load("worlds/heightmap_no_visual.world", false);
  physics::ModelPtr heightmap = GetModel("heightmap");
  ASSERT_NE(heightmap, nullptr);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_NE(scene, nullptr);

  // make sure scene is initialized and running
  int sleep = 0;
  int maxSleep = 30;
  while (scene->SimTime().Double() < 2.0 && sleep++ < maxSleep)
    common::Time::MSleep(100);

  // no heightmaps should exist in the scene
  EXPECT_EQ(scene->GetHeightmap(), nullptr);
}

/////////////////////////////////////////////////
void HeightmapTest::LODVisualPlugin()
{
  // load a heightmap with no visual
  Load("worlds/heightmap_lod_plugin.world", false);
  physics::ModelPtr heightmap = GetModel("heightmap");
  ASSERT_NE(heightmap, nullptr);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_NE(scene, nullptr);

  // make sure scene is initialized and running
  int sleep = 0;
  int maxSleep = 30;
  while (scene->SimTime().Double() < 2.0 && sleep++ < maxSleep)
    common::Time::MSleep(100);

  // check the heightmap lod via scene
  EXPECT_EQ(scene->HeightmapLOD(), 5u);
  // check skirt length param via scene
  EXPECT_EQ(scene->HeightmapSkirtLength(), 0.5);

  // get heightmap object and check lod params
  rendering::Heightmap *h = scene->GetHeightmap();
  EXPECT_NE(h, nullptr);
  EXPECT_EQ(h->LOD(), 5u);
  EXPECT_EQ(h->SkirtLength(), 0.5);
}

/////////////////////////////////////////////////
void HeightmapTest::LODVisualPluginServerGUI()
{
  // load a heightmap with no visual
  Load("worlds/heightmap_lod_plugin_server_gui.world", false);
  physics::ModelPtr heightmap = GetModel("heightmap");
  ASSERT_NE(heightmap, nullptr);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_NE(scene, nullptr);
  EXPECT_TRUE(scene->IsServer());

  // make sure scene is initialized and running
  int sleep = 0;
  int maxSleep = 30;
  while (scene->SimTime().Double() < 2.0 && sleep++ < maxSleep)
    common::Time::MSleep(100);

  // check the heightmap lod via scene
  EXPECT_EQ(scene->HeightmapLOD(), 0u);
  // check skirt length param via scene
  EXPECT_EQ(scene->HeightmapSkirtLength(), 0.5);

  // get heightmap object and check lod params
  rendering::Heightmap *h = scene->GetHeightmap();
  EXPECT_NE(h, nullptr);
  EXPECT_EQ(h->LOD(), 0u);
  EXPECT_EQ(h->SkirtLength(), 0.5);
}

/////////////////////////////////////////////////
void HeightmapTest::HeightmapCache()
{
  // path to heightmap cache files
  std::string heightmapName = "heightmap_bowl";
  std::string heightmapDir(common::SystemPaths::Instance()->GetLogPath()
      + "/paging");
  std::string shaPath = heightmapDir + "/" + heightmapName + "/gzterrain.SHA1";
  std::string cachePath = heightmapDir +
      "/" + heightmapName + "/gazebo_terrain_00000000.dat";

  // temporary backup files for testing if cache files exist.
  std::string shaPathBk = shaPath + ".bk";
  std::string cachePathBk = cachePath + ".bk";
  if (common::exists(shaPath))
    common::moveFile(shaPath, shaPathBk);
  if (common::exists(cachePath))
    common::moveFile(cachePath, cachePathBk);

  // there should be no cache files
  EXPECT_FALSE(common::exists(shaPath));
  EXPECT_FALSE(common::exists(cachePath));

  // load a heightmap
  Load("worlds/heightmap_test.world", false);
  physics::ModelPtr heightmap = this->GetModel("heightmap");
  ASSERT_NE(heightmap, nullptr);

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");
  ASSERT_NE(scene, nullptr);

  // make sure scene is initialized and running
  int sleep = 0;
  int maxSleep = 30;
  while (scene->SimTime().Double() < 2.0 && sleep++ < maxSleep)
    common::Time::MSleep(100);

  // make sure we have the heightmap object
  rendering::Heightmap *h = scene->GetHeightmap();
  EXPECT_NE(h, nullptr);

  // verify new sha-1 file exists
  EXPECT_TRUE(common::exists(shaPath));
  EXPECT_TRUE(common::isFile(shaPath));

  // wait for the terrain tile cache to be saved
  sleep = 0;
  while (!common::exists(cachePath) && sleep++ < maxSleep)
    common::Time::MSleep(100);

  // verify that terrain tile cache exists
  EXPECT_TRUE(common::exists(cachePath));
  EXPECT_TRUE(common::isFile(cachePath));

  // clean up by moving old files back
  if (common::exists(shaPathBk))
    common::moveFile(shaPathBk, shaPath);
  if (common::exists(cachePathBk))
    common::moveFile(cachePathBk, cachePath);
  EXPECT_FALSE(common::exists(shaPathBk));
  EXPECT_FALSE(common::exists(cachePathBk));
}

/////////////////////////////////////////////////
void HeightmapTest::TerrainCollision(const std::string &_physicsEngine,
                                     const std::string &_dartCollision)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Skipping test for bullet. See issue #2506" << std::endl;
    return;
  }

  if (_physicsEngine == "simbody")
  {
    // SimbodyHeightmapShape unimplemented.
    gzerr << "Aborting test for " << _physicsEngine << std::endl;
    return;
  }

  // world file to use
  std::string useWorld = "worlds/heightmap_test_with_sphere.world";
  if (_physicsEngine == "dart")
  {
    if (_dartCollision.empty() || _dartCollision == "bullet")
    {
      // test DART with the bullet collision detector by default or when
      // specified collision detector. Use a world file where
      // <collision_detector> is set accordingly.
      useWorld = "worlds/heightmap_test_with_sphere_dart_bullet.world";
    }
    else if (_dartCollision == "ode")
    {
      useWorld = "worlds/heightmap_test_with_sphere_dart_ode.world";
    }
    else
    {
      gzerr << "Cannot test DART with collision detector " << _dartCollision
            << ", which is not supported yet. Aborting test." << std::endl;
      return;
    }
  }
  Load(useWorld, true, _physicsEngine);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  if (_physicsEngine == "dart")
  {
#ifdef HAVE_DART
    physics::PhysicsEnginePtr engine = world->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  // step the world and verify the sphere has rolled into the valley
  world->Step(5000);

  // get shapes and min height
  physics::ModelPtr heightmap = GetModel("heightmap");
  ASSERT_NE(heightmap, nullptr);

  physics::CollisionPtr hmCollision =
    heightmap->GetLink("link")->GetCollision("collision");

  physics::HeightmapShapePtr heightmapShape =
    boost::dynamic_pointer_cast<
      physics::HeightmapShape>(hmCollision->GetShape());
  ASSERT_NE(heightmapShape, nullptr);
  double minHeight = heightmapShape->GetMinHeight();

  physics::ModelPtr sphere = GetModel("test_sphere");
  ASSERT_NE(sphere, nullptr);

  physics::CollisionPtr sphereCollision =
    sphere->GetLink("link")->GetCollision("collision");
  ASSERT_NE(sphereCollision, nullptr);

  physics::SphereShapePtr sphereShape =
    boost::dynamic_pointer_cast<
      physics::SphereShape>(sphereCollision->GetShape());
  ASSERT_NE(sphereShape, nullptr);
  double radius = sphereShape->GetRadius();

  // verify that the sphere has rolled to valley as expected
  ignition::math::Pose3d spherePose = sphere->WorldPose();

  // ensure it in fact has rolled all the way down, with some tolerance
  EXPECT_LE(spherePose.Pos().Z(), (minHeight + radius*1.01));
  // ensure it has not dropped below terrain, with some tolerance
  EXPECT_GE(spherePose.Pos().Z(), (minHeight + radius*0.99));
}

/////////////////////////////////////////////////
void HeightmapTest::TerrainCollisionAsymmetric(const std::string &_physics)
{
  if (_physics == "bullet")
  {
    gzerr << "Skipping test for bullet. See issue #2506" << std::endl;
    return;
  }

  if (_physics == "simbody")
  {
    // SimbodyHeightmapShape unimplemented.
    gzerr << "Aborting test for " << _physics << std::endl;
    return;
  }

  Load("worlds/heightmap.world", true, _physics);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  if (_physics == "dart")
  {
#ifdef HAVE_DART
    physics::PhysicsEnginePtr engine = world->Physics();
    ASSERT_NE(engine, nullptr);
    physics::DARTPhysicsPtr dartEngine
      = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
    ASSERT_NE(dartEngine, nullptr);
    std::string cd = dartEngine->CollisionDetectorInUse();
    ASSERT_FALSE(cd.empty());
    if (cd != "bullet")
    {
      // the test only works if DART uses bullet as a collision detector at the
      // moment.
      gzerr << "Aborting test for dart, see issue #909 and pull request #2956"
            << std::endl;
      return;
    }
#else
    gzerr << "Have no DART installed, skipping test for DART." << std::endl;
    return;
#endif
  }

  // each box has an initial z position of 10. meters
  physics::ModelPtr box1 = GetModel("box1");
  physics::ModelPtr box2 = GetModel("box2");
  physics::ModelPtr box3 = GetModel("box3");
  physics::ModelPtr box4 = GetModel("box4");
  ASSERT_NE(box1, nullptr);
  ASSERT_NE(box2, nullptr);
  ASSERT_NE(box3, nullptr);
  ASSERT_NE(box4, nullptr);
  EXPECT_GE(box1->WorldPose().Pos().Z(), 9.9);
  EXPECT_GE(box2->WorldPose().Pos().Z(), 9.9);
  EXPECT_GE(box3->WorldPose().Pos().Z(), 9.9);
  EXPECT_GE(box4->WorldPose().Pos().Z(), 9.9);

  // step the world and verify that only box2 falls
  world->Step(1000);

  EXPECT_GE(box1->WorldPose().Pos().Z(), 9.9);
  EXPECT_GE(box3->WorldPose().Pos().Z(), 9.9);
  EXPECT_GE(box4->WorldPose().Pos().Z(), 9.9);

  EXPECT_LT(box2->WorldPose().Pos().Z(), 5.5);
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, NotSquareImage)
{
  NotSquareImage();
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, InvalidSizeImage)
{
  InvalidSizeImage();
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, PhysicsLoad)
{
  PhysicsLoad(GetParam());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, WhiteAlpha)
{
  WhiteAlpha(GetParam());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, WhiteNoAlpha)
{
  WhiteNoAlpha(GetParam());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, Volume)
{
  Volume(GetParam());
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, LoadDEM)
{
  LoadDEM(GetParam());
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, DartCollisionDetectorSelectionBullet)
{
#ifndef HAVE_DART
  gzdbg << "Not testing DART because it is not installed." << std::endl;
  return;
#endif

#ifndef HAVE_DART_BULLET
  gzerr << "Aborting test for DART with bullet, because the "
        << "required DART extension is not installed. Please install "
        << "libdart<version>-collision-bullet-dev." << std::endl;
  return;
#endif

  // first, test if the collision_detector tag in general can be read
  // from a simple SDF, or if the SDF format does not support it.
  sdf::SDFPtr sdf(new sdf::SDF());
  // initalize the SDF descriptoins for the supported version
  sdf::init(sdf);
  // make sure we have all elements at least until <physics><dart>...
  ASSERT_NE(sdf->Root(), nullptr) << "Could not initialize SDF.";
  sdf::ElementPtr elemWorld = sdf->Root()->GetElementDescription("world");
  ASSERT_NE(elemWorld, nullptr) << "World element is expected in SDF";
  sdf::ElementPtr elemPhysics = elemWorld->GetElementDescription("physics");
  ASSERT_NE(elemPhysics, nullptr) << "Physics element is expected in SDF";
  sdf::ElementPtr elemDart = elemPhysics->GetElementDescription("dart");
  ASSERT_NE(elemDart, nullptr) << "Dart element is expected in SDF";
  // if the collision_detector element is not there, we can't run this test.
  if (!elemDart->HasElementDescription("collision_detector"))
  {
    gzerr << "SDF format version does not support <collision_detector> tag. "
          << "Skipping test." << std::endl;
    return;
  }

  // test using a world file with the <dart><collision_detector>
  // tag: verify the right collision detector has been selected.
  // Use bullet as an example (the test could also be done for ode etc.,
  // but we presume that if it works for bullet, the functionality is there
  // and it will also work for ode).
  std::string loadWorld = "worlds/heightmap_test_with_sphere_dart_bullet.world";
  Load(loadWorld, true, "dart");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::PhysicsEnginePtr engine = world->Physics();
  ASSERT_NE(engine, nullptr);
#ifdef HAVE_DART
  physics::DARTPhysicsPtr dartEngine
    = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
  ASSERT_NE(dartEngine, nullptr);
  std::string cd = dartEngine->CollisionDetectorInUse();
  EXPECT_FALSE(cd.empty());
  EXPECT_EQ(cd, "bullet");
#endif
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, DartCollisionDetectorSelectionOde)
{
#ifndef HAVE_DART
  gzdbg << "Not testing DART because it is not installed." << std::endl;
  return;
#endif

  // test using a world file with the <dart><collision_detector>
  // tag: verify the right collision detector has been selected.
  std::string loadWorld = "worlds/heightmap_test_with_sphere_dart_ode.world";
  Load(loadWorld, true, "dart");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(world, nullptr);

  physics::PhysicsEnginePtr engine = world->Physics();
  ASSERT_NE(engine, nullptr);

#ifdef HAVE_DART
  physics::DARTPhysicsPtr dartEngine
    = boost::dynamic_pointer_cast<physics::DARTPhysics>(engine);
  ASSERT_NE(dartEngine, nullptr);
  std::string cd = dartEngine->CollisionDetectorInUse();
  EXPECT_FALSE(cd.empty());

  // ODE collision detector should be disabled, as it causes conflicts.
  // Instad, the default collision detector should have been selected.
  EXPECT_NE(cd, "ode");
#endif
}


/////////////////////////////////////////////////
TEST_P(HeightmapTest, TerrainCollision)
{
  std::string param = GetParam();
  // do this test for all engines but DART, because for DART it needs to be run
  // several times for different collision detectors to use.
  // Different tests exist for this below.
  if (param == "dart")
    return;

  TerrainCollision(param);
}

/////////////////////////////////////////////////
// TerrainCollision() call for Dart using Bullet.
// We could do the same for ODE but this is disabled
// due to conflicts with the internally compiled ODE library.
// Collision engines in Dart other than Bullet don't support
// heighmaps yet, but once they do, we can do this test for
// them as well.
TEST_F(HeightmapTest, TerrainCollisionDartBullet)
{
#ifndef HAVE_DART
  gzdbg << "Not testing DART because it is not installed." << std::endl;
  return;
#endif

#ifndef HAVE_DART_BULLET
  gzerr << "Aborting test for DART with bullet, because the "
        << "required DART extension is not installed. Please install "
        << "libdart<version>-collision-bullet-dev." << std::endl;
  return;
#endif

  TerrainCollision("dart", "bullet");
}

/////////////////////////////////////////////////
TEST_P(HeightmapTest, TerrainCollisionAsymmetric)
{
  TerrainCollisionAsymmetric(GetParam());
}

/////////////////////////////////////////////////
//
// Disabled: segfaults ocassionally
// See https://github.com/osrf/gazebo/issues/521 for details

/*
TEST_P(HeightmapTest, Heights)
{
  Heights(GetParam());
}
*/

/////////////////////////////////////////////////
TEST_P(HeightmapTest, Material)
{
  Material("worlds/heightmap_material.world", GetParam());
}

// This test fails on OSX
// It uses glsl 130 which is not supported yet
#ifndef __APPLE__
/////////////////////////////////////////////////
TEST_P(HeightmapTest, MaterialShader)
{
  Material("worlds/heightmap_material_shader.world", GetParam());
}
#endif

/////////////////////////////////////////////////
TEST_F(HeightmapTest, NoVisual)
{
  NoVisual();
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, LODVisualPlugin)
{
  LODVisualPlugin();
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, LODVisualPluginServerGUI)
{
  LODVisualPluginServerGUI();
}

/////////////////////////////////////////////////
TEST_F(HeightmapTest, HeightmapCache)
{
  HeightmapCache();
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, HeightmapTest, PHYSICS_ENGINE_VALUES,);  // NOLINT

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
