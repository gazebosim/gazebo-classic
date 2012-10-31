/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "ServerFixture.hh"
#include "common/common.hh"

using namespace gazebo;
class CommonTest : public ServerFixture
{
};

std::string asciiSTLBox =
"solid MYSOLID\n\
  facet normal  0.0   0.0  -1.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    1.0   1.0   0.0\n\
      vertex    1.0   0.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   0.0  -1.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    0.0   1.0   0.0\n\
      vertex    1.0   1.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal -1.0   0.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    0.0   1.0   1.0\n\
      vertex    0.0   1.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal -1.0   0.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    0.0   0.0   1.0\n\
      vertex    0.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   1.0   0.0\n\
    outer loop\n\
      vertex    0.0   1.0   0.0\n\
      vertex    1.0   1.0   1.0\n\
      vertex    1.0   1.0   0.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   1.0   0.0\n\
    outer loop\n\
      vertex    0.0   1.0   0.0\n\
      vertex    0.0   1.0   1.0\n\
      vertex    1.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  1.0   0.0   0.0\n\
    outer loop\n\
      vertex    1.0   0.0   0.0\n\
      vertex    1.0   1.0   0.0\n\
      vertex    1.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  1.0   0.0   0.0\n\
    outer loop\n\
      vertex    1.0   0.0   0.0\n\
      vertex    1.0   1.0   1.0\n\
      vertex    1.0   0.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0  -1.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    1.0   0.0   0.0\n\
      vertex    1.0   0.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0  -1.0   0.0\n\
    outer loop\n\
      vertex    0.0   0.0   0.0\n\
      vertex    1.0   0.0   1.0\n\
      vertex    0.0   0.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   0.0   1.0\n\
    outer loop\n\
      vertex    0.0   0.0   1.0\n\
      vertex    1.0   0.0   1.0\n\
      vertex    1.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
  facet normal  0.0   0.0   1.0\n\
    outer loop\n\
      vertex    0.0   0.0   1.0\n\
      vertex    1.0   1.0   1.0\n\
      vertex    0.0   1.0   1.0\n\
    endloop\n\
  endfacet\n\
endsolid MYSOLID";

TEST_F(CommonTest, Image)
{
  common::Image img;
  EXPECT_EQ(-1, img.Load("/file/shouldn/never/exist.png"));
  EXPECT_EQ(0, img.Load("file://media/materials/textures/wood.jpg"));
  EXPECT_EQ(static_cast<unsigned int>(496), img.GetWidth());
  EXPECT_EQ(static_cast<unsigned int>(329), img.GetHeight());
  EXPECT_EQ(static_cast<unsigned int>(24), img.GetBPP());
  EXPECT_TRUE(img.GetPixel(10, 10) ==
      common::Color(0.133333, 0.376471, 0.654902, 1));
  EXPECT_TRUE(img.GetAvgColor() ==
      common::Color(0.260456, 0.506047, 0.758062, 1));
  EXPECT_TRUE(img.GetMaxColor() ==
      common::Color(0.807843, 0.909804, 0.964706, 1));
  EXPECT_TRUE(img.Valid());
  EXPECT_TRUE(img.GetFilename().find("materials/textures/wood.jpg") !=
      std::string::npos);

  unsigned char *data = NULL;
  unsigned int size = 0;
  img.GetData(&data, size);
  EXPECT_EQ(static_cast<unsigned int>(489552), size);

  img.SetFromData(data, img.GetWidth(), img.GetHeight(),
                  common::Image::RGB_INT8);
}

TEST_F(CommonTest, Paths)
{
  std::string gazeboResourcePathBackup = "GAZEBO_RESOURCE_PATH=";
  std::string ogreResourcePathBackup = "OGRE_RESOURCE_PATH=";
  std::string pluginPathBackup = "GAZEBO_PLUGIN_PATH=";

  gazeboResourcePathBackup += getenv("GAZEBO_RESOURCE_PATH");
  ogreResourcePathBackup += getenv("GAZEBO_RESOURCE_PATH");
  pluginPathBackup += getenv("GAZEBO_PLUGIN_PATH");

  putenv(const_cast<char*>("GAZEBO_LOG_PATH="));
  common::SystemPaths *paths = common::SystemPaths::Instance();

  paths->ClearGazeboPaths();
  paths->ClearOgrePaths();
  paths->ClearPluginPaths();

  EXPECT_FALSE(paths->GetLogPath().empty());

  putenv(const_cast<char*>("GAZEBO_RESOURCE_PATH=/tmp/resource:/test/me/now"));
  const std::list<std::string> pathList1 = paths->GetGazeboPaths();
  EXPECT_EQ(static_cast<unsigned int>(2), pathList1.size());
  EXPECT_STREQ("/tmp/resource", pathList1.front().c_str());
  EXPECT_STREQ("/test/me/now", pathList1.back().c_str());

  putenv(const_cast<char*>("OGRE_RESOURCE_PATH=/tmp/ogre:/test/ogre/now"));
  const std::list<std::string> pathList2 = paths->GetOgrePaths();
  EXPECT_EQ(static_cast<unsigned int>(2), pathList2.size());
  EXPECT_STREQ("/tmp/ogre", pathList2.front().c_str());
  EXPECT_STREQ("/test/ogre/now", pathList2.back().c_str());

  putenv(const_cast<char*>("GAZEBO_PLUGIN_PATH=/tmp/plugin:/test/plugin/now"));
  const std::list<std::string> pathList3 = paths->GetPluginPaths();
  EXPECT_EQ(static_cast<unsigned int>(2), pathList3.size());
  EXPECT_STREQ("/tmp/plugin", pathList3.front().c_str());
  EXPECT_STREQ("/test/plugin/now", pathList3.back().c_str());

  EXPECT_STREQ("/worlds", paths->GetWorldPathExtension().c_str());

  paths->AddGazeboPaths("/gazebo/path:/other/gazebo");
  EXPECT_EQ(static_cast<unsigned int>(4), paths->GetGazeboPaths().size());
  EXPECT_STREQ("/other/gazebo", paths->GetGazeboPaths().back().c_str());

  paths->AddPluginPaths("/plugin/path:/other/plugin");
  EXPECT_EQ(static_cast<unsigned int>(4), paths->GetGazeboPaths().size());
  EXPECT_STREQ("/other/plugin", paths->GetPluginPaths().back().c_str());

  paths->AddOgrePaths("/ogre/path:/other/ogre");
  EXPECT_EQ(static_cast<unsigned int>(4), paths->GetOgrePaths().size());
  EXPECT_STREQ("/other/ogre", paths->GetOgrePaths().back().c_str());

  paths->ClearGazeboPaths();
  paths->ClearOgrePaths();
  paths->ClearPluginPaths();

  EXPECT_EQ(static_cast<unsigned int>(2), paths->GetGazeboPaths().size());
  EXPECT_EQ(static_cast<unsigned int>(2), paths->GetOgrePaths().size());
  EXPECT_EQ(static_cast<unsigned int>(2), paths->GetPluginPaths().size());

  putenv(const_cast<char*>("GAZEBO_RESOURCE_PATH="));
  paths->ClearGazeboPaths();
  EXPECT_EQ(static_cast<unsigned int>(0), paths->GetGazeboPaths().size());

  putenv(const_cast<char*>("OGRE_RESOURCE_PATH="));
  paths->ClearOgrePaths();
  EXPECT_EQ(static_cast<unsigned int>(0), paths->GetOgrePaths().size());

  putenv(const_cast<char*>("GAZEBO_PLUGIN_PATH="));
  paths->ClearPluginPaths();
  EXPECT_EQ(static_cast<unsigned int>(0), paths->GetPluginPaths().size());

  std::cout << "GAZEBO_RESOURCE_BACKUP[" << gazeboResourcePathBackup << "]\n";
  std::cout << "OGRE_RESOURCE_BACKUP[" << ogreResourcePathBackup << "]\n";
  std::cout << "GAZEBO_PLUGIN_BACKUP[" << ogreResourcePathBackup << "]\n";

  putenv(const_cast<char*>(gazeboResourcePathBackup.c_str()));
  putenv(const_cast<char*>(ogreResourcePathBackup.c_str()));
  putenv(const_cast<char*>(pluginPathBackup.c_str()));
}

TEST_F(CommonTest, PoseAnimation)
{
  {
    common::PoseAnimation anim("test", 1.0, true);
    anim.SetTime(-0.5);
    EXPECT_EQ(0.5, anim.GetTime());
  }

  {
    common::PoseAnimation anim("test", 1.0, false);
    anim.SetTime(-0.5);
    EXPECT_EQ(0.0, anim.GetTime());

    anim.SetTime(1.5);
    EXPECT_EQ(1.0, anim.GetTime());
  }


  common::PoseAnimation anim("pose_test", 5.0, false);
  common::PoseKeyFrame *key = anim.CreateKeyFrame(0.0);

  EXPECT_EQ(5.0, anim.GetLength());
  anim.SetLength(10.0);
  EXPECT_EQ(10.0, anim.GetLength());

  key->SetTranslation(math::Vector3(0, 0, 0));
  EXPECT_TRUE(key->GetTranslation() == math::Vector3(0, 0, 0));

  key->SetRotation(math::Quaternion(0, 0, 0));
  EXPECT_TRUE(key->GetRotation() == math::Quaternion(0, 0, 0));

  key = anim.CreateKeyFrame(10.0);
  key->SetTranslation(math::Vector3(10, 20, 30));
  EXPECT_TRUE(key->GetTranslation() == math::Vector3(10, 20, 30));

  key->SetRotation(math::Quaternion(0.1, 0.2, 0.3));
  EXPECT_TRUE(key->GetRotation() == math::Quaternion(0.1, 0.2, 0.3));

  anim.AddTime(5.0);
  EXPECT_EQ(5.0, anim.GetTime());

  anim.SetTime(4.0);
  EXPECT_EQ(4.0, anim.GetTime());

  common::PoseKeyFrame interpolatedKey(-1.0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  EXPECT_TRUE(interpolatedKey.GetTranslation() ==
      math::Vector3(3.76, 7.52, 11.28));
  EXPECT_TRUE(interpolatedKey.GetRotation() ==
      math::Quaternion(0.0302776, 0.0785971, 0.109824));
}

TEST_F(CommonTest, NumericAnimation)
{
  common::NumericAnimation anim("numeric_test", 10, false);
  common::NumericKeyFrame *key = anim.CreateKeyFrame(0.0);

  key->SetValue(0.0);
  EXPECT_EQ(0.0, key->GetValue());

  key = anim.CreateKeyFrame(10.0);
  key->SetValue(30);
  EXPECT_EQ(30, key->GetValue());

  anim.AddTime(5.0);
  EXPECT_EQ(5.0, anim.GetTime());

  anim.SetTime(4.0);
  EXPECT_EQ(4.0, anim.GetTime());

  common::NumericKeyFrame interpolatedKey(0);
  anim.GetInterpolatedKeyFrame(interpolatedKey);
  EXPECT_EQ(12, interpolatedKey.GetValue());
}

TEST_F(CommonTest, Color)
{
  common::Color clr(.1, .2, .3, 1.0);
  EXPECT_EQ(0.1f, clr.r);
  EXPECT_EQ(0.2f, clr.g);
  EXPECT_EQ(0.3f, clr.b);
  EXPECT_EQ(1.0f, clr.a);

  clr.Reset();
  EXPECT_EQ(0.0f, clr.r);
  EXPECT_EQ(0.0f, clr.g);
  EXPECT_EQ(0.0f, clr.b);
  EXPECT_EQ(0.0f, clr.a);

  clr.SetFromHSV(0, 0.5, 1.0);
  EXPECT_EQ(1.0f, clr.r);
  EXPECT_EQ(0.5f, clr.g);
  EXPECT_EQ(0.5f, clr.b);
  EXPECT_EQ(0.0f, clr.a);

  EXPECT_TRUE(clr.GetAsHSV() == math::Vector3(6, 0.5, 1));

  clr.SetFromHSV(60, 0.0, 1.0);
  EXPECT_EQ(1.0f, clr.r);
  EXPECT_EQ(1.0f, clr.g);
  EXPECT_EQ(1.0f, clr.b);
  EXPECT_EQ(0.0f, clr.a);

  clr.SetFromHSV(120, 0.5, 1.0);
  EXPECT_EQ(0.5f, clr.r);
  EXPECT_EQ(1.0f, clr.g);
  EXPECT_EQ(0.5f, clr.b);
  EXPECT_EQ(0.0f, clr.a);

  clr.SetFromHSV(180, 0.5, 1.0);
  EXPECT_EQ(0.5f, clr.r);
  EXPECT_EQ(1.0f, clr.g);
  EXPECT_EQ(1.0f, clr.b);
  EXPECT_EQ(0.0f, clr.a);

  clr.SetFromHSV(240, 0.5, 1.0);
  EXPECT_EQ(0.5f, clr.r);
  EXPECT_EQ(0.5f, clr.g);
  EXPECT_EQ(1.0f, clr.b);
  EXPECT_EQ(0.0f, clr.a);

  clr.SetFromHSV(300, 0.5, 1.0);
  EXPECT_EQ(1.0f, clr[0]);
  EXPECT_EQ(0.5f, clr[1]);
  EXPECT_EQ(1.0f, clr[2]);
  EXPECT_EQ(0.0f, clr[3]);
  EXPECT_EQ(0.0f, clr[4]);

  clr.r = 0.1;
  clr.g = 0.2;
  clr.b = 0.3;
  clr.a = 0.4;
  EXPECT_EQ(0.1f, clr[0]);
  EXPECT_EQ(0.2f, clr[1]);
  EXPECT_EQ(0.3f, clr[2]);
  EXPECT_EQ(0.4f, clr[3]);

  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr = clr + 0.2;
  EXPECT_TRUE(clr == common::Color(0.3, 0.4, 0.5, 0.6));

  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr += common::Color(0.2, 0.2, 0.2, 0.2);
  EXPECT_TRUE(clr == common::Color(0.3, 0.4, 0.5, 0.6));


  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr = clr - 0.1;
  EXPECT_TRUE(clr == common::Color(0.0, 0.1, 0.2, 0.3));

  clr.Set(0.1, 0.2, 0.3, 0.4);
  clr -= common::Color(0.1, 0.1, 0.1, 0.1);
  EXPECT_TRUE(clr == common::Color(0.0, 0.1, 0.2, 0.3));


  clr.Set(1, 1, 1, 1.);
  clr = clr / 1.6;
  EXPECT_TRUE(clr == common::Color(0.625, 0.625, 0.625, 0.625));

  clr.Set(1, 1, 1, 1);
  clr /= common::Color(1, 1, 1, 1);
  EXPECT_TRUE(clr == common::Color(1, 1, 1, 1));


  clr.Set(.1, .2, .3, .4);
  clr = clr * .1;
  EXPECT_TRUE(clr == common::Color(0.01, 0.02, 0.03, 0.04));

  clr.Set(.1, .2, .3, .4);
  clr *= common::Color(0.1, 0.1, 0.1, 0.1);
  EXPECT_TRUE(clr == common::Color(0.01, 0.02, 0.03, 0.04));


  clr.SetFromYUV(0.5, 0.2, 0.8);
  EXPECT_TRUE(math::equal(0.00553f, clr.r, 1e-3f));
  EXPECT_TRUE(math::equal(0.0f, clr.g));
  EXPECT_TRUE(math::equal(0.9064f, clr.b, 1e-3f));
  EXPECT_TRUE(math::equal(0.04f, clr.a));

  EXPECT_TRUE(clr.GetAsYUV() == math::Vector3(0.104985, 0.95227, 0.429305));

  clr = common::Color(1.0, 0.0, 0.5, 1.0) + common::Color(0.1, 0.3, 0.4, 1.0);
  EXPECT_TRUE(math::equal(0.00431373f, clr.r));
  EXPECT_TRUE(math::equal(0.3f, clr.g));
  EXPECT_TRUE(math::equal(0.9f, clr.b));
  EXPECT_TRUE(math::equal(2.0f, clr.a));

  clr = common::Color(1.0, 0.0, 0.5, 1.0) - common::Color(0.1, 0.3, 0.4, 1.0);
  EXPECT_TRUE(math::equal(0.9f, clr.r));
  EXPECT_TRUE(math::equal(0.0f, clr.g));
  EXPECT_TRUE(math::equal(0.1f, clr.b));
  EXPECT_TRUE(math::equal(0.0f, clr.a));

  clr = common::Color(0.5, 0.2, 0.4, 0.6) / 2.0;
  EXPECT_TRUE(math::equal(0.25f, clr.r));
  EXPECT_TRUE(math::equal(0.1f, clr.g));
  EXPECT_TRUE(math::equal(0.2f, clr.b));
  EXPECT_TRUE(math::equal(0.3f, clr.a));
}

TEST_F(CommonTest, Time)
{
  common::Timer timer;
  timer.Start();
  common::Time::MSleep(100);
  EXPECT_TRUE(timer.GetElapsed() > common::Time(0, 100000000));

  struct timeval tv;
  gettimeofday(&tv, NULL);
  common::Time time(tv);
  EXPECT_EQ(time.sec, tv.tv_sec);
  EXPECT_EQ(time.nsec, tv.tv_usec * 1000);

  time.SetToWallTime();
  EXPECT_TRUE(common::Time::GetWallTime() - time < common::Time(0, 1000000));

  time = common::Time(1, 1000) + common::Time(1.5, 1000000000);
  EXPECT_TRUE(time == common::Time(3.5, 1000));

  time.Set(1, 1000);
  time += common::Time(1.5, 1000000000);
  EXPECT_TRUE(time == common::Time(3.5, 1000));

  time.Set(1, 1000);
  time -= common::Time(1, 1000);
  EXPECT_TRUE(time == common::Time(0, 0));

  time.Set(1, 1000);
  time *= common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(2, 2002));

  time.Set(2, 4000);
  time /= common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(1, 1999));
  EXPECT_FALSE(time != common::Time(1, 1999));

  time += common::Time(0, 1);
  tv.tv_sec = 1;
  tv.tv_usec = 2;
  EXPECT_TRUE(time == tv);
  EXPECT_FALSE(time != tv);

  tv.tv_sec = 2;
  EXPECT_TRUE(time < tv);

  tv.tv_sec = 0;
  EXPECT_TRUE(time > tv);
  EXPECT_TRUE(time >= tv);


  EXPECT_TRUE(time == 1.0 + 2000*1e-9);
  EXPECT_FALSE(time != 1.0 + 2000*1e-9);
  EXPECT_TRUE(time < 2.0);
  EXPECT_TRUE(time > 0.1);
  EXPECT_TRUE(time >= 0.1);


  tv.tv_sec = 2;
  tv.tv_usec = 1000000;
  time = common::Time(1, 1000) + tv;
  EXPECT_TRUE(time == common::Time(4.0, 1000));

  time.Set(1, 1000);
  time += tv;
  EXPECT_TRUE(time == common::Time(4, 1000));

  time = common::Time(1, 1000) - tv;
  EXPECT_TRUE(time == common::Time(-2, 1000));

  time.Set(1, 1000);
  time -= tv;
  EXPECT_TRUE(time == common::Time(-2, 1000));

  tv.tv_sec = 2;
  tv.tv_usec = 1000;
  time = common::Time(1, 1000) * tv;
  EXPECT_TRUE(time == common::Time(2, 1002001));

  time.Set(1, 1000);
  time *= tv;
  EXPECT_TRUE(time == common::Time(2, 1002001));

  time.Set(1, 1000);
  time = common::Time(1, 1000) * common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(2, 2002));


  time = common::Time(1, 2000000) / tv;
  EXPECT_TRUE(time == common::Time(0, 500749625));

  time.Set(1, 2000000);
  time /= tv;
  EXPECT_TRUE(time == common::Time(0, 500749625));

  time.Set(1, 1000);
  time = common::Time(1, 1000) / common::Time(2, 2);
  EXPECT_TRUE(time == common::Time(0, 500000499));
}



TEST_F(CommonTest, Material)
{
  common::Material mat(common::Color(1.0, 0.5, 0.2, 1.0));
  EXPECT_TRUE(mat.GetAmbient() == common::Color(1.0, 0.5, 0.2, 1.0));
  EXPECT_TRUE(mat.GetDiffuse() == common::Color(1.0, 0.5, 0.2, 1.0));
  EXPECT_STREQ("gazebo_material_0", mat.GetName().c_str());

  mat.SetTextureImage("texture_image");
  EXPECT_STREQ("texture_image", mat.GetTextureImage().c_str());

  mat.SetTextureImage("texture_image", "/path");
  EXPECT_STREQ("/path/../materials/textures/texture_image",
               mat.GetTextureImage().c_str());

  mat.SetAmbient(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetAmbient() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetDiffuse(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetDiffuse() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetSpecular(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetSpecular() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetEmissive(common::Color(0.1, 0.2, 0.3, 0.4));
  EXPECT_TRUE(mat.GetEmissive() == common::Color(0.1, 0.2, 0.3, 0.4));

  mat.SetTransparency(0.2);
  EXPECT_EQ(0.2, mat.GetTransparency());

  mat.SetShininess(0.2);
  EXPECT_EQ(0.2, mat.GetShininess());

  mat.SetBlendFactors(.1, .5);
  double a, b;
  mat.GetBlendFactors(a, b);
  EXPECT_EQ(.1, a);
  EXPECT_EQ(0.5, b);

  mat.SetBlendMode(common::Material::MODULATE);
  EXPECT_EQ(common::Material::MODULATE, mat.GetBlendMode());

  mat.SetShadeMode(common::Material::BLINN);
  EXPECT_EQ(common::Material::BLINN, mat.GetShadeMode());

  mat.SetPointSize(0.2);
  EXPECT_EQ(0.2, mat.GetPointSize());

  mat.SetDepthWrite(false);
  EXPECT_FALSE(mat.GetDepthWrite());

  mat.SetLighting(true);
  EXPECT_TRUE(mat.GetLighting());
}

TEST_F(CommonTest, Console)
{
  gzlog << "Log test\n";
  common::Console::Instance()->Load();
  common::Console::Instance()->SetQuiet(true);
}

TEST_F(CommonTest, Exception)
{
  try
  {
    gzthrow("test");
  }
  catch(common::Exception &_e)
  {
    std::cout << "Exception[" << _e.GetErrorFile() << "]\n";
    std::cout << "Exception[" << _e.GetErrorStr() << "]\n";
  }
}

TEST_F(CommonTest, Diagnostics)
{
  common::DiagnosticManager *mgr = common::DiagnosticManager::Instance();
  EXPECT_TRUE(mgr != NULL);

  mgr->SetEnabled(true);
  EXPECT_TRUE(mgr->GetEnabled());

  common::Time prev = common::Time::GetWallTime();
  {
    common::DiagnosticTimerPtr timer = mgr->CreateTimer("test");
    EXPECT_STREQ("test", timer->GetName().c_str());
    EXPECT_STREQ("test", mgr->GetLabel(0).c_str());
    EXPECT_EQ(1, mgr->GetTimerCount());
  }
  common::Time after = common::Time::GetWallTime();

  EXPECT_TRUE(mgr->GetTime(0) == mgr->GetTime("test"));
  EXPECT_TRUE(mgr->GetTime(0) <= after - prev);
}

TEST_F(CommonTest, Mesh)
{
  EXPECT_EQ(NULL, common::MeshManager::Instance()->Load("break.mesh"));
  EXPECT_EQ(NULL, common::MeshManager::Instance()->Load("break.3ds"));
  EXPECT_EQ(NULL, common::MeshManager::Instance()->Load("break.xml"));

  const common::Mesh *mesh =
    common::MeshManager::Instance()->GetMesh("unit_box");
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetVertexCount());
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetNormalCount());
  EXPECT_EQ(static_cast<unsigned int>(36), mesh->GetIndexCount());
  EXPECT_EQ(static_cast<unsigned int>(24), mesh->GetTexCoordCount());
  EXPECT_EQ(static_cast<unsigned int>(0), mesh->GetMaterialCount());

  math::Vector3 center, min, max;
  mesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == math::Vector3(0, 0, 0));
  EXPECT_TRUE(min == math::Vector3(-.5, -.5, -.5));
  EXPECT_TRUE(max == math::Vector3(.5, .5, .5));


  float *vertArray = NULL;
  int *indArray = NULL;
  mesh->FillArrays(&vertArray, &indArray);

  int i = 0;
  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);

  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);

  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);

  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);

  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);

  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);

  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);

  EXPECT_EQ(-.5, vertArray[i++]);
  EXPECT_EQ(.5, vertArray[i++]);
  EXPECT_EQ(-.5, vertArray[i++]);

  common::Mesh *newMesh = new common::Mesh();
  newMesh->SetName("testBox");
  common::SubMesh *subMesh = new common::SubMesh();
  newMesh->AddSubMesh(subMesh);

  std::vector<math::Vector3> verts;
  std::vector<math::Vector3> norms;

  EXPECT_THROW(mesh->GetSubMesh(1), common::Exception);

  for (i = 0; i < 24; ++i)
  {
    verts.push_back(mesh->GetSubMesh(0)->GetVertex(i));
    norms.push_back(mesh->GetSubMesh(0)->GetNormal(i));
  }

  subMesh->CopyVertices(verts);
  subMesh->CopyNormals(norms);
  EXPECT_TRUE(subMesh->HasVertex(math::Vector3(-.5, -.5, -.5)));
  EXPECT_FALSE(subMesh->HasVertex(math::Vector3(0, 0, 0)));

  newMesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == math::Vector3(0, 0, 0));
  EXPECT_TRUE(min == math::Vector3(-.5, -.5, -.5));
  EXPECT_TRUE(max == math::Vector3(.5, .5, .5));

  subMesh->SetVertexCount(1);
  subMesh->SetIndexCount(1);
  subMesh->SetNormalCount(1);
  subMesh->SetTexCoordCount(1);

  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetVertexCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetIndexCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetNormalCount());
  EXPECT_EQ(static_cast<unsigned int>(1), subMesh->GetTexCoordCount());

  subMesh->SetVertex(0, math::Vector3(1, 2, 3));
  EXPECT_TRUE(subMesh->GetVertex(0) == math::Vector3(1, 2, 3));

  subMesh->SetTexCoord(0, math::Vector2d(.1, .2));
  EXPECT_TRUE(subMesh->GetTexCoord(0) == math::Vector2d(.1, .2));

  newMesh->GenSphericalTexCoord(math::Vector3(0, 0, 0));
  delete newMesh;

  std::ofstream stlFile("/tmp/gazebo_stl_test.stl", std::ios::out);
  stlFile << asciiSTLBox;
  stlFile.close();

  mesh = common::MeshManager::Instance()->Load("/tmp/gazebo_stl_test-bad.stl");
  EXPECT_EQ(NULL, mesh);

  mesh = common::MeshManager::Instance()->Load("/tmp/gazebo_stl_test.stl");
  mesh->GetAABB(center, min, max);
  EXPECT_TRUE(center == math::Vector3(0.5, 0.5, 0.5));
  EXPECT_TRUE(min == math::Vector3(0, 0, 0));
  EXPECT_TRUE(max == math::Vector3(1, 1, 1));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
