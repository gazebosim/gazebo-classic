/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <algorithm>
#include <fstream>
#include <sstream>

#include <boost/algorithm/string.hpp>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/BVHLoader.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/math/Matrix3.hh"
#include "gazebo/math/Angle.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
BVHLoader::BVHLoader()
{
}

/////////////////////////////////////////////////
BVHLoader::~BVHLoader()
{
}

/////////////////////////////////////////////////
Skeleton *BVHLoader::Load(const std::string &_filename, double _scale)
{
  std::string fullname = common::find_file(_filename);

  Skeleton *skeleton = NULL;
  std::ifstream file;
  file.open(fullname.c_str());
  std::vector<SkeletonNode*> nodes;
  std::vector<std::vector<std::string> > nodeChannels;
  unsigned int totalChannels = 0;
  std::string line;
  if (file.is_open())
  {
    getline(file, line);
    if (line.find("HIERARCHY") == std::string::npos)
    {
      file.close();
      return NULL;
    }

    SkeletonNode *parent = NULL;
    SkeletonNode *node = NULL;
    while (!file.eof())
    {
      getline(file, line);
      std::vector<std::string> words;
      boost::trim(line);
      boost::split(words, line, boost::is_any_of("   "));
      if (words[0] == "ROOT" || words[0] == "JOINT")
      {
        if (words.size() < 2)
        {
          file.close();
          return NULL;
        }
        SkeletonNode::SkeletonNodeType type = SkeletonNode::JOINT;
        std::string name = words[1];
        node = new SkeletonNode(parent, name, name, type);
        if (words[0] != "End")
          nodes.push_back(node);
      }
      else
        if (words[0] == "OFFSET")
        {
          if (words.size() < 4)
          {
            file.close();
            return NULL;
          }
          math::Vector3 offset = math::Vector3(
              math::parseFloat(words[1]) * _scale,
              math::parseFloat(words[2]) * _scale,
              math::parseFloat(words[3]) * _scale);
          math::Matrix4 transform(math::Matrix4::IDENTITY);
          transform.SetTranslate(offset);
          node->SetTransform(transform);
        }
        else
          if (words[0] == "CHANNELS")
          {
            if (words.size() < 3 ||
                static_cast<size_t>(math::parseInt(words[1]) + 2) >
                 words.size())
            {
              file.close();
              return NULL;
            }
            nodeChannels.push_back(words);
            totalChannels += math::parseInt(words[1]);
          }
          else
            if (words[0] == "{")
              parent = node;
            else
              if (words[0] == "}")
                parent = parent->GetParent();
              else
                if (words.size() == 2 && words[0] == "End"
                        && words[1] == "Site")
                {
                  /// ignore End Sites
                  getline(file, line);  /// read {
                  getline(file, line);  /// read OFFSET
                  getline(file, line);  /// read }
                }
                else
                {
                  if (nodes.empty())
                  {
                    file.close();
                    return NULL;
                  }
                  skeleton = new Skeleton(nodes[0]);
                  break;
                }
    }
  }
  getline(file, line);
  std::vector<std::string> words;
  boost::trim(line);
  boost::split(words, line, boost::is_any_of("   "));
  unsigned int frameCount = 0;
  double frameTime = 0.0;
  if (words[0] != "Frames:" || words.size() < 2)
  {
    file.close();
    return NULL;
  }
  else
    frameCount = math::parseInt(words[1]);

  getline(file, line);
  words.clear();
  boost::trim(line);
  boost::split(words, line, boost::is_any_of("   "));

  if (words.size() < 3 || words[0] != "Frame" || words[1] != "Time:")
  {
    file.close();
    return NULL;
  }
  else
    frameTime = math::parseFloat(words[2]);

  double time = 0.0;
  unsigned int frameNo = 0;

  SkeletonAnimation *animation = new SkeletonAnimation(_filename);

  while (!file.eof())
  {
    getline(file, line);
    words.clear();
    boost::trim(line);
    boost::split(words, line, boost::is_any_of("   "));
    if (words.size() < totalChannels)
    {
      gzwarn << "Frame " << frameNo << " invalid.\n";
      frameNo++;
      time += frameTime;
      continue;
    }

    unsigned int cursor = 0;
    for (unsigned int i = 0; i < nodes.size(); i++)
    {
      SkeletonNode *node = nodes[i];
      std::vector<std::string> channels = nodeChannels[i];
      math::Vector3 translation = node->GetTransform().GetTranslation();
      math::Vector3 xAxis(1, 0, 0);
      math::Vector3 yAxis(0, 1, 0);
      math::Vector3 zAxis(0, 0, 1);
      double xAngle = 0.0;
      double yAngle = 0.0;
      double zAngle = 0.0;
      math::Matrix4 transform(math::Matrix4::IDENTITY);
      std::vector<math::Matrix4> mats;
      unsigned int chanCount = math::parseInt(channels[1]);
      for (unsigned int j = 2; j < (2 + chanCount); j++)
      {
        double value = math::parseFloat(words[cursor]);
        cursor++;
        std::string channel = channels[j];
        if (channel == "Xposition")
          translation.x = value * _scale;
        else
          if (channel == "Yposition")
            translation.y = value * _scale;
          else
            if (channel == "Zposition")
            {
              translation.z = value * _scale;
            }
            else
              if (channel == "Zrotation")
              {
                zAngle = GZ_DTOR(value);
                mats.push_back(math::Quaternion(zAxis, zAngle).GetAsMatrix4());
              }
              else
                if (channel == "Xrotation")
                {
                  xAngle = GZ_DTOR(value);
                  mats.push_back(
                    math::Quaternion(xAxis, xAngle).GetAsMatrix4());
                }
                else
                  if (channel == "Yrotation")
                  {
                    yAngle = GZ_DTOR(value);
                    mats.push_back(
                      math::Quaternion(yAxis, yAngle).GetAsMatrix4());
                  }
      }
      while (!mats.empty())
      {
        transform = mats.back() * transform;
        mats.pop_back();
      }
      math::Matrix4 pos(math::Matrix4::IDENTITY);
      pos.SetTranslate(translation);
      transform = pos * transform;
      animation->AddKeyFrame(node->GetName(), time, transform);
    }

    frameNo++;
    time += frameTime;
    if (frameNo == frameCount)
      break;
  }
  if (frameNo < frameCount - 1)
    gzwarn << "BVH file ended unexpectedly.\n";

  skeleton->AddAnimation(animation);

  file.close();
  return skeleton;
}
