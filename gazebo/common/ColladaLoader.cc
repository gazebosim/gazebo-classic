/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <curl/curl.h>
#include <tinyxml.h>
#include <math.h>
#include <sstream>
#include <set>
#include <memory>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/unordered_map.hpp>

#include <ignition/math/Color.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/common/SkeletonAnimation.hh"
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/ColladaLoaderPrivate.hh"
#include "gazebo/common/ColladaLoader.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
// std::unary_function was removed in c++17 but can be reproduced easily
// copied from https://stackoverflow.com/a/56001387
template <typename ArgumentType, typename ResultType>
struct unary_function
{
    using argument_type = ArgumentType;
    using result_type = ResultType;
};

/////////////////////////////////////////////////
struct Vector3Hash : unary_function<const ignition::math::Vector3d,
  std::size_t>
{
  std::size_t operator()(const ignition::math::Vector3d &_v) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, _v.X());
    boost::hash_combine(seed, _v.Y());
    boost::hash_combine(seed, _v.Z());
    return seed;
  }
};

/////////////////////////////////////////////////
struct Vector2dHash : unary_function<const ignition::math::Vector2d,
  std::size_t>
{
  std::size_t operator()(const ignition::math::Vector2d &_v) const
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, _v.X());
    boost::hash_combine(seed, _v.Y());
    return seed;
  }
};

//////////////////////////////////////////////////
  ColladaLoader::ColladaLoader()
: MeshLoader(), dataPtr(new ColladaLoaderPrivate)
{
  this->dataPtr->meter = 1.0;
}

//////////////////////////////////////////////////
ColladaLoader::~ColladaLoader()
{
  delete this->dataPtr;
  this->dataPtr = 0;
}

//////////////////////////////////////////////////
Mesh *ColladaLoader::Load(const std::string &_filename)
{
  this->dataPtr->positionIds.clear();
  this->dataPtr->normalIds.clear();
  this->dataPtr->texcoordIds.clear();
  this->dataPtr->materialIds.clear();
  this->dataPtr->positionDuplicateMap.clear();
  this->dataPtr->normalDuplicateMap.clear();
  this->dataPtr->texcoordDuplicateMap.clear();

  // reset scale
  this->dataPtr->meter = 1.0;

  TiXmlDocument xmlDoc;

  boost::filesystem::path p(_filename);
  this->dataPtr->path = p.parent_path().generic_string();

  this->dataPtr->filename = _filename;
  if (!xmlDoc.LoadFile(_filename))
    gzerr << "Unable to load collada file[" << _filename << "]\n";

  this->dataPtr->colladaXml = xmlDoc.FirstChildElement("COLLADA");
  if (!this->dataPtr->colladaXml)
    gzerr << "Missing COLLADA tag\n";

  if (std::string(this->dataPtr->colladaXml->Attribute("version")) != "1.4.0" &&
      std::string(this->dataPtr->colladaXml->Attribute("version")) != "1.4.1")
    gzerr << "Invalid collada file. Must be version 1.4.0 or 1.4.1\n";

  TiXmlElement *assetXml =
      this->dataPtr->colladaXml->FirstChildElement("asset");
  if (assetXml)
  {
    TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
    if (unitXml && unitXml->Attribute("meter"))
      this->dataPtr->meter = ignition::math::parseFloat(
          unitXml->Attribute("meter"));
  }

  Mesh *mesh = new Mesh();
  mesh->SetPath(this->dataPtr->path);

  this->LoadScene(mesh);

  if (mesh->HasSkeleton())
    ApplyInvBindTransform(mesh->GetSkeleton());

  // This will make the model the correct size.
  mesh->Scale(this->dataPtr->meter);
  if (mesh->HasSkeleton())
    mesh->GetSkeleton()->Scale(this->dataPtr->meter);

  return mesh;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadScene(Mesh *_mesh)
{
  TiXmlElement *sceneXml =
      this->dataPtr->colladaXml->FirstChildElement("scene");
  std::string sceneURL =
    sceneXml->FirstChildElement("instance_visual_scene")->Attribute("url");

  TiXmlElement *visSceneXml = this->GetElementId("visual_scene", sceneURL);
  this->dataPtr->currentScene = visSceneXml;

  if (!visSceneXml)
  {
    gzerr << "Unable to find visual_scene id ='" << sceneURL << "'\n";
    return;
  }

  TiXmlElement *nodeXml = visSceneXml->FirstChildElement("node");

  while (nodeXml)
  {
    this->LoadNode(nodeXml, _mesh, ignition::math::Matrix4d::Identity);
    nodeXml = nodeXml->NextSiblingElement("node");
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadNode(TiXmlElement *_elem, Mesh *_mesh,
    const ignition::math::Matrix4d &_transform)
{
  TiXmlElement *nodeXml;
  TiXmlElement *instGeomXml;

  ignition::math::Matrix4d transform = this->LoadNodeTransform(_elem);
  transform = _transform * transform;

  if (_elem->Attribute("name"))
  {
    this->dataPtr->currentNodeName = _elem->Attribute("name");
  }

  nodeXml = _elem->FirstChildElement("node");
  while (nodeXml)
  {
    this->LoadNode(nodeXml, _mesh, transform);
    nodeXml = nodeXml->NextSiblingElement("node");
  }

  if (_elem->FirstChildElement("instance_node"))
  {
    std::string nodeURLStr =
      _elem->FirstChildElement("instance_node")->Attribute("url");

    nodeXml = this->GetElementId("node", nodeURLStr);
    if (!nodeXml)
    {
      gzerr << "Unable to find node[" << nodeURLStr << "]\n";
      return;
    }
    this->LoadNode(nodeXml, _mesh, transform);
    return;
  }
  else
    nodeXml = _elem;

  instGeomXml = nodeXml->FirstChildElement("instance_geometry");
  while (instGeomXml)
  {
    std::string geomURL = instGeomXml->Attribute("url");
    TiXmlElement *geomXml = this->GetElementId("geometry", geomURL);

    this->dataPtr->materialMap.clear();
    TiXmlElement *bindMatXml, *techniqueXml, *matXml;
    bindMatXml = instGeomXml->FirstChildElement("bind_material");
    while (bindMatXml)
    {
      if ((techniqueXml = bindMatXml->FirstChildElement("technique_common")))
      {
        matXml = techniqueXml->FirstChildElement("instance_material");
        while (matXml)
        {
          std::string symbol = matXml->Attribute("symbol");
          std::string target = matXml->Attribute("target");
          this->dataPtr->materialMap[symbol] = target;
          matXml = matXml->NextSiblingElement("instance_material");
        }
      }
      bindMatXml = bindMatXml->NextSiblingElement("bind_material");
    }

    if (_mesh->GetSkeleton())
      _mesh->GetSkeleton()->SetNumVertAttached(0);
    this->LoadGeometry(geomXml, transform, _mesh);
    instGeomXml = instGeomXml->NextSiblingElement("instance_geometry");
  }

  TiXmlElement *instContrXml =
    nodeXml->FirstChildElement("instance_controller");
  while (instContrXml)
  {
    std::string contrURL = instContrXml->Attribute("url");
    TiXmlElement *contrXml = this->GetElementId("controller", contrURL);

    this->dataPtr->materialMap.clear();
    TiXmlElement *bindMatXml, *techniqueXml, *matXml;
    bindMatXml = instContrXml->FirstChildElement("bind_material");
    while (bindMatXml)
    {
      if ((techniqueXml = bindMatXml->FirstChildElement("technique_common")))
      {
        matXml = techniqueXml->FirstChildElement("instance_material");
        while (matXml)
        {
          std::string symbol = matXml->Attribute("symbol");
          std::string target = matXml->Attribute("target");
          this->dataPtr->materialMap[symbol] = target;
          matXml = matXml->NextSiblingElement("instance_material");
        }
      }
      bindMatXml = bindMatXml->NextSiblingElement("bind_material");
    }

    std::vector<TiXmlElement*> rootNodeXmls;
    TiXmlNode *child = nullptr;
    while ((child = instContrXml->IterateChildren("skeleton", child)))
    {
      TiXmlElement *instSkelXml = child->ToElement();
      std::string rootURL = instSkelXml->GetText();
      rootNodeXmls.emplace_back(this->GetElementId("node", rootURL));
    }
    // no skeleton tag present, assume whole scene is a skeleton
    if (rootNodeXmls.empty())
    {
      TiXmlNode *childNode = nullptr;
      while ((childNode =
          this->dataPtr->currentScene->IterateChildren("node", childNode)))
      {
        rootNodeXmls.emplace_back(childNode->ToElement());
      }
    }

    this->LoadController(contrXml, rootNodeXmls, transform, _mesh);
    instContrXml = instContrXml->NextSiblingElement("instance_controller");
  }
}

/////////////////////////////////////////////////
ignition::math::Matrix4d ColladaLoader::LoadNodeTransform(TiXmlElement *_elem)
{
  ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);

  if (_elem->FirstChildElement("matrix"))
  {
    std::string matrixStr = _elem->FirstChildElement("matrix")->GetText();
    std::istringstream iss(matrixStr);
    std::vector<double> values(16);
    for (unsigned int i = 0; i < 16; ++i)
      iss >> values[i];
    transform.Set(values[0], values[1], values[2], values[3],
        values[4], values[5], values[6], values[7],
        values[8], values[9], values[10], values[11],
        values[12], values[13], values[14], values[15]);
  }
  else
  {
    if (_elem->FirstChildElement("translate"))
    {
      std::string transStr = _elem->FirstChildElement("translate")->GetText();
      ignition::math::Vector3d translate;
      translate = boost::lexical_cast<ignition::math::Vector3d>(transStr);
      // translate *= this->dataPtr->meter;
      transform.SetTranslation(translate);
    }

    TiXmlElement *rotateXml = _elem->FirstChildElement("rotate");
    while (rotateXml)
    {
      ignition::math::Matrix3d mat;
      ignition::math::Vector3d axis;
      double angle;

      std::string rotateStr = rotateXml->GetText();
      std::istringstream iss(rotateStr);

      iss >> axis.X() >> axis.Y() >> axis.Z();
      iss >> angle;
      mat.Axis(axis, IGN_DTOR(angle));
      ignition::math::Matrix4d mat4(ignition::math::Matrix4d::Identity);
      mat4 = mat;

      transform = transform * mat4;

      rotateXml = rotateXml->NextSiblingElement("rotate");
    }

    if (_elem->FirstChildElement("scale"))
    {
      std::string scaleStr = _elem->FirstChildElement("scale")->GetText();
      ignition::math::Vector3d scale;
      scale = boost::lexical_cast<ignition::math::Vector3d>(scaleStr);
      ignition::math::Matrix4d scaleMat;
      scaleMat.Scale(scale);
      transform = transform * scaleMat;
    }
  }

  return transform;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadController(TiXmlElement *_contrXml,
      const std::vector<TiXmlElement*> &rootNodeXmls,
      const ignition::math::Matrix4d &_transform, Mesh *_mesh)
{
  TiXmlElement *skinXml = _contrXml->FirstChildElement("skin");
  std::string geomURL = skinXml->Attribute("source");

  ignition::math::Matrix4d bindTrans;
  std::string matrixStr =
        skinXml->FirstChildElement("bind_shape_matrix")->GetText();
  std::istringstream iss(matrixStr);
  std::vector<double> values(16);
  for (unsigned int i = 0; i < 16; ++i)
    iss >> values[i];
  bindTrans.Set(values[0], values[1], values[2], values[3],
                values[4], values[5], values[6], values[7],
                values[8], values[9], values[10], values[11],
                values[12], values[13], values[14], values[15]);

  TiXmlElement *jointsXml = skinXml->FirstChildElement("joints");
  std::string jointsURL, invBindMatURL;
  TiXmlElement *inputXml = jointsXml->FirstChildElement("input");
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string source = inputXml->Attribute("source");
    if (semantic == "JOINT")
      jointsURL = source;
    else
    {
      if (semantic == "INV_BIND_MATRIX")
        invBindMatURL = source;
    }
    inputXml = inputXml->NextSiblingElement("input");
  }

  jointsXml = this->GetElementId("source", jointsURL);

  if (!jointsXml)
  {
    gzerr << "Could not find node[" << jointsURL << "]\n";
    gzthrow("Faild to parse skinning information in Collada file.");
  }

  std::string jointsStr = jointsXml->FirstChildElement("Name_array")->GetText();

  std::vector<std::string> joints;
  boost::split(joints, jointsStr, boost::is_any_of(" \t"));

  // Load the skeleton
  Skeleton *skeleton = nullptr;
  if (_mesh->HasSkeleton())
    skeleton = _mesh->GetSkeleton();
  for (TiXmlElement *rootNodeXml : rootNodeXmls)
  {
    SkeletonNode *rootSkelNode =
        this->LoadSkeletonNodes(rootNodeXml, nullptr);
    if (skeleton)
      this->MergeSkeleton(skeleton, rootSkelNode);
    else
    {
      skeleton = new Skeleton(rootSkelNode);
      _mesh->SetSkeleton(skeleton);
    }
  }
  skeleton->SetBindShapeTransform(bindTrans);

  TiXmlElement *rootXml = _contrXml->GetDocument()->RootElement();
  if (rootXml->FirstChildElement("library_animations"))
  {
    this->LoadAnimations(rootXml->FirstChildElement("library_animations"),
        skeleton);
  }

  TiXmlElement *invBMXml = this->GetElementId("source", invBindMatURL);

  if (!invBMXml)
  {
    gzerr << "Could not find node[" << invBindMatURL << "]\n";
    gzthrow("Faild to parse skinning information in Collada file.");
  }

  std::string posesStr = invBMXml->FirstChildElement("float_array")->GetText();

  std::vector<std::string> strs;
  boost::split(strs, posesStr, boost::is_any_of(" \t"));

  for (unsigned int i = 0; i < joints.size(); ++i)
  {
    unsigned int id = i * 16;
    ignition::math::Matrix4d mat;
    mat.Set(ignition::math::parseFloat(strs[id +  0]),
            ignition::math::parseFloat(strs[id +  1]),
            ignition::math::parseFloat(strs[id +  2]),
            ignition::math::parseFloat(strs[id +  3]),
            ignition::math::parseFloat(strs[id +  4]),
            ignition::math::parseFloat(strs[id +  5]),
            ignition::math::parseFloat(strs[id +  6]),
            ignition::math::parseFloat(strs[id +  7]),
            ignition::math::parseFloat(strs[id +  8]),
            ignition::math::parseFloat(strs[id +  9]),
            ignition::math::parseFloat(strs[id + 10]),
            ignition::math::parseFloat(strs[id + 11]),
            ignition::math::parseFloat(strs[id + 12]),
            ignition::math::parseFloat(strs[id + 13]),
            ignition::math::parseFloat(strs[id + 14]),
            ignition::math::parseFloat(strs[id + 15]));

    skeleton->GetNodeByName(joints[i])->SetInverseBindTransform(mat);
  }

  TiXmlElement *vertWeightsXml = skinXml->FirstChildElement("vertex_weights");

  inputXml = vertWeightsXml->FirstChildElement("input");
  unsigned int jOffset = 0;
  unsigned int wOffset = 0;
  std::string weightsURL;
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string source = inputXml->Attribute("source");
    int offset;
    inputXml->Attribute("offset", &offset);

    if (semantic == "JOINT")
      jOffset = offset;
    else
      if (semantic == "WEIGHT")
      {
        weightsURL = source;
        wOffset = offset;
      }
    inputXml = inputXml->NextSiblingElement("input");
  }

  TiXmlElement *weightsXml = this->GetElementId("source", weightsURL);

  std::string wString = weightsXml->FirstChildElement("float_array")->GetText();
  std::vector<std::string> wStrs;
  boost::split(wStrs, wString, boost::is_any_of(" \t"));

  std::vector<float> weights;
  for (unsigned int i = 0; i < wStrs.size(); ++i)
    weights.push_back(ignition::math::parseFloat(wStrs[i]));

  std::string cString = vertWeightsXml->FirstChildElement("vcount")->GetText();
  std::string vString = vertWeightsXml->FirstChildElement("v")->GetText();
  std::vector<std::string> vCountStrs;
  std::vector<std::string> vStrs;

  boost::split(vCountStrs, cString, boost::is_any_of(" \t"));
  boost::split(vStrs, vString, boost::is_any_of(" \t"));

  std::vector<unsigned int> vCount;
  std::vector<unsigned int> v;

  for (unsigned int i = 0; i < vCountStrs.size(); ++i)
    vCount.push_back(ignition::math::parseInt(vCountStrs[i]));

  for (unsigned int i = 0; i < vStrs.size(); ++i)
    v.push_back(ignition::math::parseInt(vStrs[i]));

  skeleton->SetNumVertAttached(vCount.size());

  unsigned int vIndex = 0;
  for (unsigned int i = 0; i < vCount.size(); ++i)
  {
    for (unsigned int j = 0; j < vCount[i]; ++j)
    {
      skeleton->AddVertNodeWeight(i, joints[v[vIndex + jOffset]],
                                    weights[v[vIndex + wOffset]]);
      vIndex += (jOffset + wOffset + 1);
    }
  }

  TiXmlElement *geomXml = this->GetElementId("geometry", geomURL);
  this->LoadGeometry(geomXml, _transform, _mesh);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadAnimations(TiXmlElement *_xml, Skeleton *_skel)
{
  TiXmlElement *childXml = _xml->FirstChildElement("animation");
  if (childXml->FirstChildElement("animation"))
  {
    while (childXml)
    {
      this->LoadAnimationSet(childXml, _skel);
      childXml = childXml->NextSiblingElement("animation");
    }
  }
  else
    this->LoadAnimationSet(_xml, _skel);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadAnimationSet(TiXmlElement *_xml, Skeleton *_skel)
{
  std::stringstream animName;
  if (_xml->Attribute("name"))
    animName << _xml->Attribute("name");
  else
    if (_xml->Attribute("id"))
      animName << _xml->Attribute("id");
    else
      animName << "animation" << (_skel->GetNumAnimations() + 1);

  RawSkeletonAnim animation;

  TiXmlElement *animXml = _xml->FirstChildElement("animation");
  while (animXml)
  {
    TiXmlElement *chanXml = animXml->FirstChildElement("channel");

    while (chanXml)
    {
      std::string sourceURL = chanXml->Attribute("source");
      std::string targetStr = chanXml->Attribute("target");

      std::string targetBone = targetStr.substr(0, targetStr.find('/'));
      char sep = '0';
      if (targetStr.find('(') != std::string::npos)
        sep = '(';
      else
        if (targetStr.find('.') != std::string::npos)
          sep = '.';

      std::string targetTrans;
      if (sep == '0')
        targetTrans = targetStr.substr(targetStr.find('/') + 1);
      else
        targetTrans = targetStr.substr(targetStr.find('/') + 1,
                          targetStr.find(sep) - targetStr.find('/') - 1);

      std::string idxStr = targetStr.substr(targetStr.find(sep) + 1);
      int idx1 = -1;
      int idx2 = -1;

      if (sep == '.')
        idx1 = (idxStr == "X") ? 0 : ((idxStr == "Y") ? 1 : ((idxStr == "Z")
            ? 2 : ((idxStr == "ANGLE") ? 3 : -1)));
      else
        if (sep == '(')
        {
          std::string idx1Str = idxStr.substr(0, 1);
          idx1 = ignition::math::parseInt(idx1Str);
          if (idxStr.length() > 4)
          {
            std::string idx2Str = idxStr.substr(3, 1);
            idx2 = ignition::math::parseInt(idx2Str);
          }
        }

      TiXmlElement *frameTimesXml = nullptr;
      TiXmlElement *frameTransXml = nullptr;

      TiXmlElement *sampXml = this->GetElementId("sampler", sourceURL);
      TiXmlElement *inputXml = sampXml->FirstChildElement("input");
      while (inputXml)
      {
        std::string semantic = inputXml->Attribute("semantic");
        if (semantic == "INPUT")
          frameTimesXml = this->GetElementId("source",
                              inputXml->Attribute("source"));
        else
          if (semantic == "OUTPUT")
            frameTransXml = this->GetElementId("source",
                              inputXml->Attribute("source"));
        /// FIXME interpolation semantic?

        inputXml = inputXml->NextSiblingElement("input");
      }
      TiXmlElement *timeArray = frameTimesXml->FirstChildElement("float_array");
      std::string timeStr = timeArray->GetText();
      std::vector<std::string> timeStrs;
      boost::split(timeStrs, timeStr, boost::is_any_of(" \t"));

      std::vector<double> times;
      for (unsigned int i = 0; i < timeStrs.size(); ++i)
        times.push_back(ignition::math::parseFloat(timeStrs[i]));

      TiXmlElement *output = frameTransXml->FirstChildElement("float_array");
      std::string outputStr = output->GetText();
      std::vector<std::string> outputStrs;
      boost::split(outputStrs, outputStr, boost::is_any_of(" \t"));

      std::vector<double> values;
      for (unsigned int i = 0; i < outputStrs.size(); ++i)
        values.push_back(ignition::math::parseFloat(outputStrs[i]));

      TiXmlElement *accessor =
        frameTransXml->FirstChildElement("technique_common");
      accessor = accessor->FirstChildElement("accessor");

      // stride is optional, default to 1
      unsigned int stride = 1;
      auto *strideAttribute = accessor->Attribute("stride");
      if (strideAttribute)
      {
        stride = static_cast<unsigned int>(
            ignition::math::parseInt(strideAttribute));
      }

      SkeletonNode *targetNode = _skel->GetNodeById(targetBone);
      if (targetNode == nullptr)
      {
        TiXmlElement *targetNodeXml = this->GetElementId("node", targetBone);
        if (targetNodeXml == nullptr)
        {
          gzerr << "Failed to load animation, '" << targetBone << "' not found"
              << std::endl;
          gzthrow("Failed to load animation");
        }
        targetNode = this->LoadSkeletonNodes(targetNodeXml, nullptr);
        this->MergeSkeleton(_skel, targetNode);
      }

      // In COLLOADA, `target` is specified to be the `id` of a node, however
      // the nodes are identified by `name` in this loader. Here, we resolve
      // `targetBone` to the node's `name` to prevent missing animations.
      std::string targetBoneName = targetNode->GetName();
      for (unsigned int i = 0; i < times.size(); ++i)
      {
        if (animation[targetBoneName].find(times[i])
            == animation[targetBoneName].end())
        {
          animation[targetBoneName][times[i]] =
              _skel->GetNodeById(targetBone)->GetTransforms();
        }

        std::vector<NodeTransform> *frame =
            &animation[targetBoneName][times[i]];

        for (unsigned int j = 0; j < (*frame).size(); ++j)
        {
          NodeTransform *nt = &((*frame)[j]);
          if (nt->GetSID() == targetTrans)
          {
            if (idx1 != -1)
            {
              int index = (idx2 == -1) ? idx1 : (idx1 * 4) + idx2;
              nt->SetComponent(index, values[i]);
            }
            else
              for (unsigned int k = 0; k < stride; k++)
                nt->SetComponent(k, values[(i*stride) + k]);
          }
        }
      }

      chanXml = chanXml->NextSiblingElement("channel");
    }

    animXml = animXml->NextSiblingElement("animation");
  }

  SkeletonAnimation *anim = new SkeletonAnimation(animName.str());

  for (RawSkeletonAnim::iterator iter = animation.begin();
        iter != animation.end(); ++iter)
    for (RawNodeAnim::iterator niter = iter->second.begin();
          niter != iter->second.end(); ++niter)
    {
      ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);
      for (unsigned int i = 0; i < niter->second.size(); ++i)
      {
        niter->second[i].RecalculateMatrix();
        transform = transform * niter->second[i]();
      }
      anim->AddKeyFrame(iter->first, niter->first, transform);
    }

  _skel->AddAnimation(anim);
}

/////////////////////////////////////////////////
SkeletonNode* ColladaLoader::LoadSingleSkeletonNode(TiXmlElement *_xml,
      SkeletonNode *_parent)
{
  std::string name;
  if (_xml->Attribute("sid"))
    name = _xml->Attribute("sid");
  else if (_xml->Attribute("name"))
    name = _xml->Attribute("name");
  else
    name = _xml->Attribute("id");

  SkeletonNode* node = new SkeletonNode(_parent, name, _xml->Attribute("id"));

  if (!_xml->Attribute("type")
      || std::string(_xml->Attribute("type")) == "NODE")
  {
    node->SetType(SkeletonNode::NODE);
  }

  return node;
}

/////////////////////////////////////////////////
SkeletonNode* ColladaLoader::LoadSkeletonNodes(TiXmlElement *_xml,
      SkeletonNode *_parent)
{
  SkeletonNode *node = this->LoadSingleSkeletonNode(_xml, _parent);
  this->SetSkeletonNodeTransform(_xml, node);
  TiXmlElement *childXml = _xml->FirstChildElement("node");
  while (childXml)
  {
    this->LoadSkeletonNodes(childXml, node);
    childXml = childXml->NextSiblingElement("node");
  }
  return node;
}

/////////////////////////////////////////////////
void ColladaLoader::SetSkeletonNodeTransform(TiXmlElement *_elem,
      SkeletonNode *_node)
{
  ignition::math::Matrix4d transform(ignition::math::Matrix4d::Identity);

  if (_elem->FirstChildElement("matrix"))
  {
    std::string matrixStr = _elem->FirstChildElement("matrix")->GetText();
    std::istringstream iss(matrixStr);
    std::vector<double> values(16);
    for (unsigned int i = 0; i < 16; ++i)
      iss >> values[i];
    transform.Set(values[0], values[1], values[2], values[3],
        values[4], values[5], values[6], values[7],
        values[8], values[9], values[10], values[11],
        values[12], values[13], values[14], values[15]);

    NodeTransform nt(transform);
    nt.SetSourceValues(transform);
    if (_elem->FirstChildElement("matrix")->Attribute("sid"))
      nt.SetSID(_elem->FirstChildElement("matrix")->Attribute("sid"));
    _node->AddRawTransform(nt);
  }
  else
  {
    if (_elem->FirstChildElement("translate"))
    {
      std::string transStr = _elem->FirstChildElement("translate")->GetText();
      ignition::math::Vector3d translate;
      translate = boost::lexical_cast<ignition::math::Vector3d>(transStr);
      // translate *= this->dataPtr->meter;
      transform.SetTranslation(translate);

      NodeTransform nt(transform);
      if (_elem->FirstChildElement("translate")->Attribute("sid"))
        nt.SetSID(_elem->FirstChildElement("translate")->Attribute("sid"));
      nt.SetType(NodeTransform::TRANSLATE);
      nt.SetSourceValues(translate);
      _node->AddRawTransform(nt);
    }

    TiXmlElement *rotateXml = _elem->FirstChildElement("rotate");
    while (rotateXml)
    {
      ignition::math::Matrix3d mat;
      ignition::math::Vector3d axis;
      double angle;

      std::string rotateStr = rotateXml->GetText();
      std::istringstream iss(rotateStr);

      iss >> axis.X() >> axis.Y() >> axis.Z();
      iss >> angle;
      mat.Axis(axis, IGN_DTOR(angle));

      ignition::math::Matrix4d mat4(ignition::math::Matrix4d::Identity);
      mat4 = mat;
      NodeTransform nt(mat4);
      if (rotateXml->Attribute("sid"))
        nt.SetSID(rotateXml->Attribute("sid"));
      nt.SetType(NodeTransform::ROTATE);
      nt.SetSourceValues(axis, angle);
      _node->AddRawTransform(nt);

      transform = transform * mat4;

      rotateXml = rotateXml->NextSiblingElement("rotate");
    }

    if (_elem->FirstChildElement("scale"))
    {
      std::string scaleStr = _elem->FirstChildElement("scale")->GetText();
      ignition::math::Vector3d scale;
      scale = boost::lexical_cast<ignition::math::Vector3d>(scaleStr);
      ignition::math::Matrix4d scaleMat;
      scaleMat.Scale(scale);

      NodeTransform nt(scaleMat);
      TiXmlElement *matrix = _elem->FirstChildElement("matrix");
      if (matrix && matrix->Attribute("sid"))
        nt.SetSID(_elem->FirstChildElement("matrix")->Attribute("sid"));
      nt.SetType(NodeTransform::SCALE);
      nt.SetSourceValues(scale);
      _node->AddRawTransform(nt);

      transform = transform * scaleMat;
    }
  }

  _node->SetTransform(transform);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadGeometry(TiXmlElement *_xml,
    const ignition::math::Matrix4d &_transform, Mesh *_mesh)
{
  TiXmlElement *meshXml = _xml->FirstChildElement("mesh");
  TiXmlElement *childXml;

  if (!meshXml)
    return;

  childXml = meshXml->FirstChildElement("triangles");
  while (childXml)
  {
    this->LoadTriangles(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("triangles");
  }

  childXml = meshXml->FirstChildElement("polylist");
  while (childXml)
  {
    this->LoadPolylist(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("polylist");
  }

  childXml = meshXml->FirstChildElement("lines");
  while (childXml)
  {
    this->LoadLines(childXml, _transform, _mesh);
    childXml = childXml->NextSiblingElement("lines");
  }
}

/////////////////////////////////////////////////
TiXmlElement *ColladaLoader::GetElementId(const std::string &_name,
                                          const std::string &_id)
{
  return this->GetElementId(this->dataPtr->colladaXml, _name, _id);
}

/////////////////////////////////////////////////
TiXmlElement *ColladaLoader::GetElementId(TiXmlElement *_parent,
    const std::string &_name,
    const std::string &_id)
{
  std::string id = _id;
  if (id.length() > 0 && id[0] == '#')
    id.erase(0, 1);

  if ((id.empty() && _parent->Value() == _name) ||
      (_parent->Attribute("id") && _parent->Attribute("id") == id) ||
      (_parent->Attribute("sid") && _parent->Attribute("sid") == id))
  {
    return _parent;
  }

  TiXmlElement *elem = _parent->FirstChildElement();
  while (elem)
  {
    TiXmlElement *result = this->GetElementId(elem, _name, _id);
    if (result)
    {
      return result;
    }

    elem = elem->NextSiblingElement();
  }

  return nullptr;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadVertices(const std::string &_id,
    const ignition::math::Matrix4d &_transform,
    std::vector<ignition::math::Vector3d> &_verts,
    std::vector<ignition::math::Vector3d> &_norms)
{
  std::map<unsigned int, unsigned int> vertDup;
  std::map<unsigned int, unsigned int> normDup;
  this->LoadVertices(_id, _transform, _verts, _norms, vertDup, normDup);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadVertices(const std::string &_id,
    const ignition::math::Matrix4d &_transform,
    std::vector<ignition::math::Vector3d> &_verts,
    std::vector<ignition::math::Vector3d> &_norms,
    std::map<unsigned int, unsigned int> &_vertDups,
    std::map<unsigned int, unsigned int> &_normDups)
{
  TiXmlElement *verticesXml = this->GetElementId(this->dataPtr->colladaXml,
                                                 "vertices", _id);

  if (!verticesXml)
  {
    gzerr << "Unable to find vertices[" << _id << "] in collada file\n";
    return;
  }

  TiXmlElement *inputXml = verticesXml->FirstChildElement("input");
  while (inputXml)
  {
    std::string semantic = inputXml->Attribute("semantic");
    std::string sourceStr = inputXml->Attribute("source");
    if (semantic == "NORMAL")
    {
      this->LoadNormals(sourceStr, _transform, _norms, _normDups);
    }
    else if (semantic == "POSITION")
    {
      this->LoadPositions(sourceStr, _transform, _verts, _vertDups);
    }

    inputXml = inputXml->NextSiblingElement("input");
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadPositions(const std::string &_id,
    const ignition::math::Matrix4d &_transform,
    std::vector<ignition::math::Vector3d> &_values,
    std::map<unsigned int, unsigned int> &_duplicates)
{
  if (this->dataPtr->positionIds.find(_id) != this->dataPtr->positionIds.end())
  {
    _values = this->dataPtr->positionIds[_id];
    _duplicates = this->dataPtr->positionDuplicateMap[_id];
    return;
  }

  TiXmlElement *sourceXml = this->GetElementId("source", _id);
  if (!sourceXml)
  {
    gzerr << "Unable to find source\n";
    return;
  }

  TiXmlElement *floatArrayXml = sourceXml->FirstChildElement("float_array");
  if (!floatArrayXml || !floatArrayXml->GetText())
  {
    int count = 1;
    if (floatArrayXml && floatArrayXml->Attribute("count"))
    {
      try
      {
        count = boost::lexical_cast<int>(floatArrayXml->Attribute("count"));
      }
      catch(...)
      {
        // Do nothing. Messages are printed out below.
      }
    }

    if (count)
    {
      gzerr << "Vertex source missing float_array element, "
        << "or count is invalid.\n";
    }
    else
    {
      gzlog << "Vertex source has a float_array with a count of zero. "
        << "This is likely not desired\n";
    }

    return;
  }
  std::string valueStr = floatArrayXml->GetText();

  boost::unordered_map<ignition::math::Vector3d,
    unsigned int, Vector3Hash> unique;

  std::vector<std::string> strs;
  std::vector<std::string>::iterator iter, end;
  boost::split(strs, valueStr, boost::is_any_of(" \t"));

  end = strs.end();
  for (iter = strs.begin(); iter != end; iter += 3)
  {
    ignition::math::Vector3d vec(ignition::math::parseFloat(*iter),
        ignition::math::parseFloat(*(iter+1)),
        ignition::math::parseFloat(*(iter+2)));

    vec = _transform * vec;
    _values.push_back(vec);

    // create a map of duplicate indices
    if (unique.find(vec) != unique.end())
      _duplicates[_values.size()-1] = unique[vec];
    else
      unique[vec] = _values.size()-1;
  }

  this->dataPtr->positionDuplicateMap[_id] = _duplicates;
  this->dataPtr->positionIds[_id] = _values;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadNormals(const std::string &_id,
    const ignition::math::Matrix4d &_transform,
    std::vector<ignition::math::Vector3d> &_values,
    std::map<unsigned int, unsigned int> &_duplicates)
{
  if (this->dataPtr->normalIds.find(_id) != this->dataPtr->normalIds.end())
  {
    _values = this->dataPtr->normalIds[_id];
    _duplicates = this->dataPtr->normalDuplicateMap[_id];
    return;
  }

  ignition::math::Matrix4d rotMat = _transform;
  rotMat.SetTranslation(ignition::math::Vector3d::Zero);

  TiXmlElement *normalsXml = this->GetElementId("source", _id);
  if (!normalsXml)
  {
    gzerr << "Unable to find normals[" << _id << "] in collada file\n";
    return;
  }

  TiXmlElement *floatArrayXml = normalsXml->FirstChildElement("float_array");
  if (!floatArrayXml || !floatArrayXml->GetText())
  {
    int count = 1;
    if (floatArrayXml && floatArrayXml->Attribute("count"))
    {
      try
      {
        count = boost::lexical_cast<int>(floatArrayXml->Attribute("count"));
      }
      catch(...)
      {
        // Do nothing. Messages are printed out below.
      }
    }

    if (count)
    {
      gzwarn << "Normal source missing float_array element, or count is "
        << "invalid.\n";
    }
    else
    {
      gzlog << "Normal source has a float_array with a count of zero. "
        << "This is likely not desired\n";
    }

    return;
  }

  boost::unordered_map<ignition::math::Vector3d,
    unsigned int, Vector3Hash> unique;

  std::string valueStr = floatArrayXml->GetText();
  std::istringstream iss(valueStr);
  do
  {
    ignition::math::Vector3d vec;
    iss >> vec.X() >> vec.Y() >> vec.Z();
    if (iss)
    {
      vec = rotMat * vec;
      vec.Normalize();
      _values.push_back(vec);

      // create a map of duplicate indices
      if (unique.find(vec) != unique.end())
        _duplicates[_values.size()-1] = unique[vec];
      else
        unique[vec] = _values.size()-1;
    }
  } while (iss);

  this->dataPtr->normalDuplicateMap[_id] = _duplicates;
  this->dataPtr->normalIds[_id] = _values;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadTexCoords(const std::string &_id,
    std::vector<ignition::math::Vector2d> &_values,
    std::map<unsigned int, unsigned int> &_duplicates)
{
  if (this->dataPtr->texcoordIds.find(_id) != this->dataPtr->texcoordIds.end())
  {
    _values = this->dataPtr->texcoordIds[_id];
    _duplicates = this->dataPtr->texcoordDuplicateMap[_id];
    return;
  }

  int stride = 0;
  int texCount = 0;
  int totCount = 0;

  // Get the source element for the texture coordinates.
  TiXmlElement *xml = this->GetElementId("source", _id);
  if (!xml)
  {
    gzerr << "Unable to find tex coords[" << _id << "] in collada file\n";
    return;
  }

  // Get the array of float values. These are the raw values for the texture
  // coordinates.
  TiXmlElement *floatArrayXml = xml->FirstChildElement("float_array");
  if (!floatArrayXml || !floatArrayXml->GetText())
  {
    int count = 1;
    if (floatArrayXml && floatArrayXml->Attribute("count"))
    {
      try
      {
        count = boost::lexical_cast<int>(floatArrayXml->Attribute("count"));
      }
      catch(...)
      {
        // Do nothing. Messages are printed out below.
      }
    }

    if (count)
    {
      gzerr << "Normal source missing float_array element, or count is "
        << "invalid.\n";
    }
    else
    {
      gzlog << "Normal source has a float_array with a count of zero. "
        << "This is likely not desired\n";
    }

    return;
  }
  // Read in the total number of texture coordinate values
  else if (floatArrayXml->Attribute("count"))
    totCount = boost::lexical_cast<int>(floatArrayXml->Attribute("count"));
  else
  {
    gzerr << "<float_array> has no count attribute in texture coordinate "
          << "element with id[" << _id << "]\n";
    return;
  }

  // The technique_common holds an <accessor> element that indicates how to
  // parse the float array.
  xml = xml->FirstChildElement("technique_common");
  if (!xml)
  {
    gzerr << "Unable to find technique_common element for texture "
          << "coordinates with id[" << _id << "]\n";
    return;
  }

  // Get the accessor XML element.
  xml = xml->FirstChildElement("accessor");
  if (!xml)
  {
    gzerr << "Unable to find <accessor> as a child of <technique_common> "
          << "for texture coordinates with id[" << _id << "]\n";
    return;
  }

  // Read in the stride for the texture coordinate values. The stride
  // indicates the number of values in the float array the comprise
  // a complete texture coordinate.
  if (xml->Attribute("stride"))
    stride = boost::lexical_cast<int>(xml->Attribute("stride"));
  else
  {
    gzerr << "<accessor> has no stride attribute in texture coordinate element "
          << "with id[" << _id << "]\n";
    return;
  }

  // Read in the count of texture coordinates.
  if (xml->Attribute("count"))
    texCount = boost::lexical_cast<int>(xml->Attribute("count"));
  else
  {
    gzerr << "<accessor> has no count attribute in texture coordinate element "
          << "with id[" << _id << "]\n";
    return;
  }

  // \TODO This is a good a GZ_ASSERT
  // The total number of texture values should equal the stride multiplied
  // by the number of texture coordinates.
  if (texCount * stride != totCount)
  {
    gzerr << "Error reading texture coordinates. Coordinate counts in element "
             "with id[" << _id << "] do not add up correctly\n";
    return;
  }

  // Nothing to read. Don't print a warning because the collada file is
  // correct.
  if (totCount == 0)
    return;

  boost::unordered_map<ignition::math::Vector2d,
    unsigned int, Vector2dHash> unique;

  // Read the raw texture values, and split them on spaces.
  std::string valueStr = floatArrayXml->GetText();
  std::vector<std::string> values;
  boost::split(values, valueStr, boost::is_any_of(" "));

  // Read in all the texture coordinates.
  for (int i = 0; i < totCount; i += stride)
  {
    // We only handle 2D texture coordinates right now.
    ignition::math::Vector2d vec(boost::lexical_cast<double>(values[i]),
          1.0 - boost::lexical_cast<double>(values[i+1]));
    _values.push_back(vec);

    // create a map of duplicate indices
    if (unique.find(vec) != unique.end())
    {
      _duplicates[_values.size()-1] = unique[vec];
    }
    else
      unique[vec] = _values.size()-1;
  }

  this->dataPtr->texcoordDuplicateMap[_id] = _duplicates;
  this->dataPtr->texcoordIds[_id] = _values;
}

/////////////////////////////////////////////////
Material *ColladaLoader::LoadMaterial(const std::string &_name)
{
  if (this->dataPtr->materialIds.find(_name)
      != this->dataPtr->materialIds.end())
  {
    return this->dataPtr->materialIds[_name];
  }

  TiXmlElement *matXml = this->GetElementId("material", _name);
  if (!matXml || !matXml->FirstChildElement("instance_effect"))
    return nullptr;

  Material *mat = new Material();
  std::string effectName =
    matXml->FirstChildElement("instance_effect")->Attribute("url");
  TiXmlElement *effectXml = this->GetElementId("effect", effectName);

  TiXmlElement *commonXml = effectXml->FirstChildElement("profile_COMMON");
  if (commonXml)
  {
    TiXmlElement *techniqueXml = commonXml->FirstChildElement("technique");
    TiXmlElement *lambertXml = techniqueXml->FirstChildElement("lambert");

    TiXmlElement *phongXml = techniqueXml->FirstChildElement("phong");
    TiXmlElement *blinnXml = techniqueXml->FirstChildElement("blinn");
    if (lambertXml)
    {
      this->LoadColorOrTexture(lambertXml, "ambient", mat);
      this->LoadColorOrTexture(lambertXml, "emission", mat);
      this->LoadColorOrTexture(lambertXml, "diffuse", mat);
      // order matters: transparency needs to be loaded before transparent.
      if (lambertXml->FirstChildElement("transparency"))
      {
        mat->SetTransparency(
            this->LoadFloat(lambertXml->FirstChildElement("transparency")));
      }

      if (lambertXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = lambertXml->FirstChildElement("transparent");
        this->LoadTransparent(transXml, mat);
      }
      else
      {
        // no <transparent> tag, revert to zero transparency
        mat->SetTransparency(0.0);
      }
    }
    else if (phongXml)
    {
      this->LoadColorOrTexture(phongXml, "ambient", mat);
      this->LoadColorOrTexture(phongXml, "emission", mat);
      this->LoadColorOrTexture(phongXml, "specular", mat);
      this->LoadColorOrTexture(phongXml, "diffuse", mat);
      if (phongXml->FirstChildElement("shininess"))
        mat->SetShininess(
            this->LoadFloat(phongXml->FirstChildElement("shininess")));

      // order matters: transparency needs to be loaded before transparent
      if (phongXml->FirstChildElement("transparency"))
        mat->SetTransparency(
            this->LoadFloat(phongXml->FirstChildElement("transparency")));
      if (phongXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = phongXml->FirstChildElement("transparent");
        this->LoadTransparent(transXml, mat);
      }
      else
      {
        // no <transparent> tag, revert to zero transparency
        mat->SetTransparency(0.0);
      }
    }
    else if (blinnXml)
    {
      this->LoadColorOrTexture(blinnXml, "ambient", mat);
      this->LoadColorOrTexture(blinnXml, "emission", mat);
      this->LoadColorOrTexture(blinnXml, "specular", mat);
      this->LoadColorOrTexture(blinnXml, "diffuse", mat);
      if (blinnXml->FirstChildElement("shininess"))
        mat->SetShininess(
            this->LoadFloat(blinnXml->FirstChildElement("shininess")));

      // order matters: transparency needs to be loaded before transparent
      if (blinnXml->FirstChildElement("transparency"))
        mat->SetTransparency(
            this->LoadFloat(blinnXml->FirstChildElement("transparency")));
      if (blinnXml->FirstChildElement("transparent"))
      {
        TiXmlElement *transXml = blinnXml->FirstChildElement("transparent");
        this->LoadTransparent(transXml, mat);
      }
      else
      {
        // no <transparent> tag, revert to zero transparency
        mat->SetTransparency(0.0);
      }
    }
  }

  TiXmlElement *glslXml = effectXml->FirstChildElement("profile_GLSL");
  if (glslXml)
    gzerr << "profile_GLSL unsupported\n";

  TiXmlElement *cgXml = effectXml->FirstChildElement("profile_CG");
  if (cgXml)
    gzerr << "profile_CG unsupported\n";

  this->dataPtr->materialIds[_name] = mat;

  return mat;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadColorOrTexture(TiXmlElement *_elem,
    const std::string &_type, Material *_mat)
{
  if (!_elem || !_elem->FirstChildElement(_type))
    return;

  TiXmlElement *typeElem = _elem->FirstChildElement(_type);

  if (typeElem->FirstChildElement("color"))
  {
    std::string colorStr = typeElem->FirstChildElement("color")->GetText();
    auto color = boost::lexical_cast<ignition::math::Color>(colorStr);
    if (_type == "diffuse")
      _mat->SetDiffuse(color);
    else if (_type == "ambient")
      _mat->SetAmbient(color);
    else if (_type == "emission")
      _mat->SetEmissive(color);
    else if (_type == "specular")
      _mat->SetSpecular(color);
  }
  else if (typeElem->FirstChildElement("texture"))
  {
    if (_type == "ambient")
    {
      gzwarn << "ambient texture not supported" << std::endl;
      return;
    }
    if (_type == "emission")
    {
      gzwarn << "emission texture not supported" << std::endl;
      return;
    }
    if (_type == "specular")
    {
      gzwarn << "specular texture not supported" << std::endl;
      return;
    }

    // gazebo rendering pipeline doesn't respect the blend mode, here we set
    // the diffuse to full white as a workaround.
    if (_type == "diffuse"
        && _mat->GetBlendMode() == Material::BlendMode::REPLACE)
    {
      _mat->SetDiffuse(ignition::math::Color(1, 1, 1, 1));
    }

    _mat->SetLighting(true);
    TiXmlElement *imageXml = nullptr;
    std::string textureName =
      typeElem->FirstChildElement("texture")->Attribute("texture");
    TiXmlElement *textureXml = this->GetElementId("newparam", textureName);
    if (textureXml)
    {
      if (std::string(textureXml->Value()) == "image")
      {
        imageXml = textureXml;
      }
      else
      {
        TiXmlElement *sampler = textureXml->FirstChildElement("sampler2D");
        if (sampler)
        {
          std::string sourceName =
            sampler->FirstChildElement("source")->GetText();
          TiXmlElement *sourceXml = this->GetElementId("newparam", sourceName);
          if (sourceXml)
          {
            TiXmlElement *surfaceXml = sourceXml->FirstChildElement("surface");
            if (surfaceXml && surfaceXml->FirstChildElement("init_from"))
            {
              imageXml = this->GetElementId("image",
                  surfaceXml->FirstChildElement("init_from")->GetText());
            }
          }
        }
      }
    }
    else
    {
      imageXml = this->GetElementId("image", textureName);
    }

    if (imageXml && imageXml->FirstChildElement("init_from"))
    {
      std::string imgFile =
        imageXml->FirstChildElement("init_from")->GetText();

      auto curl = curl_easy_init();
      if (curl)
      {
        imgFile = std::string(curl_easy_unescape(curl, imgFile.c_str(), 0,
            nullptr));
      }
      else
      {
        gzerr << "Can't unescape file, failed to initialize curl" << std::endl;
      }

      _mat->SetTextureImage(imgFile, this->dataPtr->path);
    }
  }
}

/////////////////////////////////////////////////
void ColladaLoader::LoadPolylist(TiXmlElement *_polylistXml,
    const ignition::math::Matrix4d &_transform,
    Mesh *_mesh)
{
  // This function parses polylist types in collada into
  // a set of triangle meshes.  The assumption is that
  // each polylist polygon is convex, and we do decomposion
  // by anchoring each triangle about vertex 0 or each polygon
  SubMesh *subMesh = new SubMesh;
  subMesh->SetName(this->dataPtr->currentNodeName);
  bool combinedVertNorms = false;

  subMesh->SetPrimitiveType(SubMesh::TRIANGLES);

  if (_polylistXml->Attribute("material"))
  {
    std::map<std::string, std::string>::iterator iter;
    std::string matStr = _polylistXml->Attribute("material");

    int matIndex = -1;
    iter = this->dataPtr->materialMap.find(matStr);
    if (iter != this->dataPtr->materialMap.end())
      matStr = iter->second;

    common::Material *mat = this->LoadMaterial(matStr);

    matIndex = _mesh->GetMaterialIndex(mat);
    if (matIndex < 0)
      matIndex = _mesh->AddMaterial(mat);

    if (matIndex < 0)
      gzwarn << "Unable to add material[" << matStr << "]\n";
    else
      subMesh->SetMaterialIndex(matIndex);
  }

  TiXmlElement *polylistInputXml = _polylistXml->FirstChildElement("input");

  std::vector<ignition::math::Vector3d> verts;
  std::vector<ignition::math::Vector3d> norms;
  std::vector<ignition::math::Vector2d> texcoords;

  const unsigned int VERTEX = 0;
  const unsigned int NORMAL = 1;
  const unsigned int TEXCOORD = 2;
  unsigned int otherSemantics = TEXCOORD + 1;

  // look up table of position/normal/texcoord duplicate indices
  std::map<unsigned int, unsigned int> texDupMap;
  std::map<unsigned int, unsigned int> normalDupMap;
  std::map<unsigned int, unsigned int> positionDupMap;

  ignition::math::Matrix4d bindShapeMat(ignition::math::Matrix4d::Identity);
  if (_mesh->HasSkeleton())
    bindShapeMat = _mesh->GetSkeleton()->BindShapeTransform();

  // read input elements. A vector of int is used because there can be
  // multiple TEXCOORD inputs.
  std::map<const unsigned int, std::set<int>> inputs;
  unsigned int inputSize = 0;
  while (polylistInputXml)
  {
    std::string semantic = polylistInputXml->Attribute("semantic");
    std::string source = polylistInputXml->Attribute("source");
    std::string offset = polylistInputXml->Attribute("offset");
    if (semantic == "VERTEX")
    {
      unsigned int count = norms.size();
      this->LoadVertices(source, _transform, verts, norms,
          positionDupMap, normalDupMap);
      if (norms.size() > count)
        combinedVertNorms = true;
      inputs[VERTEX].insert(ignition::math::parseInt(offset));
    }
    else if (semantic == "NORMAL")
    {
      this->LoadNormals(source, _transform, norms, normalDupMap);
      combinedVertNorms = false;
      inputs[NORMAL].insert(ignition::math::parseInt(offset));
    }
    else if (semantic == "TEXCOORD")
    {
      this->LoadTexCoords(source, texcoords, texDupMap);
      inputs[TEXCOORD].insert(ignition::math::parseInt(offset));
    }
    else
    {
      inputs[otherSemantics++].insert(ignition::math::parseInt(offset));
      gzwarn << "Polylist input semantic: '" << semantic << "' is currently"
          << " not supported" << std::endl;
    }

    polylistInputXml = polylistInputXml->NextSiblingElement("input");
  }

  std::set<int> totalInputs;
  for (const auto &input : inputs)
  {
    totalInputs.insert(input.second.begin(), input.second.end());
  }
  inputSize += totalInputs.size();

  // read vcount
  // break poly into triangles
  // if vcount >= 4, anchor around 0 (note this is bad for concave elements)
  //   e.g. if vcount = 4, break into triangle 1: [0,1,2], triangle 2: [0,2,3]
  std::vector<std::string> vcountStrs;
  TiXmlElement *vcountXml = _polylistXml->FirstChildElement("vcount");
  std::string vcountStr = vcountXml->GetText();
  boost::split(vcountStrs, vcountStr, boost::is_any_of(" \t"));
  std::vector<int> vcounts;
  for (unsigned int j = 0; j < vcountStrs.size(); ++j)
    vcounts.push_back(ignition::math::parseInt(vcountStrs[j]));

  // read p
  TiXmlElement *pXml = _polylistXml->FirstChildElement("p");
  std::string pStr = pXml->GetText();

  // vertexIndexMap is a map of collada vertex index to Gazebo submesh vertex
  // indices, used for identifying vertices that can be shared.
  std::map<unsigned int, std::vector<GeometryIndices> > vertexIndexMap;
  unsigned int *values = new unsigned int[inputSize];
  memset(values, 0, inputSize);

  std::vector<std::string> strs;
  boost::split(strs, pStr, boost::is_any_of(" \t"));
  std::vector<std::string>::iterator strsIter = strs.begin();
  for (unsigned int l = 0; l < vcounts.size(); ++l)
  {
    // put us at the beginning of the polygon list
    if (l > 0)
      strsIter += inputSize*vcounts[l-1];

    for (unsigned int k = 2; k < (unsigned int)vcounts[l]; ++k)
    {
      // if vcounts[l] = 5, then read 0,1,2, then 0,2,3, 0,3,4,...
      // here k = the last number in the series
      // j is the triangle loop
      for (unsigned int j = 0; j < 3; ++j)
      {
        // break polygon into triangles
        unsigned int triangle_index;

        if (j == 0)
          triangle_index = 0;
        if (j == 1)
          triangle_index = (k-1)*inputSize;
        if (j == 2)
          triangle_index = (k)*inputSize;

        for (unsigned int i = 0; i < inputSize; ++i)
        {
          values[i] = ignition::math::parseInt(strsIter[triangle_index+i]);
          /*gzerr << "debug parsing "
                << " poly-i[" << l
                << "] tri-end-index[" << k
                << "] tri-vertex-i[" << j
                << "] triangle[" << triangle_index
                << "] input[" << i
                << "] value[" << values[i]
                << "]\n"; */
        }


        unsigned int daeVertIndex = 0;
        bool addIndex = inputs[VERTEX].empty();

        // find a set of vertex/normal/texcoord that can be reused
        // only do this if the mesh has vertices
        if (!inputs[VERTEX].empty())
        {
          // Get the vertex position index value. If it is a duplicate then use
          // the existing index instead
          daeVertIndex = values[*inputs[VERTEX].begin()];
          if (positionDupMap.find(daeVertIndex) != positionDupMap.end())
            daeVertIndex = positionDupMap[daeVertIndex];

          // if the vertex index has not been previously added then just add it.
          if (vertexIndexMap.find(daeVertIndex) == vertexIndexMap.end())
          {
            addIndex = true;
          }
          else
          {
            // if the vertex index was previously added, check to see if it has
            // the same normal and texcoord index values
            bool toDuplicate = true;
            unsigned int reuseIndex = 0;
            std::vector<GeometryIndices> inputValues =
                vertexIndexMap[daeVertIndex];

            for (unsigned int i = 0; i < inputValues.size(); ++i)
            {
              GeometryIndices iv = inputValues[i];
              bool normEqual = false;
              bool texEqual = false;

              if (!inputs[NORMAL].empty())
              {
                // Get the vertex normal index value. If the normal is a
                // duplicate then reset the index to the first instance of the
                // duplicated position
                unsigned int remappedNormalIndex =
                  values[*inputs[NORMAL].begin()];
                if (normalDupMap.find(remappedNormalIndex)
                    != normalDupMap.end())
                 {
                  remappedNormalIndex = normalDupMap[remappedNormalIndex];
                 }

                if (iv.normalIndex == remappedNormalIndex)
                  normEqual = true;
              }

              if (!inputs[TEXCOORD].empty())
              {
                // \todo: Add support for multiple texture maps to SubMesh.
                // Here we are only using the first texture coordinates, when
                // multiple could have been specified. See Gazebo issue #532.

                // Get the vertex texcoord index value. If the texcoord is a
                // duplicate then reset the index to the first instance of the
                // duplicated texcoord
                unsigned int remappedTexcoordIndex =
                  values[*inputs[TEXCOORD].begin()];

                if (texDupMap.find(remappedTexcoordIndex) != texDupMap.end())
                  remappedTexcoordIndex = texDupMap[remappedTexcoordIndex];

                texEqual = iv.texcoordIndex == remappedTexcoordIndex;
              }

              // if the vertex has matching normal and texcoord index values
              // then the vertex can be reused.
              if ((inputs[NORMAL].empty() || normEqual) &&
                  (inputs[TEXCOORD].empty() || texEqual))
              {
                // found a vertex that can be shared.
                toDuplicate = false;
                reuseIndex = iv.mappedIndex;
                subMesh->AddIndex(reuseIndex);
                break;
              }
            }
            addIndex = toDuplicate;
          }
        }

        // if the vertex index is new or can not be shared then add it
        if (addIndex)
        {
          GeometryIndices input;
          if (!inputs[VERTEX].empty())
          {
            subMesh->AddVertex(verts[daeVertIndex]);
            unsigned int newVertIndex = subMesh->GetVertexCount()-1;
            subMesh->AddIndex(newVertIndex);
            if (combinedVertNorms)
              subMesh->AddNormal(norms[daeVertIndex]);
            if (_mesh->HasSkeleton())
            {
              subMesh->SetVertex(newVertIndex, bindShapeMat *
                  subMesh->Vertex(newVertIndex));
              Skeleton *skel = _mesh->GetSkeleton();
              for (unsigned int i = 0;
                  i < skel->GetNumVertNodeWeights(daeVertIndex); ++i)
              {
                std::pair<std::string, double> node_weight =
                  skel->GetVertNodeWeight(daeVertIndex, i);
                SkeletonNode *node =
                    _mesh->GetSkeleton()->GetNodeByName(node_weight.first);
                subMesh->AddNodeAssignment(subMesh->GetVertexCount()-1,
                                node->GetHandle(), node_weight.second);
              }
            }
            input.vertexIndex = daeVertIndex;
            input.mappedIndex = newVertIndex;
          }
          if (!inputs[NORMAL].empty())
          {
            unsigned int inputRemappedNormalIndex =
              values[*inputs[NORMAL].begin()];
            if (normalDupMap.find(inputRemappedNormalIndex)
                != normalDupMap.end())
              inputRemappedNormalIndex = normalDupMap[inputRemappedNormalIndex];
            subMesh->AddNormal(norms[inputRemappedNormalIndex]);
            input.normalIndex = inputRemappedNormalIndex;
          }

          if (!inputs[TEXCOORD].empty())
          {
            // \todo: Add support for multiple texture maps to SubMesh.
            // Here we are only using the first texture coordinates, when
            // multiple could have been specified.
            unsigned int inputRemappedTexcoordIndex =
              values[*inputs[TEXCOORD].begin()];

            if (texDupMap.find(inputRemappedTexcoordIndex) != texDupMap.end())
            {
              inputRemappedTexcoordIndex =
                  texDupMap[inputRemappedTexcoordIndex];
            }
            subMesh->AddTexCoord(texcoords[inputRemappedTexcoordIndex].X(),
                texcoords[inputRemappedTexcoordIndex].Y());
            input.texcoordIndex = inputRemappedTexcoordIndex;
          }

          // add the new gazebo submesh vertex index to the map
          if (!inputs[VERTEX].empty())
          {
            std::vector<GeometryIndices> inputValues;
            inputValues.push_back(input);
            vertexIndexMap[daeVertIndex] = inputValues;
          }
        }
      }
    }
  }
  delete [] values;

  _mesh->AddSubMesh(subMesh);
}

/////////////////////////////////////////////////
void ColladaLoader::LoadTriangles(TiXmlElement *_trianglesXml,
                                  const ignition::math::Matrix4d &_transform,
                                  Mesh *_mesh)
{
  std::unique_ptr<SubMesh> subMesh(new SubMesh);
  subMesh->SetName(this->dataPtr->currentNodeName);
  bool combinedVertNorms = false;

  subMesh->SetPrimitiveType(SubMesh::TRIANGLES);

  if (_trianglesXml->Attribute("material"))
  {
    std::map<std::string, std::string>::iterator iter;
    std::string matStr = _trianglesXml->Attribute("material");

    int matIndex = -1;
    iter = this->dataPtr->materialMap.find(matStr);
    if (iter != this->dataPtr->materialMap.end())
      matStr = iter->second;

    common::Material *mat = this->LoadMaterial(matStr);
    matIndex = _mesh->GetMaterialIndex(mat);
    if (matIndex < 0)
      matIndex = _mesh->AddMaterial(mat);

    if (matIndex < 0)
      gzwarn << "Unable to add material[" << matStr << "]\n";
    else
      subMesh->SetMaterialIndex(matIndex);
  }

  TiXmlElement *trianglesInputXml = _trianglesXml->FirstChildElement("input");

  std::vector<ignition::math::Vector3d> verts;
  std::vector<ignition::math::Vector3d> norms;
  std::vector<ignition::math::Vector2d> texcoords;

  const unsigned int VERTEX = 0;
  const unsigned int NORMAL = 1;
  const unsigned int TEXCOORD = 2;
  unsigned int otherSemantics = TEXCOORD + 1;
  bool hasVertices = false;
  bool hasNormals = false;
  bool hasTexcoords = false;
  unsigned int offsetSize = 0;

  // read input elements. A vector of int is used because there can be
  // multiple TEXCOORD inputs.
  std::map<const unsigned int, std::set<int>> inputs;

  // look up table of position/normal/texcoord duplicate indices
  std::map<unsigned int, unsigned int> texDupMap;
  std::map<unsigned int, unsigned int> normalDupMap;
  std::map<unsigned int, unsigned int> positionDupMap;

  while (trianglesInputXml)
  {
    std::string semantic = trianglesInputXml->Attribute("semantic");
    std::string source = trianglesInputXml->Attribute("source");
    std::string offset = trianglesInputXml->Attribute("offset");

    if (semantic == "VERTEX")
    {
      unsigned int count = norms.size();
      this->LoadVertices(source, _transform, verts, norms,
          positionDupMap, normalDupMap);
      if (norms.size() > count)
        combinedVertNorms = true;
      inputs[VERTEX].insert(ignition::math::parseInt(offset));
      hasVertices = true;
    }
    else if (semantic == "NORMAL")
    {
      this->LoadNormals(source, _transform, norms, normalDupMap);
      combinedVertNorms = false;
      inputs[NORMAL].insert(ignition::math::parseInt(offset));
      hasNormals = true;
    }
    else if (semantic == "TEXCOORD")
    {
      // we currently only support one set of UVs
      this->LoadTexCoords(source, texcoords, texDupMap);
      inputs[TEXCOORD].insert(ignition::math::parseInt(offset));
      hasTexcoords = true;
    }
    else
    {
      inputs[otherSemantics++].insert(ignition::math::parseInt(offset));
      gzwarn << "Triangle input semantic: '" << semantic << "' is currently"
          << " not supported" << std::endl;
    }
    trianglesInputXml = trianglesInputXml->NextSiblingElement("input");
  }

  for (const auto &input : inputs)
    offsetSize += input.second.size();

  TiXmlElement *pXml = _trianglesXml->FirstChildElement("p");
  if (!pXml || !pXml->GetText())
  {
    int count = 1;
    if (_trianglesXml->Attribute("count"))
    {
      try
      {
        count = boost::lexical_cast<int>(_trianglesXml->Attribute("count"));
      }
      catch(...)
      {
        // Do nothing. Messages are printed out below.
      }
    }

    // It's possible that the triangle count is zero. In this case, we
    // should not output an error message
    if (count)
    {
      gzerr << "Collada file[" << this->dataPtr->filename
        << "] is invalid. Loading what we can...\n";
    }
    else
    {
      gzlog << "Triangle input has a count of zero. "
        << "This is likely not desired\n";
    }

    return;
  }
  std::string pStr = pXml->GetText();

  // Collada format allows normals and texcoords to have their own set of
  // indices for more efficient storage of data but opengl only supports one
  // index buffer. So we need to reorder normals/texcoord to match the vertex
  // index and duplicate any vertices that have the same index but different
  // normal/texcoord.

  // vertexIndexMap is a map of collada vertex index to Gazebo submesh vertex
  // indices, used for identifying vertices that can be shared.
  std::map<unsigned int, std::vector<GeometryIndices> > vertexIndexMap;

  std::vector<unsigned int> values(offsetSize);
  std::vector<std::string> strs;

  boost::split(strs, pStr, boost::is_any_of(" \t"));

  for (unsigned int j = 0; j < strs.size(); j += offsetSize)
  {
    for (unsigned int i = 0; i < offsetSize; ++i)
      values.at(i) = ignition::math::parseInt(strs[j+i]);

    unsigned int daeVertIndex = 0;
    bool addIndex = !hasVertices;

    // find a set of vertex/normal/texcoord that can be reused
    // only do this if the mesh has vertices
    if (hasVertices)
    {
      // Get the vertex position index value. If the position is a duplicate
      // then reset the index to the first instance of the duplicated position
      daeVertIndex = values.at(*inputs[VERTEX].begin());
      if (positionDupMap.find(daeVertIndex) != positionDupMap.end())
        daeVertIndex = positionDupMap[daeVertIndex];

      // if the vertex index has not been previously added then just add it.
      if (vertexIndexMap.find(daeVertIndex) == vertexIndexMap.end())
      {
        addIndex = true;
      }
      else
      {
        // if the vertex index was previously added, check to see if it has the
        // same normal and texcoord index values
        bool toDuplicate = true;
        unsigned int reuseIndex = 0;
        std::vector<GeometryIndices> inputValues = vertexIndexMap[daeVertIndex];

        for (unsigned int i = 0; i < inputValues.size(); ++i)
        {
          GeometryIndices iv = inputValues[i];
          bool normEqual = false;
          bool texEqual = false;
          if (hasNormals)
          {
            // Get the vertex normal index value. If the normal is a duplicate
            // then reset the index to the first instance of the duplicated
            // position
            unsigned int remappedNormalIndex =
              values.at(*inputs[NORMAL].begin());
            if (normalDupMap.find(remappedNormalIndex) != normalDupMap.end())
              remappedNormalIndex = normalDupMap[remappedNormalIndex];

            if (iv.normalIndex == remappedNormalIndex)
              normEqual = true;
          }
          if (hasTexcoords)
          {
            // Get the vertex texcoord index value. If the texcoord is a
            // duplicate then reset the index to the first instance of the
            // duplicated texcoord
            unsigned int remappedTexcoordIndex =
                values.at(*inputs[TEXCOORD].begin());
            if (texDupMap.find(remappedTexcoordIndex) != texDupMap.end())
              remappedTexcoordIndex = texDupMap[remappedTexcoordIndex];

            if (iv.texcoordIndex == remappedTexcoordIndex)
              texEqual = true;
          }

          // if the vertex has matching normal and texcoord index values then
          // the vertex can be reused.
          if ((!hasNormals || normEqual) && (!hasTexcoords || texEqual))
          {
            // found a vertex that can be shared.
            toDuplicate = false;
            reuseIndex = iv.mappedIndex;
            subMesh->AddIndex(reuseIndex);
            break;
          }
        }
        addIndex = toDuplicate;
      }
    }

    // if the vertex index is new or can not be shared then add it
    if (addIndex)
    {
      GeometryIndices input;
      if (hasVertices)
      {
        subMesh->AddVertex(verts[daeVertIndex]);
        unsigned int newVertIndex = subMesh->GetVertexCount()-1;
        subMesh->AddIndex(newVertIndex);

        if (combinedVertNorms)
          subMesh->AddNormal(norms[daeVertIndex]);
        if (_mesh->HasSkeleton())
        {
          Skeleton *skel = _mesh->GetSkeleton();
          for (unsigned int i = 0;
              i < skel->GetNumVertNodeWeights(daeVertIndex); ++i)
          {
            std::pair<std::string, double> node_weight =
              skel->GetVertNodeWeight(daeVertIndex, i);
            SkeletonNode *node =
                _mesh->GetSkeleton()->GetNodeByName(node_weight.first);
            subMesh->AddNodeAssignment(subMesh->GetVertexCount()-1,
                            node->GetHandle(), node_weight.second);
          }
        }
        input.vertexIndex = daeVertIndex;
        input.mappedIndex = newVertIndex;
      }
      if (hasNormals)
      {
        unsigned int inputRemappedNormalIndex =
          values.at(*inputs[NORMAL].begin());
        if (normalDupMap.find(inputRemappedNormalIndex) != normalDupMap.end())
          inputRemappedNormalIndex = normalDupMap[inputRemappedNormalIndex];
        subMesh->AddNormal(norms[inputRemappedNormalIndex]);
        input.normalIndex = inputRemappedNormalIndex;
      }
      if (hasTexcoords)
      {
        unsigned int inputRemappedTexcoordIndex =
            values.at(*inputs[TEXCOORD].begin());
        if (texDupMap.find(inputRemappedTexcoordIndex) != texDupMap.end())
          inputRemappedTexcoordIndex = texDupMap[inputRemappedTexcoordIndex];
        subMesh->AddTexCoord(texcoords[inputRemappedTexcoordIndex].X(),
            texcoords[inputRemappedTexcoordIndex].Y());
        input.texcoordIndex = inputRemappedTexcoordIndex;
      }

      // add the new gazebo submesh vertex index to the map
      if (hasVertices)
      {
        std::vector<GeometryIndices> inputValues;
        inputValues.push_back(input);
        vertexIndexMap[daeVertIndex] = inputValues;
      }
    }
  }

  _mesh->AddSubMesh(subMesh.release());
}

/////////////////////////////////////////////////
void ColladaLoader::LoadLines(TiXmlElement *_xml,
    const ignition::math::Matrix4d &_transform,
    Mesh *_mesh)
{
  SubMesh *subMesh = new SubMesh;
  subMesh->SetName(this->dataPtr->currentNodeName);
  subMesh->SetPrimitiveType(SubMesh::LINES);

  TiXmlElement *inputXml = _xml->FirstChildElement("input");
  // std::string semantic = inputXml->Attribute("semantic");
  std::string source = inputXml->Attribute("source");

  std::vector<ignition::math::Vector3d> verts;
  std::vector<ignition::math::Vector3d> norms;
  this->LoadVertices(source, _transform, verts, norms);

  TiXmlElement *pXml = _xml->FirstChildElement("p");
  std::string pStr = pXml->GetText();
  std::istringstream iss(pStr);

  do
  {
    int a, b;
    iss >> a >> b;

    if (!iss)
      break;
    subMesh->AddVertex(verts[a]);
    subMesh->AddIndex(subMesh->GetVertexCount() - 1);
    subMesh->AddVertex(verts[b]);
    subMesh->AddIndex(subMesh->GetVertexCount() - 1);
  } while (iss);

  _mesh->AddSubMesh(subMesh);
}

/////////////////////////////////////////////////
float ColladaLoader::LoadFloat(TiXmlElement *_elem)
{
  float value = 0;

  if (_elem->FirstChildElement("float"))
  {
    value =
      ignition::math::parseFloat(_elem->FirstChildElement("float")->GetText());
  }

  return value;
}

/////////////////////////////////////////////////
void ColladaLoader::LoadTransparent(TiXmlElement *_elem, Material *_mat)
{
  const char *opaqueCStr = _elem->Attribute("opaque");
  if (!opaqueCStr)
  {
    // no opaque mode, revert transparency to 0.0
    _mat->SetTransparency(0.0);
    return;
  }

  // TODO: Handle transparent textures
  if (_elem->FirstChildElement("texture"))
  {
    gzwarn << "texture based transparency not supported" << std::endl;
    _mat->SetTransparency(0.0);
  }
  else if (_elem->FirstChildElement("color"))
  {
    const char *colorCStr = _elem->FirstChildElement("color")->GetText();
    if (!colorCStr)
    {
      gzerr << "No color string\n";
      return;
    }

    std::string opaqueStr = opaqueCStr;
    std::string colorStr = colorCStr;
    auto color = boost::lexical_cast<ignition::math::Color>(colorStr);

    // src is the texel value and dst is the existing pixel value
    double srcFactor = 0;
    double dstFactor = 0;

    // Calculate alpha based on opaque mode.
    // Equations are extracted from collada spec
    // Make sure to update the final transparency value
    // final mat transparency = 1 - srcFactor = dstFactor
    if (opaqueStr == "RGB_ZERO")
    {
      // Lunimance based on ISO/CIE color standards ITU-R BT.709-4
      float luminance = 0.212671 * color.R() +
                        0.715160 * color.G() +
                        0.072169 * color.B();
      // result.a = fb.a * (lumiance(transparent.rgb) * transparency) + mat.a *
      // (1.0f - luminance(transparent.rgb) * transparency)
      // where fb corresponds to the framebuffer (existing pixel) and
      // mat corresponds to material before transparency (texel)
      dstFactor = luminance * _mat->GetTransparency();
      srcFactor = 1.0 - luminance * _mat->GetTransparency();
      _mat->SetTransparency(dstFactor);
    }
    else if (opaqueStr == "RGB_ONE")
    {
      // Lunimance based on ISO/CIE color standards ITU-R BT.709-4
      float luminance = 0.212671 * color.R() +
                        0.715160 * color.G() +
                        0.072169 * color.B();

      // result.a = fb.a * (1.0f - lumiance(transparent.rgb) * transparency) +
      // mat.a * (luminance(transparent.rgb) * transparency)
      // where fb corresponds to the framebuffer (existing pixel) and
      // mat corresponds to material before transparency (texel)
      dstFactor = 1.0 - luminance * _mat->GetTransparency();
      srcFactor = luminance * _mat->GetTransparency();
      _mat->SetTransparency(dstFactor);
    }
    else if (opaqueStr == "A_ONE")
    {
      // result.a = fb.a * (1.0f - transparent.a * transparency) + mat.a *
      // (transparent.a * transparency)
      // where fb corresponds to the framebuffer (existing pixel) and
      // mat corresponds to material before transparency (texel)
      dstFactor = 1.0 - color.A() * _mat->GetTransparency();
      srcFactor = color.A() * _mat->GetTransparency();
      _mat->SetTransparency(dstFactor);
    }
    else if (opaqueStr == "A_ZERO")
    {
      // result.a = fb.a * (transparent.a * transparency) + mat.a *
      // (1.0f - transparent.a * transparency)
      // where fb corresponds to the framebuffer (existing pixel) and
      // mat corresponds to material before transparency (texel)
      dstFactor = color.A() * _mat->GetTransparency();
      srcFactor = 1.0 - color.A() * _mat->GetTransparency();
      _mat->SetTransparency(dstFactor);
    }

    _mat->SetBlendFactors(srcFactor, dstFactor);
  }
}

/////////////////////////////////////////////////
void ColladaLoader::MergeSkeleton(Skeleton *_skeleton, SkeletonNode *_mergeNode)
{
  if (_skeleton->GetNodeById(_mergeNode->GetId()))
    return;

  SkeletonNode *currentRoot = _skeleton->GetRootNode();
  if (currentRoot->GetId() == _mergeNode->GetId())
    return;

  if (_mergeNode->GetChildById(currentRoot->GetId()))
  {
    _skeleton->SetRootNode(_mergeNode);
    return;
  }

  SkeletonNode *dummyRoot = nullptr;
  if (currentRoot->GetId() == "gazebo-dummy-root")
    dummyRoot = currentRoot;
  else
  {
    dummyRoot =
        new SkeletonNode(nullptr, "gazebo-dummy-root", "gazebo-dummy-root");
  }
  if (dummyRoot != currentRoot)
  {
    dummyRoot->AddChild(currentRoot);
    currentRoot->SetParent(dummyRoot);
  }
  dummyRoot->AddChild(_mergeNode);
  _mergeNode->SetParent(dummyRoot);
  dummyRoot->SetTransform(ignition::math::Matrix4d::Identity);
  _skeleton->SetRootNode(dummyRoot);
}

/////////////////////////////////////////////////
void ColladaLoader::ApplyInvBindTransform(Skeleton *_skeleton)
{
  std::list<SkeletonNode*> queue;
  queue.push_back(_skeleton->GetRootNode());

  while (!queue.empty())
  {
    SkeletonNode *node = queue.front();
    if (node->HasInvBindTransform())
      node->SetModelTransform(node->InverseBindTransform().Inverse(), false);
    for (unsigned int i = 0; i < node->GetChildCount(); i++)
      queue.push_back(node->GetChild(i));
    queue.pop_front();
  }
}
