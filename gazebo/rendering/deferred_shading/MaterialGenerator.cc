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
#include <OgreMaterialSerializer.h>
#include <OgreMaterialManager.h>
#include <OgreStringConverter.h>
#include <OgreException.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>

#include "gazebo/rendering/deferred_shading/MaterialGenerator.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
MaterialGenerator::MaterialGenerator()
  : vsMask(0), fsMask(0), matMask(0), impl(0)
{
  this->impl = NULL;
}

/////////////////////////////////////////////////
MaterialGenerator::~MaterialGenerator()
{
  delete this->impl;
}

/////////////////////////////////////////////////
const Ogre::MaterialPtr &MaterialGenerator::GetMaterial(Perm _permutation)
{
  /// Check if material/shader permutation already was generated
  MaterialMap::iterator i = this->materials.find(_permutation);

  if (i != this->materials.end())
  {
    return i->second;
  }
  else
  {
    Ogre::MaterialPtr mat;

    Ogre::MaterialPtr templ = this->GetTemplateMaterial(_permutation & matMask);
    Ogre::GpuProgramPtr vertShader =
      this->GetVertexShader(_permutation & vsMask);
    Ogre::GpuProgramPtr fragShader =
      this->GetFragmentShader(_permutation & fsMask);

    /// Create material name
    Ogre::String name = materialBaseName +
      Ogre::StringConverter::toString(_permutation);

    /// Create material from template, and set shaders
    mat = templ->clone(name);
    Ogre::Technique *tech = mat->getTechnique(0);
    if (!this->schemeName.empty())
    {
      tech->setSchemeName(this->schemeName);
    }

    Ogre::Pass *pass = tech->getPass(0);
    pass->setVertexProgram(vertShader->getName());
    pass->setFragmentProgram(fragShader->getName());

    /*Ogre::MaterialSerializer serializer;
    serializer.queueForExport(mat, true);
    Ogre::String matString = serializer.getQueuedAsString();
    std::cout << matString << "\n";
    std::cout << "*************\n" << fragShader->getSource() << "******\n";
    */

    /// And store it
    this->materials[_permutation] = mat;
    return this->materials[_permutation];
  }
}

/////////////////////////////////////////////////
const Ogre::GpuProgramPtr &MaterialGenerator::GetVertexShader(Perm _permutation)
{
  ProgramMap::iterator i = this->vs.find(_permutation);

  if (i != this->vs.end())
    return i->second;
  else
  {
    /// Create it
    this->vs[_permutation] = this->impl->GenerateVertexShader(_permutation);
    return this->vs[_permutation];
  }
}

/////////////////////////////////////////////////
const Ogre::GpuProgramPtr &MaterialGenerator::GetFragmentShader(
    Perm _permutation)
{
  ProgramMap::iterator i = this->fs.find(_permutation);

  if (i != this->fs.end())
    return i->second;
  else
  {
    /// Create it
    this->fs[_permutation] = this->impl->GenerateFragmentShader(_permutation);
    return this->fs[_permutation];
  }
}

/////////////////////////////////////////////////
const Ogre::MaterialPtr &MaterialGenerator::GetTemplateMaterial(
    Perm _permutation)
{
  MaterialMap::iterator i = this->templateMat.find(_permutation);
  if (i != this->templateMat.end())
    return i->second;
  else
  {
    if (this->impl == NULL)
      printf("ERROR NULL\n");
    /// Create it
    this->templateMat[_permutation] =
      this->impl->GenerateTemplateMaterial(_permutation);
    return this->templateMat[_permutation];
  }
}

/////////////////////////////////////////////////
MaterialGenerator::Impl::~Impl()
{
}
