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
/**
 * File: MovableText.cpp
 *
 * description: This create create a billboarding object that display a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 * @update  2007 by independentCreations see independentCreations@gmail.com
 */

#include <boost/thread/recursive_mutex.hpp>

#include "gazebo/common/common.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/rendering/MovableText.hh"

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
MovableText::MovableText()
    : camera(NULL),
    renderWindow(NULL) ,
    font(NULL) ,
    viewportAspectCoef(0.75),
    spaceWidth(0) ,
    updateColors(true) ,
    vertAlign(V_BELOW) ,
    horizAlign(H_LEFT) ,
    onTop(false) ,
    baseline(0.0)
{
  this->renderOp.vertexData = NULL;

  this->dirty = true;
  this->mutex = new boost::recursive_mutex();
  this->aabb = new Ogre::AxisAlignedBox;
}

//////////////////////////////////////////////////
MovableText::~MovableText()
{
  delete this->renderOp.vertexData;
  delete this->mutex;
  delete this->aabb;
}

//////////////////////////////////////////////////
void MovableText::Load(const std::string &name_,
                        const std::string &text_,
                        const std::string &fontName_,
                        float charHeight_,
                        const common::Color &color_)
{
  {
    boost::recursive_mutex::scoped_lock lock(*this->mutex);

    this->text = text_;
    this->color = color_;
    this->fontName = fontName_;
    this->charHeight = charHeight_;
    this->mName = name_;

    if (this->mName == "")
      throw Ogre::Exception(Ogre::Exception::ERR_INVALIDPARAMS,
          "Trying to create MovableText without name",
          "MovableText::MovableText");

    if (this->text == "")
      throw Ogre::Exception(Ogre::Exception::ERR_INVALIDPARAMS,
          "Trying to create MovableText without text",
          "MovableText::MovableText");

    this->dirty = true;
  }

  this->SetFontName(this->fontName);

  this->_setupGeometry();
}

//////////////////////////////////////////////////
void MovableText::Update()
{
  if (this->dirty)
  {
    this->_setupGeometry();
    this->dirty = false;
  }
}

//////////////////////////////////////////////////
void MovableText::SetFontName(const std::string &newFontName)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if ((Ogre::MaterialManager::getSingletonPtr()->resourceExists(
          this->mName + "Material")))
  {
    Ogre::MaterialManager::getSingleton().remove(this->mName + "Material");
  }

  if (this->fontName != newFontName || this->material.isNull() || !this->font)
  {
    this->fontName = newFontName;

    this->font = (Ogre::Font*)Ogre::FontManager::getSingleton().getByName(
        this->fontName).getPointer();

    if (!this->font)
    {
      throw Ogre::Exception(Ogre::Exception::ERR_ITEM_NOT_FOUND,
                            "Could not find font " + fontName,
                            "MovableText::setFontName");
    }

    this->font->load();

    if (!this->material.isNull())
    {
      Ogre::MaterialManager::getSingletonPtr()->remove(
          this->material->getName());
      this->material.setNull();
    }

    this->material = this->font->getMaterial()->clone(this->mName + "Material");

    if (!this->material->isLoaded())
      this->material->load();

    this->material->setDepthCheckEnabled(!this->onTop);
    this->material->setDepthBias(!this->onTop, 0);
    this->material->setDepthWriteEnabled(this->onTop);
    this->material->setLightingEnabled(false);

    this->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetText(const std::string &newText)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if (this->text != newText)
  {
    this->text = newText;
    this->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetColor(const common::Color &newColor)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if (this->color != newColor)
  {
    this->color = newColor;
    this->updateColors = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetCharHeight(float _height)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if (!math::equal(this->charHeight, _height))
  {
    this->charHeight = _height;
    this->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetSpaceWidth(float _width)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if (!math::equal(this->spaceWidth, _width))
  {
    this->spaceWidth = _width;
    this->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetTextAlignment(const HorizAlign &h, const VertAlign &v)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->horizAlign != h)
  {
    this->horizAlign = h;
    this->needUpdate = true;
  }

  if (this->vertAlign != v)
  {
    this->vertAlign = v;
    this->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetBaseline(float _base)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (!math::equal(this->baseline, _base))
  {
    this->baseline = _base;
    this->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetShowOnTop(bool show)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->onTop != show && !this->material.isNull())
  {
    this->onTop = show;

    this->material->setDepthBias(!this->onTop, 0);
    this->material->setDepthCheckEnabled(!this->onTop);
    this->material->setDepthWriteEnabled(this->onTop);
  }
}

//////////////////////////////////////////////////
bool MovableText::GetShowOnTop() const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return this->onTop;
}

//////////////////////////////////////////////////
math::Box MovableText::GetAABB(void)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return math::Box(
      math::Vector3(this->aabb->getMinimum().x,
                    this->aabb->getMinimum().y,
                    this->aabb->getMinimum().z),
      math::Vector3(this->aabb->getMaximum().x,
                    this->aabb->getMaximum().y,
                    this->aabb->getMaximum().z));
}

//////////////////////////////////////////////////
void MovableText::_setupGeometry()
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  assert(this->font);
  assert(!this->material.isNull());

  Ogre::VertexDeclaration *decl = NULL;
  Ogre::VertexBufferBinding *bind = NULL;
  Ogre::HardwareVertexBufferSharedPtr ptbuf;
  Ogre::HardwareVertexBufferSharedPtr cbuf;
  float *pVert = NULL;
  float largestWidth = 0;
  float left = 0;
  float top = 0;
  size_t offset = 0;
  float maxSquaredRadius = 0.0f;
  bool first = true;
  std::string::iterator i;
  bool newLine = true;
  float len = 0.0f;

  // for calculation of AABB
  Ogre::Vector3 min(0, 0, 0);
  Ogre::Vector3 max(0, 0, 0);
  Ogre::Vector3 currPos(0, 0, 0);

  unsigned int vertexCount = static_cast<unsigned int>(this->text.size() * 6);


  if (this->renderOp.vertexData)
  {
    // Removed this test as it causes problems when replacing a caption
    // of the same size: replacing "Hello" with "hello"
    // as well as when changing the text alignment
    // if (this->renderOp.vertexData->vertexCount != vertexCount)
    {
      delete this->renderOp.vertexData;
      this->renderOp.vertexData = NULL;
      this->updateColors = true;
    }
  }

  if (!this->renderOp.vertexData)
    this->renderOp.vertexData = new Ogre::VertexData();

  this->renderOp.indexData = 0;
  this->renderOp.vertexData->vertexStart = 0;
  this->renderOp.vertexData->vertexCount = vertexCount;
  this->renderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
  this->renderOp.useIndexes = false;

  decl = this->renderOp.vertexData->vertexDeclaration;
  bind = this->renderOp.vertexData->vertexBufferBinding;

  // create/bind positions/tex.ccord. buffer
  if (!decl->findElementBySemantic(Ogre::VES_POSITION))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT3,
                     Ogre::VES_POSITION);

  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (!decl->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT2,
                     Ogre::VES_TEXTURE_COORDINATES, 0);

  ptbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            decl->getVertexSize(POS_TEX_BINDING),
            this->renderOp.vertexData->vertexCount,
            Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(POS_TEX_BINDING, ptbuf);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(Ogre::VES_DIFFUSE))
    decl->addElement(COLOUR_BINDING, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  cbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
           decl->getVertexSize(COLOUR_BINDING),
           this->renderOp.vertexData->vertexCount,
           Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(COLOUR_BINDING, cbuf);

  pVert = static_cast<float*>(ptbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Derive space width from a capital A
  if (math::equal(this->spaceWidth, 0.0f))
    this->spaceWidth =
      this->font->getGlyphAspectRatio('A') * this->charHeight * 2.0;

  if (this->vertAlign == MovableText::V_ABOVE)
  {
    // Raise the first line of the caption
    top += this->charHeight;

    for (i = this->text.begin(); i != this->text.end(); ++i)
    {
      if (*i == '\n')
        top += this->charHeight * 2.0;
    }
  }

  for (i = this->text.begin(); i != this->text.end(); ++i)
  {
    if (newLine)
    {
      len = 0.0;
      for (std::string::iterator j = i; j != this->text.end(); ++j)
      {
        Ogre::Font::CodePoint character = *j;
        if (character == 0x000D  // CR
            || character == 0x0085)  // NEL
        {
          break;
        }
        else if (character == 0x0020)  // space
        {
          len += this->spaceWidth;
        }
        else
        {
          len += this->font->getGlyphAspectRatio(character) *
                 this->charHeight * 2.0 * this->viewportAspectCoef;
        }
      }

      newLine = false;
    }

    Ogre::Font::CodePoint character = (*i);

    if (character == 0x000D  // CR
        || character == 0x0085)  // NEL
    {
      top -= this->charHeight * 2.0;
      newLine = true;

      // Also reduce tri count
      this->renderOp.vertexData->vertexCount -= 6;
      continue;
    }
    else if (character == 0x0020)  // space
    {
      // Just leave a gap, no tris
      left += this->spaceWidth;

      // Also reduce tri count
      this->renderOp.vertexData->vertexCount -= 6;
      continue;
    }

    float horiz_height = this->font->getGlyphAspectRatio(character) *
                         this->viewportAspectCoef;

    const Ogre::Font::UVRect& uvRect = this->font->getGlyphTexCoords(character);

    // each vert is (x, y, z, u, v)
    //------------------------------------------------------------------------
    // First tri
    //
    // Upper left
    if (this->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.top;

    // Deal with bounds
    if (this->horizAlign == MovableText::H_LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2.0), top, 0);

    if (first)
    {
      min = max = currPos;
      maxSquaredRadius = currPos.squaredLength();
      first = false;
    }
    else
    {
      min.makeFloor(currPos);
      max.makeCeil(currPos);
      maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());
    }

    top -= this->charHeight * 2.0;

    // Bottom left
    if (this->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len / 2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    if (this->horizAlign == MovableText::H_LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2), top, 0);

    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    top += this->charHeight * 2.0;
    left += horiz_height * this->charHeight * 2.0;

    // Top right
    if (this->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;

    // Deal with bounds
    if (this->horizAlign == MovableText::H_LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2), top, 0);

    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    //------------------------------------------------------------------------

    //------------------------------------------------------------------------
    // Second tri
    //
    // Top right (again)
    if (this->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);
    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;

    // Deal with bounds
    currPos = Ogre::Vector3(left, top, 0);
    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    top -= this->charHeight * 2.0;
    left -= horiz_height  * this->charHeight * 2.0;

    // Bottom left (again)
    if (this->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);
    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    currPos = Ogre::Vector3(left, top, 0);
    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    left += horiz_height  * this->charHeight * 2.0;

    // Bottom right
    if (this->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);
    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    currPos = Ogre::Vector3(left, top, 0);
    min.makeFloor(currPos);
    max.makeFloor(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());

    //-------------------------------------------------------------------------


    // Go back up with top
    top += this->charHeight * 2.0;

    float currentWidth = (left + 1.0)/2.0;
    if (currentWidth > largestWidth)
    {
      largestWidth = currentWidth;
    }
  }

  // Unlock vertex buffer
  ptbuf->unlock();



  // update AABB/Sphere radius
  this->aabb->setMinimum(min);
  this->aabb->setMaximum(max);
  this->radius = Ogre::Math::Sqrt(maxSquaredRadius);

  if (this->updateColors)
    this->_updateColors();

  this->needUpdate = false;
}

//////////////////////////////////////////////////
void MovableText::_updateColors(void)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  Ogre::RGBA clr;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  Ogre::RGBA *pDest = NULL;
  unsigned int i;

  assert(this->font);
  assert(!this->material.isNull());

  // Convert to system-specific
  Ogre::ColourValue cv(this->color.r, this->color.g,
                       this->color.b, this->color.a);
  Ogre::Root::getSingleton().convertColourValue(cv, &clr);

  vbuf = this->renderOp.vertexData->vertexBufferBinding->getBuffer(
         COLOUR_BINDING);

  pDest = static_cast<Ogre::RGBA*>(
      vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  for (i = 0; i < this->renderOp.vertexData->vertexCount; ++i)
  {
    *pDest++ = clr;
  }

  vbuf->unlock();
  this->updateColors = false;
}

//////////////////////////////////////////////////
const Ogre::Quaternion & MovableText::getWorldOrientation(void) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  assert(this->camera);
  return const_cast<Ogre::Quaternion&>(this->camera->getDerivedOrientation());
  // return mParentNode->_getDerivedOrientation();
}

//////////////////////////////////////////////////
const Ogre::Vector3 & MovableText::getWorldPosition(void) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  assert(mParentNode);
  return mParentNode->_getDerivedPosition();
}

//////////////////////////////////////////////////
const Ogre::AxisAlignedBox &MovableText::getBoundingBox(void) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return *this->aabb;
}

//////////////////////////////////////////////////
const Ogre::String &MovableText::getMovableType() const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  static Ogre::String movType = "MovableText";
  return movType;
}

//////////////////////////////////////////////////
void MovableText::getWorldTransforms(Ogre::Matrix4 * xform) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->isVisible() && this->camera)
  {
    Ogre::Matrix3 rot3x3, scale3x3 = Ogre::Matrix3::IDENTITY;

    // store rotation in a matrix
    this->camera->getDerivedOrientation().ToRotationMatrix(rot3x3);
    // mParentNode->_getDerivedOrientation().ToRotationMatrix(rot3x3);

    // parent node position
    Ogre::Vector3 ppos = mParentNode->_getDerivedPosition() +
                         Ogre::Vector3::UNIT_Z * this->baseline;

    // apply scale
    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    // apply all transforms to xform
    *xform = (rot3x3 * scale3x3);
    xform->setTrans(ppos);
  }
}

//////////////////////////////////////////////////
float MovableText::getBoundingRadius() const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return this->radius;
}

//////////////////////////////////////////////////
float MovableText::getSquaredViewDepth(const Ogre::Camera * /*cam_*/) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return 0;
}

//////////////////////////////////////////////////
void MovableText::getRenderOperation(Ogre::RenderOperation & op)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);

  if (this->isVisible())
  {
    if (this->needUpdate)
      this->_setupGeometry();
    if (this->updateColors)
      this->_updateColors();
    op = this->renderOp;
  }
}

//////////////////////////////////////////////////
const Ogre::MaterialPtr &MovableText::getMaterial(void) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  assert(!this->material.isNull());
  return this->material;
}

//////////////////////////////////////////////////
const Ogre::LightList &MovableText::getLights(void) const
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return this->lightList;
}

//////////////////////////////////////////////////
void MovableText::_notifyCurrentCamera(Ogre::Camera *cam)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  this->camera = cam;
}

//////////////////////////////////////////////////
void MovableText::_updateRenderQueue(Ogre::RenderQueue* queue)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  if (this->isVisible())
  {
    if (this->needUpdate)
      this->_setupGeometry();

    if (this->updateColors)
      this->_updateColors();

    queue->addRenderable(this, mRenderQueueID,
                         OGRE_RENDERABLE_DEFAULT_PRIORITY);
  }
}

//////////////////////////////////////////////////
void MovableText::visitRenderables(Ogre::Renderable::Visitor* /*visitor*/,
                                 bool /*debug*/)
{
  boost::recursive_mutex::scoped_lock lock(*this->mutex);
  return;
}
