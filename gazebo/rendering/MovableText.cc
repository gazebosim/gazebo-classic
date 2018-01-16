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
/**
 * File: MovableText.cpp
 *
 * description: This create create a billboarding object that display a text.
 *
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 * @update  2007 by independentCreations see independentCreations@gmail.com
 */

#ifdef _WIN32
  // Ensure that windows types like LONGLONG are defined
  #include <windows.h>
#endif

#include <mutex>

#include "gazebo/common/common.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/rendering/MovableText.hh"

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1

using namespace gazebo;
using namespace rendering;

/// \brief Private data for the MovableText class.
class gazebo::rendering::MovableTextPrivate
{
  /// \brief Font name, such as "Arial"
  public: std::string fontName;

  /// \brief Text being displayed
  public: std::string text;

  /// \brief Text color
  public: ignition::math::Color color;

  /// \brief Character height in meters
  public: float charHeight;

  /// \brief Set to true when needing an update on getRenderOperation or
  /// _updateRenderQueue.
  public: bool needUpdate;

  /// \brief Set to true when color needs updating
  public: bool updateColors = true;

  /// \brief True when needing an update on Update()
  public: bool dirty = true;

  /// \brief Bounding radius
  public: float radius;

  /// \brief Viewport aspect coefficient
  public: float viewportAspectCoef = 0.75;

  /// \brief Width of space between letters
  public: float spaceWidth = 0.0;

  /// \brief Vertical alignment
  public: MovableText::VertAlign vertAlign = MovableText::V_BELOW;

  /// \brief Horizontal alignment
  public: MovableText::HorizAlign horizAlign = MovableText::H_LEFT;

  /// \brief True for text to be displayed on top of other objects in the scene.
  public: bool onTop = false;

  /// \brief Baseline height in meters.
  public: float baseline = 0.0;

  /// \brief Mutex to protect updated
  public: mutable std::recursive_mutex mutex;

  /// \brief Ogre render operation
  public: Ogre::RenderOperation renderOp;

  /// \brief Axis aligned box
  public: Ogre::AxisAlignedBox *aabb = nullptr;

  /// \brief Pointer to camera which the text is facing - never set.
  public: Ogre::Camera *camera = nullptr;

  /// \brief Pointer to font
  public: Ogre::Font *font = nullptr;

  /// \brief Text material
  public: Ogre::MaterialPtr material;

  /// \brief Keep an empty list of lights.
  public: Ogre::LightList lightList;
};

//////////////////////////////////////////////////
MovableText::MovableText()
    : dataPtr(new MovableTextPrivate)
{
  this->dataPtr->renderOp.vertexData = nullptr;

  this->dataPtr->aabb = new Ogre::AxisAlignedBox;
}

//////////////////////////////////////////////////
MovableText::~MovableText()
{
  delete this->dataPtr->renderOp.vertexData;
  delete this->dataPtr->aabb;
}

//////////////////////////////////////////////////
void MovableText::Load(const std::string &_name,
                       const std::string &_text,
                       const std::string &_fontName,
                       const float _charHeight,
                       const common::Color &_color)
{
  this->Load(_name, _text, _fontName, _charHeight, _color.Ign());
}

//////////////////////////////////////////////////
void MovableText::Load(const std::string &_name,
                       const std::string &_text,
                       const std::string &_fontName,
                       const float _charHeight,
                       const ignition::math::Color &_color)
{
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

    this->dataPtr->text = _text;
    this->dataPtr->color = _color;
    this->dataPtr->fontName = _fontName;
    this->dataPtr->charHeight = _charHeight;
    this->mName = _name;

    if (this->mName == "")
    {
      throw Ogre::Exception(Ogre::Exception::ERR_INVALIDPARAMS,
          "Trying to create MovableText without name",
          "MovableText::MovableText");
    }

    if (this->dataPtr->text == "")
    {
      throw Ogre::Exception(Ogre::Exception::ERR_INVALIDPARAMS,
          "Trying to create MovableText without text",
          "MovableText::MovableText");
    }

    this->dataPtr->dirty = true;
  }

  this->SetFontName(this->dataPtr->fontName);

  this->SetupGeometry();
}

//////////////////////////////////////////////////
void MovableText::Update()
{
  if (this->dataPtr->dirty)
  {
    this->SetupGeometry();
    this->dataPtr->dirty = false;
  }
}

//////////////////////////////////////////////////
void MovableText::SetFontName(const std::string &_newFontName)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  if ((Ogre::MaterialManager::getSingletonPtr()->resourceExists(
          this->mName + "Material")))
  {
    Ogre::MaterialManager::getSingleton().remove(this->mName + "Material");
  }

  if (this->dataPtr->fontName != _newFontName ||
      this->dataPtr->material.isNull() || !this->dataPtr->font)
  {
    auto font = (Ogre::Font*)Ogre::FontManager::getSingleton()
        .getByName(_newFontName).getPointer();

    if (!font)
    {
      throw Ogre::Exception(Ogre::Exception::ERR_ITEM_NOT_FOUND,
                            "Could not find font " + _newFontName,
                            "MovableText::setFontName");
    }
    this->dataPtr->font = font;
    this->dataPtr->fontName = _newFontName;

    this->dataPtr->font->load();

    if (!this->dataPtr->material.isNull())
    {
      Ogre::MaterialManager::getSingletonPtr()->remove(
          this->dataPtr->material->getName());
      this->dataPtr->material.setNull();
    }

    this->dataPtr->material = this->dataPtr->font->getMaterial()->clone(
        this->mName + "Material");

    if (!this->dataPtr->material->isLoaded())
      this->dataPtr->material->load();

    this->dataPtr->material->setDepthCheckEnabled(!this->dataPtr->onTop);
    this->dataPtr->material->setDepthBias(!this->dataPtr->onTop, 0);
    this->dataPtr->material->setDepthWriteEnabled(this->dataPtr->onTop);
    this->dataPtr->material->setLightingEnabled(false);

    this->dataPtr->needUpdate = true;
  }
}

//////////////////////////////////////////////////
const std::string &MovableText::GetFont() const
{
  return this->dataPtr->fontName;
}

//////////////////////////////////////////////////
const std::string &MovableText::FontName() const
{
  return this->dataPtr->fontName;
}

//////////////////////////////////////////////////
void MovableText::SetText(const std::string &newText)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->text != newText)
  {
    this->dataPtr->text = newText;
    this->dataPtr->needUpdate = true;
  }
}

//////////////////////////////////////////////////
const std::string &MovableText::GetText() const
{
  return this->dataPtr->text;
}

//////////////////////////////////////////////////
const std::string &MovableText::Text() const
{
  return this->dataPtr->text;
}

//////////////////////////////////////////////////
void MovableText::SetColor(const common::Color &_newColor)
{
  this->SetColor(_newColor.Ign());
}

//////////////////////////////////////////////////
void MovableText::SetColor(const ignition::math::Color &_newColor)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->color != _newColor)
  {
    this->dataPtr->color = _newColor;
    this->dataPtr->updateColors = true;
  }
}

//////////////////////////////////////////////////
const common::Color MovableText::GetColor() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return this->Color();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}

//////////////////////////////////////////////////
const ignition::math::Color &MovableText::Color() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->color;
}

//////////////////////////////////////////////////
void MovableText::SetCharHeight(float _height)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  if (!ignition::math::equal(this->dataPtr->charHeight, _height))
  {
    this->dataPtr->charHeight = _height;
    this->dataPtr->needUpdate = true;
  }
}

//////////////////////////////////////////////////
float MovableText::GetCharHeight() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->charHeight;
}

//////////////////////////////////////////////////
float MovableText::CharHeight() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->charHeight;
}

//////////////////////////////////////////////////
void MovableText::SetSpaceWidth(float _width)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  if (!ignition::math::equal(this->dataPtr->spaceWidth, _width))
  {
    this->dataPtr->spaceWidth = _width;
    this->dataPtr->needUpdate = true;
  }
}

//////////////////////////////////////////////////
float MovableText::GetSpaceWidth() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->spaceWidth;
}

//////////////////////////////////////////////////
float MovableText::SpaceWidth() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->spaceWidth;
}

//////////////////////////////////////////////////
void MovableText::SetTextAlignment(const HorizAlign &h, const VertAlign &v)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->horizAlign != h)
  {
    this->dataPtr->horizAlign = h;
    this->dataPtr->needUpdate = true;
  }

  if (this->dataPtr->vertAlign != v)
  {
    this->dataPtr->vertAlign = v;
    this->dataPtr->needUpdate = true;
  }
}

//////////////////////////////////////////////////
void MovableText::SetBaseline(float _base)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  if (!ignition::math::equal(this->dataPtr->baseline, _base))
  {
    this->dataPtr->baseline = _base;
    this->dataPtr->needUpdate = true;
  }
}

//////////////////////////////////////////////////
float MovableText::GetBaseline() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->baseline;
}

//////////////////////////////////////////////////
float MovableText::Baseline() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->baseline;
}

//////////////////////////////////////////////////
void MovableText::SetShowOnTop(bool show)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->onTop != show && !this->dataPtr->material.isNull())
  {
    this->dataPtr->onTop = show;

    this->dataPtr->material->setDepthBias(!this->dataPtr->onTop, 0);
    this->dataPtr->material->setDepthCheckEnabled(!this->dataPtr->onTop);
    this->dataPtr->material->setDepthWriteEnabled(this->dataPtr->onTop);
  }
}

//////////////////////////////////////////////////
bool MovableText::GetShowOnTop() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->onTop;
}

//////////////////////////////////////////////////
bool MovableText::ShowOnTop() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->onTop;
}

//////////////////////////////////////////////////
ignition::math::Box MovableText::AABB()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return ignition::math::Box(
      ignition::math::Vector3d(this->dataPtr->aabb->getMinimum().x,
                    this->dataPtr->aabb->getMinimum().y,
                    this->dataPtr->aabb->getMinimum().z),
      ignition::math::Vector3d(this->dataPtr->aabb->getMaximum().x,
                    this->dataPtr->aabb->getMaximum().y,
                    this->dataPtr->aabb->getMaximum().z));
}

//////////////////////////////////////////////////
void MovableText::_setupGeometry()
{
  this->SetupGeometry();
}

//////////////////////////////////////////////////
void MovableText::SetupGeometry()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  GZ_ASSERT(this->dataPtr->font, "font class member is null");
  GZ_ASSERT(!this->dataPtr->material.isNull(), "material class member is null");

  Ogre::VertexDeclaration *decl = nullptr;
  Ogre::VertexBufferBinding *bind = nullptr;
  Ogre::HardwareVertexBufferSharedPtr ptbuf;
  Ogre::HardwareVertexBufferSharedPtr cbuf;
  float *pVert = nullptr;
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

  auto vertexCount = static_cast<unsigned int>(this->dataPtr->text.size() * 6);

  if (this->dataPtr->renderOp.vertexData)
  {
    // Removed this test as it causes problems when replacing a caption
    // of the same size: replacing "Hello" with "hello"
    // as well as when changing the text alignment
    // if (this->dataPtr->renderOp.vertexData->vertexCount != vertexCount)
    {
      delete this->dataPtr->renderOp.vertexData;
      this->dataPtr->renderOp.vertexData = nullptr;
      this->dataPtr->updateColors = true;
    }
  }

  if (!this->dataPtr->renderOp.vertexData)
    this->dataPtr->renderOp.vertexData = new Ogre::VertexData();

  this->dataPtr->renderOp.indexData = 0;
  this->dataPtr->renderOp.vertexData->vertexStart = 0;
  this->dataPtr->renderOp.vertexData->vertexCount = vertexCount;
  this->dataPtr->renderOp.operationType =
      Ogre::RenderOperation::OT_TRIANGLE_LIST;
  this->dataPtr->renderOp.useIndexes = false;

  decl = this->dataPtr->renderOp.vertexData->vertexDeclaration;
  bind = this->dataPtr->renderOp.vertexData->vertexBufferBinding;

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
            this->dataPtr->renderOp.vertexData->vertexCount,
            Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(POS_TEX_BINDING, ptbuf);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(Ogre::VES_DIFFUSE))
    decl->addElement(COLOUR_BINDING, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  cbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
           decl->getVertexSize(COLOUR_BINDING),
           this->dataPtr->renderOp.vertexData->vertexCount,
           Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(COLOUR_BINDING, cbuf);

  pVert = static_cast<float*>(ptbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Derive space width from a capital A
  if (ignition::math::equal(this->dataPtr->spaceWidth, 0.0f))
  {
    this->dataPtr->spaceWidth = this->dataPtr->font->getGlyphAspectRatio('A') *
        this->dataPtr->charHeight * 2.0;
  }

  if (this->dataPtr->vertAlign == MovableText::V_ABOVE)
  {
    // Raise the first line of the caption
    top += this->dataPtr->charHeight;

    for (i = this->dataPtr->text.begin(); i != this->dataPtr->text.end(); ++i)
    {
      if (*i == '\n')
        top += this->dataPtr->charHeight * 2.0;
    }
  }

  for (i = this->dataPtr->text.begin(); i != this->dataPtr->text.end(); ++i)
  {
    if (newLine)
    {
      len = 0.0;
      for (std::string::iterator j = i; j != this->dataPtr->text.end(); ++j)
      {
        Ogre::Font::CodePoint character = *j;
        if (character == 0x000D  // CR
            || character == 0x0085)  // NEL
        {
          break;
        }
        else if (character == 0x0020)  // space
        {
          len += this->dataPtr->spaceWidth;
        }
        else
        {
          len += this->dataPtr->font->getGlyphAspectRatio(character) *
                 this->dataPtr->charHeight * 2.0 *
                 this->dataPtr->viewportAspectCoef;
        }
      }

      newLine = false;
    }

    Ogre::Font::CodePoint character = (*i);

    if (character == 0x000D  // CR
        || character == 0x0085)  // NEL
    {
      top -= this->dataPtr->charHeight * 2.0;
      newLine = true;

      // Also reduce tri count
      this->dataPtr->renderOp.vertexData->vertexCount -= 6;
      continue;
    }
    else if (character == 0x0020)  // space
    {
      // Just leave a gap, no tris
      left += this->dataPtr->spaceWidth;

      // Also reduce tri count
      this->dataPtr->renderOp.vertexData->vertexCount -= 6;
      continue;
    }

    float horiz_height = this->dataPtr->font->getGlyphAspectRatio(character) *
                         this->dataPtr->viewportAspectCoef;

    auto &uvRect = this->dataPtr->font->getGlyphTexCoords(character);

    // each vert is (x, y, z, u, v)
    //------------------------------------------------------------------------
    // First tri
    //
    // Upper left
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.top;

    // Deal with bounds
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
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

    top -= this->dataPtr->charHeight * 2.0;

    // Bottom left
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len / 2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    // Deal with bounds
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
      currPos = Ogre::Vector3(left, top, 0);
    else
      currPos = Ogre::Vector3(left - (len/2), top, 0);

    min.makeFloor(currPos);
    max.makeCeil(currPos);
    maxSquaredRadius = std::max(maxSquaredRadius, currPos.squaredLength());


    top += this->dataPtr->charHeight * 2.0;
    left += horiz_height * this->dataPtr->charHeight * 2.0;

    // Top right
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
      *pVert++ = left;
    else
      *pVert++ = left - (len/2.0);

    *pVert++ = top;
    *pVert++ = 0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;

    // Deal with bounds
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
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
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
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


    top -= this->dataPtr->charHeight * 2.0;
    left -= horiz_height  * this->dataPtr->charHeight * 2.0;

    // Bottom left (again)
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
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


    left += horiz_height  * this->dataPtr->charHeight * 2.0;

    // Bottom right
    if (this->dataPtr->horizAlign == MovableText::H_LEFT)
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
    top += this->dataPtr->charHeight * 2.0;

    float currentWidth = (left + 1.0)/2.0;
    if (currentWidth > largestWidth)
    {
      largestWidth = currentWidth;
    }
  }

  // Unlock vertex buffer
  ptbuf->unlock();

  // update AABB/Sphere radius
  this->dataPtr->aabb->setMinimum(min);
  this->dataPtr->aabb->setMaximum(max);
  this->dataPtr->radius = Ogre::Math::Sqrt(maxSquaredRadius);

  if (this->dataPtr->updateColors)
    this->UpdateColors();

  this->dataPtr->needUpdate = false;
}

//////////////////////////////////////////////////
void MovableText::_updateColors()
{
  this->UpdateColors();
}

//////////////////////////////////////////////////
void MovableText::UpdateColors()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  Ogre::RGBA clr;
  Ogre::HardwareVertexBufferSharedPtr vbuf;
  Ogre::RGBA *pDest = nullptr;
  unsigned int i;

  GZ_ASSERT(this->dataPtr->font, "font class member is null");
  GZ_ASSERT(!this->dataPtr->material.isNull(), "material class member is null");

  // Convert to system-specific
  Ogre::ColourValue cv(this->dataPtr->color.R(), this->dataPtr->color.G(),
                       this->dataPtr->color.B(), this->dataPtr->color.A());
  Ogre::Root::getSingleton().convertColourValue(cv, &clr);

  vbuf = this->dataPtr->renderOp.vertexData->vertexBufferBinding->getBuffer(
         COLOUR_BINDING);

  pDest = static_cast<Ogre::RGBA*>(
      vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  for (i = 0; i < this->dataPtr->renderOp.vertexData->vertexCount; ++i)
  {
    *pDest++ = clr;
  }

  vbuf->unlock();
  this->dataPtr->updateColors = false;
}

//////////////////////////////////////////////////
const Ogre::AxisAlignedBox &MovableText::getBoundingBox(void) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return *this->dataPtr->aabb;
}

//////////////////////////////////////////////////
const Ogre::String &MovableText::getMovableType() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  static Ogre::String movType = "MovableText";
  return movType;
}

//////////////////////////////////////////////////
void MovableText::getWorldTransforms(Ogre::Matrix4 *_xform) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  if (this->isVisible() && this->dataPtr->camera)
  {
    Ogre::Matrix3 rot3x3, scale3x3 = Ogre::Matrix3::IDENTITY;

    // store rotation in a matrix
    this->dataPtr->camera->getDerivedOrientation().ToRotationMatrix(rot3x3);
    // mParentNode->_getDerivedOrientation().ToRotationMatrix(rot3x3);

    // parent node position
    Ogre::Vector3 ppos = mParentNode->_getDerivedPosition() +
                         Ogre::Vector3::UNIT_Z * this->dataPtr->baseline;

    // apply scale
    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    // apply all transforms to xform
    *_xform = (rot3x3 * scale3x3);
    _xform->setTrans(ppos);
  }
}

//////////////////////////////////////////////////
float MovableText::getBoundingRadius() const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->radius;
}

//////////////////////////////////////////////////
float MovableText::getSquaredViewDepth(const Ogre::Camera * /*cam_*/) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return 0;
}

//////////////////////////////////////////////////
void MovableText::getRenderOperation(Ogre::RenderOperation & op)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);

  if (this->isVisible())
  {
    if (this->dataPtr->needUpdate)
      this->SetupGeometry();
    if (this->dataPtr->updateColors)
      this->UpdateColors();
    op = this->dataPtr->renderOp;
  }
}

//////////////////////////////////////////////////
const Ogre::MaterialPtr &MovableText::getMaterial(void) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  GZ_ASSERT(!this->dataPtr->material.isNull(),
      "material class member is null");
  return this->dataPtr->material;
}

//////////////////////////////////////////////////
const Ogre::LightList &MovableText::getLights(void) const
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return this->dataPtr->lightList;
}

//////////////////////////////////////////////////
void MovableText::_notifyCurrentCamera(Ogre::Camera *cam)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->camera = cam;
}

//////////////////////////////////////////////////
void MovableText::_updateRenderQueue(Ogre::RenderQueue* queue)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  if (this->isVisible())
  {
    if (this->dataPtr->needUpdate)
      this->SetupGeometry();

    if (this->dataPtr->updateColors)
      this->UpdateColors();

    queue->addRenderable(this, mRenderQueueID,
                         OGRE_RENDERABLE_DEFAULT_PRIORITY);
  }
}

//////////////////////////////////////////////////
void MovableText::visitRenderables(Ogre::Renderable::Visitor* /*visitor*/,
                                 bool /*debug*/)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->mutex);
  return;
}
