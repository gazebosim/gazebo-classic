/**
 * File: MovableText.cpp
 *
 * description: This create create a billboarding object that display a text.
 * 
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 * @update  2007 by independentCreations see independentCreations@gmail.com
 */


#include "MovableText.hh"

#include <OgreFontManager.h>
#include <OgrePrerequisites.h>

#define POS_TEX_BINDING    0
#define COLOUR_BINDING     1
#if OGRE_VERSION_MAJOR >= 1
#if OGRE_VERSION_MINOR >= 4
#define OGRE_VERS_GREATER_EIHORT
#endif
#endif

////////////////////////////////////////////////////////////////////////////////
// Constructor
MovableText::MovableText(const Ogre::String & name, 
                         const Ogre::UTFString & caption, 
                         const Ogre::String & fontName, 
                         float charHeight, const Ogre::ColourValue & color)
  : mpCam(NULL) , 
    mpWin(NULL) , 
    mpFont(NULL) , 
    mName(name) , 
    mCaption(caption) , 
    mFontName(fontName) , 
    mCharHeight(charHeight) , 
    mColor(color) , 
    mType("MovableText") , 
    mTimeUntilNextToggle(0) , 
    mSpaceWidth(0) , 
    mUpdateColors(true) , 
    mOnTop(false) , 
    mHorizontalAlignment(H_LEFT) , 
    mVerticalAlignment(V_BELOW) , 
    mAdditionalHeight(0.0) , 
    mViewportAspectCoef (0.75)
{
  if (name == "")
    Ogre::Exception(Ogre::Exception::ERR_INVALIDPARAMS, "Trying to create MovableText without name", "MovableText::MovableText");

  if (caption == "")
    Ogre::Exception(Ogre::Exception::ERR_INVALIDPARAMS, "Trying to create MovableText without caption", "MovableText::MovableText");

  mRenderOp.vertexData = NULL;

  this->setFontName(mFontName);
  this->_setupGeometry();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
MovableText::~MovableText()
{
  if (mRenderOp.vertexData)
    delete mRenderOp.vertexData;
}

////////////////////////////////////////////////////////////////////////////////
// Set the font name
void MovableText::setFontName(const Ogre::String & fontName)
{
  if ((Ogre::MaterialManager::getSingletonPtr()->resourceExists(mName + "Material"))) 
  { 
    Ogre::MaterialManager::getSingleton().remove(mName + "Material"); 
  }

  if (mFontName != fontName || mpMaterial.isNull() || !mpFont)
  {
    mFontName = fontName;
    mpFont = (Ogre::Font *)Ogre::FontManager::getSingleton().getByName(mFontName).getPointer();
    if (!mpFont)
      Ogre::Exception(Ogre::Exception::ERR_ITEM_NOT_FOUND, "Could not find font " + fontName, "MovableText::setFontName");

    mpFont->load();
    if (!mpMaterial.isNull())
    {
      Ogre::MaterialManager::getSingletonPtr()->remove(mpMaterial->getName());
      mpMaterial.setNull();
    }

    mpMaterial = mpFont->getMaterial()->clone(mName + "Material");
    if (!mpMaterial->isLoaded())
      mpMaterial->load();

    mpMaterial->setDepthCheckEnabled(!mOnTop);
    mpMaterial->setDepthBias(!mOnTop, 0);
    mpMaterial->setDepthWriteEnabled(mOnTop);
    mpMaterial->setLightingEnabled(false);
    mNeedUpdate = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the caption
void MovableText::setCaption(const Ogre::UTFString & caption)
{
  if (caption != mCaption)
  {
    mCaption = caption;
    mNeedUpdate = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the color
void MovableText::setColor(const Ogre::ColourValue & color)
{
  if (color != mColor)
  {
    mColor = color;
    mUpdateColors = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the character height
void MovableText::setCharacterHeight(float height)
{
  if (height != mCharHeight)
  {
    mCharHeight = height;
    mNeedUpdate = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the width of the space between characters
void MovableText::setSpaceWidth(unsigned int width)
{
  if (width != mSpaceWidth)
  {
    mSpaceWidth = width;
    mNeedUpdate = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set alignment of the text
void MovableText::setTextAlignment(const HorizontalAlignment &horizontalAlignment, const VerticalAlignment & verticalAlignment)
{
  if(mHorizontalAlignment != horizontalAlignment)
  {
    mHorizontalAlignment = horizontalAlignment;
    mNeedUpdate = true;
  }
  if(mVerticalAlignment != verticalAlignment)
  {
    mVerticalAlignment = verticalAlignment;
    mNeedUpdate = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set additional height
void MovableText::setAdditionalHeight( float height )
{
  if( mAdditionalHeight != height )
  {
    mAdditionalHeight = height;
    mNeedUpdate = true;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set whether the text should be shown on top
void MovableText::showOnTop(bool show)
{
  if( mOnTop != show && !mpMaterial.isNull() )
  {
    mOnTop = show;
    mpMaterial->setDepthBias(!mOnTop, 0);
    mpMaterial->setDepthCheckEnabled(!mOnTop);
    mpMaterial->setDepthWriteEnabled(mOnTop);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Setup the billboard that renders the text
void MovableText::_setupGeometry()
{
  assert(mpFont);
  assert(!mpMaterial.isNull());

  unsigned int vertexCount = static_cast<unsigned int>(mCaption.size() * 6);

  if (mRenderOp.vertexData)
  {
    // Removed this test as it causes problems when replacing a caption
    // of the same size: replacing "Hello" with "hello"
    // as well as when changing the text alignment
    //if (mRenderOp.vertexData->vertexCount != vertexCount)
    {
      delete mRenderOp.vertexData;
      mRenderOp.vertexData = NULL;
      mUpdateColors = true;
    }
  }

  if (!mRenderOp.vertexData)
    mRenderOp.vertexData = new Ogre::VertexData();

  mRenderOp.indexData = 0;
  mRenderOp.vertexData->vertexStart = 0;
  mRenderOp.vertexData->vertexCount = vertexCount;
  mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST; 
  mRenderOp.useIndexes = false; 

  Ogre::VertexDeclaration * decl = mRenderOp.vertexData->vertexDeclaration;
  Ogre::VertexBufferBinding * bind = mRenderOp.vertexData->vertexBufferBinding;
  size_t offset = 0;

  // create/bind positions/tex.ccord. buffer
  if (!decl->findElementBySemantic(Ogre::VES_POSITION))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

  offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (!decl->findElementBySemantic(Ogre::VES_TEXTURE_COORDINATES))
    decl->addElement(POS_TEX_BINDING, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);

  Ogre::HardwareVertexBufferSharedPtr ptbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(POS_TEX_BINDING),
      mRenderOp.vertexData->vertexCount,
      Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(POS_TEX_BINDING, ptbuf);

  // Colours - store these in a separate buffer because they change less often
  if (!decl->findElementBySemantic(Ogre::VES_DIFFUSE))
    decl->addElement(COLOUR_BINDING, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);

  Ogre::HardwareVertexBufferSharedPtr cbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(COLOUR_BINDING),
      mRenderOp.vertexData->vertexCount,
      Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

  bind->setBinding(COLOUR_BINDING, cbuf);

  size_t charlen = mCaption.size();
  float *pVert = static_cast<float*>(ptbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  float largestWidth = 0;
  float left = 0 * 2.0 - 1.0;
  float top = -((0 * 2.0) - 1.0);

  // Derive space width from a capital A
  if (mSpaceWidth == 0)
    mSpaceWidth = mpFont->getGlyphAspectRatio('A') * mCharHeight * 2.0;

  // for calculation of AABB
  Ogre::Vector3 min, max, currPos;
  float maxSquaredRadius = 0.0f;
  bool first = true;

  // Use iterator
  Ogre::UTFString::iterator i, iend;
  iend = mCaption.end();
  bool newLine = true;
  float len = 0.0f;

  if(mVerticalAlignment == MovableText::V_ABOVE)
  {
    // Raise the first line of the caption
    top += mCharHeight;
    for (i = mCaption.begin(); i != iend; ++i)
    {
      if (*i == '\n')
        top += mCharHeight * 2.0;
    }
  }

  iend = mCaption.end();
  for( i = mCaption.begin(); i != iend; ++i )
  {
    if( newLine )
    {
      float len = 0.0f;
      for( Ogre::UTFString::iterator j = i; j != iend; j++ )
      {
        Ogre::Font::CodePoint character = j.getCharacter();
        if (character == 0x000D // CR
            || character == 0x0085) // NEL
        {
          break;
        }
        else if (character == 0x0020) // space
        {
          len += mSpaceWidth;
        }
        else 
        {
          len += mpFont->getGlyphAspectRatio(character) * mCharHeight * 2.0 * mViewportAspectCoef;
        }
      }

      /*if( mAlignment == Right )
        left -= len;
        else if( mAlignment == Center )
        left -= len * 0.5;*/

      newLine = false;
    }

    Ogre::Font::CodePoint character = i.getCharacter();
    if (character == 0x000D // CR
        || character == 0x0085) // NEL
    {
      left = /*_getDerivedLeft() * 2.0*/ - 1.0;
      top -= mCharHeight * 2.0;
      newLine = true;
      // Also reduce tri count
      mRenderOp.vertexData->vertexCount -= 6;
      continue;
    }
    else if (character == 0x0020) // space
    {
      // Just leave a gap, no tris
      left += mSpaceWidth;
      // Also reduce tri count
      mRenderOp.vertexData->vertexCount -= 6;
      continue;
    }

    float horiz_height = mpFont->getGlyphAspectRatio(character) * mViewportAspectCoef ;
    const Ogre::Font::UVRect& uvRect = mpFont->getGlyphTexCoords(character);

    // each vert is (x, y, z, u, v)
    //------------------------------------------------------------------------
    // First tri
    //
    // Upper left
    *pVert++ = left;
    *pVert++ = top;
    *pVert++ = -1.0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.top;

    top -= mCharHeight * 2.0;

    // Bottom left
    *pVert++ = left;
    *pVert++ = top;
    *pVert++ = -1.0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    top += mCharHeight * 2.0;
    left += horiz_height * mCharHeight * 2.0;

    // Top right
    *pVert++ = left;
    *pVert++ = top;
    *pVert++ = -1.0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;
    //------------------------------------------------------------------------

    //------------------------------------------------------------------------
    // Second tri
    //
    // Top right (again)
    *pVert++ = left;
    *pVert++ = top;
    *pVert++ = -1.0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.top;

    top -= mCharHeight * 2.0;
    left -= horiz_height  * mCharHeight * 2.0;

    // Bottom left (again)
    *pVert++ = left;
    *pVert++ = top;
    *pVert++ = -1.0;
    *pVert++ = uvRect.left;
    *pVert++ = uvRect.bottom;

    left += horiz_height  * mCharHeight * 2.0;

    // Bottom right
    *pVert++ = left;
    *pVert++ = top;
    *pVert++ = -1.0;
    *pVert++ = uvRect.right;
    *pVert++ = uvRect.bottom;
    //-------------------------------------------------------------------------

    // Go back up with top
    top += mCharHeight * 2.0;

    float currentWidth = (left + 1)/2 /*- _getDerivedLeft()*/;
    if (currentWidth > largestWidth)
    {
      largestWidth = currentWidth;

    }
  }

  // Unlock vertex buffer
  ptbuf->unlock();

  min.x=min.y=min.z=-10000;
  max.x=max.y=max.z = 10000;

  // update AABB/Sphere radius
  mAABB = Ogre::AxisAlignedBox(min, max);
  mRadius = Ogre::Math::Sqrt(maxSquaredRadius);

  if (mUpdateColors)
    this->_updateColors();

  mNeedUpdate = false;
}

////////////////////////////////////////////////////////////////////////////////
// Update the colors
void MovableText::_updateColors(void)
{
  assert(mpFont);
  assert(!mpMaterial.isNull());

  // Convert to system-specific
  Ogre::RGBA color;
  Ogre::Root::getSingleton().convertColourValue(mColor, & color);
  Ogre::HardwareVertexBufferSharedPtr vbuf = mRenderOp.vertexData->vertexBufferBinding->getBuffer(COLOUR_BINDING);
  Ogre::RGBA *pDest = static_cast<Ogre::RGBA*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  for (unsigned int i = 0; i < mRenderOp.vertexData->vertexCount; ++i)
    *pDest++ = color;
  vbuf->unlock();
  mUpdateColors = false;
}

const Ogre::Quaternion & MovableText::getWorldOrientation(void) const
{
  assert(mpCam);
  //return const_cast<Ogre::Quaternion&>(mpCam->getDerivedOrientation());
  return mParentNode->_getDerivedOrientation();
}

const Ogre::Vector3 & MovableText::getWorldPosition(void) const
{
  assert(mParentNode);
  return mParentNode->_getDerivedPosition();
}

void MovableText::getWorldTransforms(Ogre::Matrix4 * xform) const 
{
  if (this->isVisible() && mpCam)
  {
    Ogre::Matrix3 rot3x3, scale3x3 = Ogre::Matrix3::IDENTITY;

    // store rotation in a matrix
    //mpCam->getDerivedOrientation().ToRotationMatrix(rot3x3);
    mParentNode->_getDerivedOrientation().ToRotationMatrix(rot3x3);

    // parent node position
    Ogre::Vector3 ppos = mParentNode->_getDerivedPosition() + Ogre::Vector3::UNIT_Y * mAdditionalHeight;

//    std::cout << "Parent Pos[" << ppos << "]\n";

    // apply scale
    scale3x3[0][0] = mParentNode->_getDerivedScale().x / 2;
    scale3x3[1][1] = mParentNode->_getDerivedScale().y / 2;
    scale3x3[2][2] = mParentNode->_getDerivedScale().z / 2;

    // apply all transforms to xform       
  //  *xform = (rot3x3 * scale3x3);
    xform->setTrans(ppos);
  }
}

void MovableText::getRenderOperation(Ogre::RenderOperation & op)
{
  if (this->isVisible())
  {
    if (mNeedUpdate)
      this->_setupGeometry();
    if (mUpdateColors)
      this->_updateColors();
    op = mRenderOp;
  }
}

void MovableText::_notifyCurrentCamera(Ogre::Camera *cam)
{
  mpCam = cam;
}

void MovableText::_updateRenderQueue(Ogre::RenderQueue* queue)
{
  if (this->isVisible())
  {
    if (mNeedUpdate)
      this->_setupGeometry();
    if (mUpdateColors)
      this->_updateColors();

    queue->addRenderable(this, mRenderQueueID, OGRE_RENDERABLE_DEFAULT_PRIORITY);
    //      queue->addRenderable(this, mRenderQueueID, RENDER_QUEUE_SKIES_LATE);
  }
}
