/*
 * File: MovableText.h
 *
 * description: This create create a billboarding object that display a text.
 * 
 * @author  2003 by cTh see gavocanov@rambler.ru
 * @update  2006 by barraq see nospam@barraquand.com
 */

#ifndef __include_MovableText_H__
#define __include_MovableText_H__

#include <Ogre.h>

//namespace gazebo {

class MovableText : public Ogre::MovableObject, public Ogre::Renderable
{
  /******************************** MovableText data ****************************/
  public:
    enum HorizontalAlignment    {H_LEFT, H_CENTER};
    enum VerticalAlignment      {V_BELOW, V_ABOVE};

  protected:
    Ogre::String			mFontName;
    Ogre::String			mType;
    Ogre::String			mName;
    Ogre::String			mCaption;
    HorizontalAlignment	mHorizontalAlignment;
    VerticalAlignment	mVerticalAlignment;

    Ogre::ColourValue		mColor;
    Ogre::RenderOperation	mRenderOp;
    Ogre::AxisAlignedBox	mAABB;
    Ogre::LightList		mLList;

    Ogre::Real			mCharHeight;
    Ogre::Real			mSpaceWidth;

    bool			mNeedUpdate;
    bool			mUpdateColors;
    bool			mOnTop;

    Ogre::Real			mTimeUntilNextToggle;
    Ogre::Real			mRadius;
    Ogre::Real            mAdditionalHeight;

    Ogre::Camera			*mpCam;
    Ogre::RenderWindow	*mpWin;
    Ogre::Font			*mpFont;
    Ogre::MaterialPtr		mpMaterial;
    Ogre::MaterialPtr		mpBackgroundMaterial;

    /******************************** public methods ******************************/
  public:
    MovableText(const Ogre::String &name, const Ogre::String &caption, const Ogre::String &fontName = "Console", Ogre::Real charHeight = 1.0, const Ogre::ColourValue &color = Ogre::ColourValue::White);
    virtual ~MovableText();

    // Add to build on Shoggoth:
    /*
       virtual void visitRenderables(Ogre::Renderable::Visitor* visitor, bool debugRenderables = false);
       */

    // Set settings
    void    setFontName(const Ogre::String &fontName);
    void    setCaption(const Ogre::String &caption);
    void    setColor(const Ogre::ColourValue &color);
    void    setCharacterHeight(Ogre::Real height);
    void    setSpaceWidth(Ogre::Real width);
    void    setTextAlignment(const HorizontalAlignment& horizontalAlignment, const VerticalAlignment& verticalAlignment);
    void    setAdditionalHeight( Ogre::Real height );
    void    showOnTop(bool show=true);

    // Get settings
    const   Ogre::String          &getFontName() const {return mFontName;}
    const   Ogre::String          &getCaption() const {return mCaption;}
    const   Ogre::ColourValue     &getColor() const {return mColor;}

    unsigned int    getCharacterHeight() const {return (unsigned int)mCharHeight;}
    unsigned int    getSpaceWidth() const {return (unsigned int)mSpaceWidth;}
    Ogre::Real    getAdditionalHeight() const {return (Ogre::Real)mAdditionalHeight;}
    bool    getShowOnTop() const {return mOnTop;}
    Ogre::AxisAlignedBox	        GetAABB(void) { return mAABB; }

    /******************************** protected methods and overload **************/
  protected:

    // from MovableText, create the object
    void	_setupGeometry();
    void	_updateColors();

    // from MovableObject
    void    getWorldTransforms(Ogre::Matrix4 *xform) const;
    Ogre::Real    getBoundingRadius(void) const {return mRadius;};
    Ogre::Real    getSquaredViewDepth(const Ogre::Camera *cam) const {return 0;};
    const   Ogre::Quaternion        &getWorldOrientation(void) const;
    const   Ogre::Vector3           &getWorldPosition(void) const;
    const   Ogre::AxisAlignedBox    &getBoundingBox(void) const {return mAABB;};
    const   Ogre::String            &getName(void) const {return mName;};
    const   Ogre::String            &getMovableType(void) const {static Ogre::String movType = ";MovableText";; return movType;};

    void    _notifyCurrentCamera(Ogre::Camera *cam);
    void    _updateRenderQueue(Ogre::RenderQueue* queue);

    // from renderable
    void    getRenderOperation(Ogre::RenderOperation &op);
    const   Ogre::MaterialPtr       &getMaterial(void) const {assert(!mpMaterial.isNull());return mpMaterial;};
    const   Ogre::LightList         &getLights(void) const {return mLList;};
};

#endif
