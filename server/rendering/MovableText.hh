/**
 *  * File: MovableText.h
 *   *
 *    * description: This create create a billboarding object that display
 *    a text.
 *     * 
 *      * @author  2003 by cTh see gavocanov@rambler.ru
 *       * @update  2006 by barraq see nospam@barraquand.com
 *        * @update  2007 by independentCreations see
 *        independentCreations@gmail.com
 *         */

#ifndef MOVABLETEXT_HH
#define MOVABLETEXT_HH

#include <Ogre.h>

class MovableText : public Ogre::MovableObject, public Ogre::Renderable
{
  public:
    enum HorizontalAlignment    {H_LEFT, H_CENTER};
    enum VerticalAlignment      {V_BELOW, V_ABOVE};

  protected:
    Ogre::String      mFontName;
    Ogre::String      mType;
    Ogre::String      mName;
    Ogre::UTFString     mCaption;
    HorizontalAlignment   mHorizontalAlignment;
    VerticalAlignment   mVerticalAlignment;

    Ogre::ColourValue   mColor;
    Ogre::RenderOperation mRenderOp;
    Ogre::AxisAlignedBox  mAABB;
    Ogre::LightList     mLList;

    float      mCharHeight;
    unsigned int      mSpaceWidth;

    bool          mNeedUpdate;
    bool          mUpdateColors;
    bool          mOnTop;

    float         mTimeUntilNextToggle;
    float         mRadius;
    float         mAdditionalHeight;

    Ogre::Camera      * mpCam;
    Ogre::RenderWindow    * mpWin;
    Ogre::Font        * mpFont;
    Ogre::MaterialPtr   mpMaterial;
    Ogre::MaterialPtr   mpBackgroundMaterial;

  public: MovableText(const Ogre::String & name, 
                      const Ogre::UTFString & caption, 
                      const Ogre::String & fontName = "Arial", 
                      float charHeight = 1.0,
                      const Ogre::ColourValue & color = Ogre::ColourValue::White);
  public: virtual ~MovableText();

    // Set settings
  public: void setFontName(const Ogre::String & fontName);
  public: void setCaption(const Ogre::UTFString & caption);
  public: void setColor(const Ogre::ColourValue & color);
  public: void setCharacterHeight(float height);
  public:void setSpaceWidth(unsigned int width);

  public: void setTextAlignment(const HorizontalAlignment & horizontalAlignment,                                const VerticalAlignment & verticalAlignment);

  public: void setAdditionalHeight( float height);
  public: void showOnTop(bool show = true);

    // Get settings
  public: const Ogre::String & getFontName() const {return mFontName;}
  public: const Ogre::UTFString & getCaption() const {return mCaption;}
  public: const Ogre::ColourValue & getColor() const {return mColor;}

  public: float getCharacterHeight() const {return mCharHeight;}
  public: unsigned int getSpaceWidth() const {return mSpaceWidth;}
  public: float getAdditionalHeight() const {return mAdditionalHeight;}
  public: bool getShowOnTop() const {return mOnTop;}
  public: Ogre::AxisAlignedBox GetAABB(void) { return mAABB; }

  protected: float mViewportAspectCoef;

    // from MovableText, create the object
  protected: void _setupGeometry();
  protected: void _updateColors();

    // from MovableObject
  protected: void getWorldTransforms(Ogre::Matrix4 *xform) const;
  protected: float getBoundingRadius(void) const {return mRadius;};
  protected: float getSquaredViewDepth(const Ogre::Camera *cam) const {return 0;};
  protected: const Ogre::Quaternion & getWorldOrientation(void) const;
  protected: const Ogre::Vector3 & getWorldPosition(void) const;
  protected: const Ogre::AxisAlignedBox & getBoundingBox(void) const {return mAABB;};
  protected: const Ogre::String & getName(void) const {return mName;};
  protected: const Ogre::String & getMovableType(void) const {static Ogre::String movType = "MovableText"; return movType;};

  protected: void _notifyCurrentCamera(Ogre::Camera *cam);
  protected: void _updateRenderQueue(Ogre::RenderQueue* queue);

    // from renderable
  protected: void getRenderOperation(Ogre::RenderOperation &op);
  protected: const Ogre::MaterialPtr &getMaterial(void) const {assert(!mpMaterial.isNull());return mpMaterial;};
  protected: const Ogre::LightList &getLights(void) const {return mLList;}; 

};

#endif
