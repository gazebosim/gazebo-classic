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
#include <string>

class MovableText : public Ogre::MovableObject, public Ogre::Renderable
{
  public: enum HorizAlign {H_LEFT, H_CENTER};
  public: enum VertAlign  {V_BELOW, V_ABOVE};

  /// \brief Constructor
  public: MovableText(const std::string &name, 
                      const Ogre::UTFString &text, 
                      const std::string fontName = "Arial", 
                      float charHeight = 1.0,
                      const Ogre::ColourValue &color = Ogre::ColourValue::White);
  /// \brief Destructor
  public: virtual ~MovableText();

  /// \brief Set the font
  public: void SetFontName(const std::string &font);
          
  /// \brief Get the font
  public: const std::string &GetFont() const;

  /// \brief Set the text to display
  public: void SetText(const Ogre::UTFString & caption);

  /// \brief Get the displayed text
  public: const Ogre::UTFString & GetText() const;

  /// \brief Set the text color
  public: void SetColor(const Ogre::ColourValue & color);

  /// \brief Get the text color
  public: const Ogre::ColourValue & GetColor() const; 

  /// \brief Set the height of a character
  public: void SetCharHeight(float height);

  /// \brief Set the height of a characters
  public: float GetCharHeight() const;

  /// \brief Set the width of a space
  public:void SetSpaceWidth(float width);

  /// \brief Get the width of a space
  public: float GetSpaceWidth() const;

  /// \brief Set the alignment of the text
  public: void SetTextAlignment(const HorizAlign &hAlign, 
                                const VertAlign &vAlign); 

  /// \brief Set the baseline height of the text
  public: void SetBaseline(float height);

  /// \brief Get the baseline height
  public: float GetBaseline() const;

  /// \brief True=text always is displayed ontop
  public: void SetShowOnTop(bool show);

  /// \brief True=text is displayed on top
  public: bool GetShowOnTop() const;

  /// \brief Get the axis aligned bounding box of the text
  public: Ogre::AxisAlignedBox GetAABB();


  // from MovableText, create the object
  protected: void _setupGeometry();
  protected: void _updateColors();

    // from MovableObject
  protected: void getWorldTransforms(Ogre::Matrix4 *xform) const;
  protected: float getBoundingRadius() const;
  protected: float getSquaredViewDepth(const Ogre::Camera *cam) const;

  private: float viewportAspectCoef;

  private: std::string fontName;
  private: Ogre::UTFString text;

  private: HorizAlign horizAlign;
  private: VertAlign vertAlign;

  private: Ogre::ColourValue color;
  private: Ogre::RenderOperation renderOp;
  private: Ogre::AxisAlignedBox aabb;
  private: Ogre::LightList lightList;

  private: float charHeight;
  private: float spaceWidth;

  private: bool needUpdate;
  private: bool updateColors;
  private: bool onTop;

  private: float radius;
  private: float baseline;

  private: Ogre::Camera *camera;
  private: Ogre::RenderWindow *renderWindow;
  private: Ogre::Font *font;
  private: Ogre::MaterialPtr material;
  private: Ogre::MaterialPtr backgroundMaterial;

  private: const Ogre::Quaternion &getWorldOrientation(void) const;
  private: const Ogre::Vector3 &getWorldPosition(void) const;
  private: const Ogre::AxisAlignedBox &getBoundingBox(void) const;

  private: const Ogre::String &getMovableType() const;

  private: void _notifyCurrentCamera(Ogre::Camera *cam);
  private: void _updateRenderQueue(Ogre::RenderQueue* queue);

    // from renderable
  protected: void getRenderOperation(Ogre::RenderOperation &op);
  protected: const Ogre::MaterialPtr &getMaterial(void) const;
  protected: const Ogre::LightList &getLights(void) const; //{return mLList;}; 

};

#endif
