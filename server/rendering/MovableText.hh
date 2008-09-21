
/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Middleman between OGRE and Gazebo
 * Author: indepedentCreations@gmail.com
 * Date: 13 Feb 2006
 * CVS: $Id$
 */

#ifndef MOVABLETEXT_HH
#define MOVABLETEXT_HH

#include <Ogre.h>
#include <string>

/// \brief Movable text
class MovableText : public Ogre::MovableObject, public Ogre::Renderable
{
  /// \brief Horizontal alignment
  public: enum HorizAlign {H_LEFT, H_CENTER};

  /// \brief vertical alignment
  public: enum VertAlign  {V_BELOW, V_ABOVE};

  /// \brief Constructor
  public: MovableText();

  /// \brief Destructor
  public: virtual ~MovableText();

  /// \brief Loads text and font info 
  public: void Load(const std::string &name, 
                      const Ogre::UTFString &text, 
                      const std::string &fontName = "Arial", 
                      float charHeight = 1.0,
                      const Ogre::ColourValue &color = Ogre::ColourValue::White);

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

  /// \brief setup the geometry (from MovableText)
  protected: void _setupGeometry();

  /// \brief update the color(from MovableText)
  protected: void _updateColors();

  /// \brief Get the world transform (from MovableObject)
  protected: void getWorldTransforms(Ogre::Matrix4 *xform) const;

  /// \brief Get the bounding radiu (from MovableObject)
  protected: float getBoundingRadius() const;

  /// \brief Get the squared view depth (from MovableObject)
  protected: float getSquaredViewDepth(const Ogre::Camera *cam) const;

  private: std::string fontName;
  private: Ogre::UTFString text;

  private: Ogre::ColourValue color;
  private: Ogre::RenderOperation renderOp;
  private: Ogre::AxisAlignedBox aabb;
  private: Ogre::LightList lightList;

  private: float charHeight;

  private: bool needUpdate;

  private: float radius;

  private: Ogre::Camera *camera;
  private: Ogre::RenderWindow *renderWindow;
  private: float viewportAspectCoef;
  private: Ogre::Font *font;
  private: float spaceWidth;
  private: bool updateColors;
  private: VertAlign vertAlign;
  private: HorizAlign horizAlign;
  private: bool onTop;
  private: float baseline;
  private: Ogre::MaterialPtr material;
  private: Ogre::MaterialPtr backgroundMaterial;

  private: const Ogre::Quaternion &getWorldOrientation(void) const;
  private: const Ogre::Vector3 &getWorldPosition(void) const;
  private: const Ogre::AxisAlignedBox &getBoundingBox(void) const;

  private: const Ogre::String &getMovableType() const;

  private: void _notifyCurrentCamera(Ogre::Camera *cam);
  private: void _updateRenderQueue(Ogre::RenderQueue* queue);

  /// \brief Get the render operation
  protected: void getRenderOperation(Ogre::RenderOperation &op);

  /// \brief Get the material
  protected: const Ogre::MaterialPtr &getMaterial(void) const;

  /// \brief Get the lights
  protected: const Ogre::LightList &getLights(void) const; //{return mLList;}; 

};

#endif
