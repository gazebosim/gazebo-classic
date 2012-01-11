/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Middleman between OGRE and Gazebo
 * Author: indepedentCreations@gmail.com
 * Date: 13 Feb 2006
 */

#ifndef MOVABLETEXT_HH
#define MOVABLETEXT_HH

// TODO: remove this line
#include <OGRE/OgreMovableObject.h>
#include <OGRE/OgreRenderable.h>

#include <string>

#include "common/CommonTypes.hh"
#include "common/Color.hh"
#include "math/MathTypes.hh"

namespace boost
{
  class recursive_mutex;
}

namespace gazebo
{
	namespace rendering
  {
    /// \addtogroup gazebo_rendering 
    /// \{
 
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
      public: void Load(const std::string &_name, 
                        const std::string &_text, 
                        const std::string &_fontName = "Arial", 
                        float _charHeight = 1.0,
                        const common::Color &_color = common::Color::White);
    
      /// \brief Set the font
      public: void SetFontName(const std::string &font);
              
      /// \brief Get the font
      public: const std::string &GetFont() const;
    
      /// \brief Set the text to display
      public: void SetText(const std::string & caption);
    
      /// \brief Get the displayed text
      public: const std::string &GetText() const;
    
      /// \brief Set the text color
      public: void SetColor(const common::Color &_color);
    
      /// \brief Get the text color
      public: const common::Color &GetColor() const; 
    
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
      public: math::Box GetAABB();
   
      /// \brief Update the text
      public: void Update();

      /// \brief Method to allow a caller to abstractly iterate over the
      //         Renderable instances
      public: virtual void visitRenderables(Ogre::Renderable::Visitor* visitor,
                  bool debug = false );
  
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
      private: std::string text;
    
      private: common::Color color;
      private: Ogre::RenderOperation renderOp;
      private: Ogre::AxisAlignedBox *aabb;
      private: Ogre::LightList lightList;
    
      private: float charHeight;
    
      private: bool needUpdate;
    
      private: float radius;
    
      private: Ogre::Camera *camera;
      private: Ogre::RenderWindow *renderWindow;
      private: Ogre::Font *font;
      private: Ogre::MaterialPtr material;
      private: Ogre::MaterialPtr backgroundMaterial;
 
      private: float viewportAspectCoef;
      private: float spaceWidth;
      private: bool updateColors;
      private: VertAlign vertAlign;
      private: HorizAlign horizAlign;
      private: bool onTop;
      private: float baseline;
   
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
      protected: const Ogre::LightList &getLights(void) const; 

      private: bool dirty;
  
      private: boost::recursive_mutex *mutex;
      private: Ogre::SimpleRenderable *renderable;
    };
    /// \}
  }

}
#endif
