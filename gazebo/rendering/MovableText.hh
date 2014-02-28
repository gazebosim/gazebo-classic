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
/* Desc: Middleman between OGRE and Gazebo
 * Author: indepedentCreations@gmail.com
 * Date: 13 Feb 2006
 */

#ifndef _MOVABLETEXT_HH_
#define _MOVABLETEXT_HH_

#include <string>

// TODO: remove this line
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/math/MathTypes.hh"

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

    /// \class MovableText MovableText.hh rendering/rendering.hh
    /// \brief Movable text
    class MovableText : public Ogre::MovableObject, public Ogre::Renderable
    {
      /// \enum HorizAlign
      /// \brief Horizontal alignment
      public: enum HorizAlign {
                /// \brief Left alignment
                H_LEFT,
                /// \brief Center alignment
                H_CENTER
              };

      /// \enum VertAlign
      /// \brief vertical alignment
      public: enum VertAlign {
                /// \brief Align below
                V_BELOW,
                /// \brief Align above
                V_ABOVE
              };

      /// \brief Constructor
      public: MovableText();

      /// \brief Destructor
      public: virtual ~MovableText();

      /// \brief Loads text and font info
      /// \param[in] _name Name of the text object
      /// \param[in] _text Text to render
      /// \param[in] _fontName Font to use
      /// \param[in] _charHeight Height of the characters
      /// \param[in] _color Text color
      public: void Load(const std::string &_name,
                        const std::string &_text,
                        const std::string &_fontName = "Arial",
                        float _charHeight = 1.0,
                        const common::Color &_color = common::Color::White);

      /// \brief Set the font
      /// \param[in] _font Name of the font
      public: void SetFontName(const std::string &_font);

      /// \brief Get the font
      /// \return The font name
      public: const std::string &GetFont() const;

      /// \brief Set the text to display.
      /// \param[in] _text The text to display.
      public: void SetText(const std::string &_text);

      /// \brief Get the displayed text.
      /// \return The displayed text.
      public: const std::string &GetText() const;

      /// \brief Set the text color.
      /// \param[in] _color Text color.
      public: void SetColor(const common::Color &_color);

      /// \brief Get the text color.
      /// \return Texture color.
      public: const common::Color &GetColor() const;

      /// \brief Set the height of a character.
      /// \param[in] _height Height of the characters.
      public: void SetCharHeight(float _height);

      /// \brief Set the height of a characters
      /// return Height of the characters.
      public: float GetCharHeight() const;

      /// \brief Set the width of a space.
      /// \param[in] _width space width
      public:void SetSpaceWidth(float _width);

      /// \brief Get the width of a space
      /// \return Space width
      public: float GetSpaceWidth() const;

      /// \brief Set the alignment of the text
      /// \param[in] _hAlign Horizontal alignment
      /// \param[in] _vAlign Vertical alignment
      public: void SetTextAlignment(const HorizAlign &_hAlign,
                                    const VertAlign &_vAlign);

      /// \brief Set the baseline height of the text
      /// \param[in] _height Baseline height
      public: void SetBaseline(float _height);

      /// \brief Get the baseline height
      /// \return Baseline height
      public: float GetBaseline() const;

      /// \brief True = text always is displayed ontop.
      /// \param[in] _show Set to true to render the text on top of
      /// all other drawables.
      public: void SetShowOnTop(bool _show);

      /// \brief True = text is displayed on top.
      /// \return True if MovableText::SetShownOnTop(true) was called.
      public: bool GetShowOnTop() const;

      /// \brief Get the axis aligned bounding box of the text.
      /// \return The axis aligned bounding box.
      public: math::Box GetAABB();

      /// \brief Update the text.
      public: void Update();

      /// \internal
      /// \brief Method to allow a caller to abstractly iterate over the
      /// renderable instances.
      /// \param[in] _visitor Renderable instances to visit
      /// \param[in] _debug True if set to debug
      public: virtual void visitRenderables(Ogre::Renderable::Visitor* _visitor,
                  bool _debug = false);

      /// \internal
      /// \brief setup the geometry (from MovableText)
      protected: void _setupGeometry();

      /// \internal
      /// \brief update the color(from MovableText)
      protected: void _updateColors();

      /// \internal
      /// \brief Get the world transform (from MovableObject)
      protected: void getWorldTransforms(Ogre::Matrix4 *xform) const;

      /// \internal
      /// \brief Get the bounding radiu (from MovableObject)
      protected: float getBoundingRadius() const;

      /// \internal
      /// \brief Get the squared view depth (from MovableObject)
      protected: float getSquaredViewDepth(const Ogre::Camera *cam) const;

      /// \internal
      private: const Ogre::Quaternion &getWorldOrientation(void) const;
      /// \internal
      private: const Ogre::Vector3 &getWorldPosition(void) const;
      /// \internal
      private: const Ogre::AxisAlignedBox &getBoundingBox(void) const;
      /// \internal
      private: const Ogre::String &getMovableType() const;

      /// \internal
      private: void _notifyCurrentCamera(Ogre::Camera *cam);

      /// \internal
      private: void _updateRenderQueue(Ogre::RenderQueue* queue);

      /// \internal
      /// \brief Get the render operation
      protected: void getRenderOperation(Ogre::RenderOperation &op);

      /// \internal
      /// \brief Get the material
      protected: const Ogre::MaterialPtr &getMaterial(void) const;

      /// \internal
      /// \brief Get the lights
      protected: const Ogre::LightList &getLights(void) const;

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

      private: bool dirty;

      private: boost::recursive_mutex *mutex;
      private: Ogre::SimpleRenderable *renderable;
    };
    /// \}
  }
}
#endif
