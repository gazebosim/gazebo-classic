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
#ifndef GAZEBO_RENDERING_MOVABLETEXT_HH_
#define GAZEBO_RENDERING_MOVABLETEXT_HH_

#include <memory>
#include <string>

#include <ignition/math/Box.hh>
#include <ignition/math/Color.hh>

// TODO: remove this line
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Color.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    class MovableTextPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class MovableText MovableText.hh rendering/rendering.hh
    /// \brief Movable text
    class GZ_RENDERING_VISIBLE MovableText
      : public Ogre::MovableObject, public Ogre::Renderable
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
      /// \deprecated See function that accepts ignition::math::Color.
      public: void Load(const std::string &_name,
                        const std::string &_text,
                        const std::string &_fontName,
                        float _charHeight,
                        const common::Color &_color)
                        GAZEBO_DEPRECATED(9.0);

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
                        const ignition::math::Color &_color =
                              ignition::math::Color::White);

      /// \brief Set the font. Valid fonts are defined on
      /// media/fonts/Gazebo.fontdef
      /// \param[in] _font Name of the font
      /// \sa FontName()
      public: void SetFontName(const std::string &_font);

      /// \brief Get the font name
      /// \return The font name
      /// \deprecated See FontName()
      public: const std::string &GetFont() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get the font name.
      /// \return The font name.
      /// \sa SetFontName()
      public: const std::string &FontName() const;

      /// \brief Set the text to display.
      /// \param[in] _text The text to display.
      /// \sa Text()
      public: void SetText(const std::string &_text);

      /// \brief Get the displayed text.
      /// \return The displayed text.
      /// \deprecated See Text()
      public: const std::string &GetText() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get the displayed text.
      /// \return The displayed text.
      /// \sa SetText()
      public: const std::string &Text() const;

      /// \brief Set the text color.
      /// \param[in] _color Text color.
      /// \deprecated See function that accepts ignition::math::Color.
      public: void SetColor(const common::Color &_color) GAZEBO_DEPRECATED(9.0);

      /// \brief Set the text color.
      /// \param[in] _color Text color.
      /// \sa Color()
      public: void SetColor(const ignition::math::Color &_color);

      /// \brief Get the text color.
      /// \return Texture color.
      /// \deprecated See function that returns ignition::math::Color.
      public: const common::Color GetColor() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get the text color.
      /// \return Text color.
      /// \sa SetColor()
      public: const ignition::math::Color &Color() const;

      /// \brief Set the height of the character in meters.
      /// \param[in] _height Height of the characters.
      /// \sa CharHeight()
      public: void SetCharHeight(const float _height);

      /// \brief Get the height of the characters in meters
      /// return Height of the characters.
      /// \deprecated See CharHeight.
      public: float GetCharHeight() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get the height of the characters in meters
      /// return Height of the characters.
      /// \sa SetCharHeight()
      public: float CharHeight() const;

      /// \brief Set the width of spaces between words.
      /// \param[in] _width Space width
      /// \sa SpaceWidth()
      public: void SetSpaceWidth(const float _width);

      /// \brief Get the width of spaces between words.
      /// \return Space width
      /// \deprecated See SpaceWidth()
      public: float GetSpaceWidth() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get the width of spaces between words.
      /// \return Space width
      /// \sa SetSpaceWidth()
      public: float SpaceWidth() const;

      /// \brief Set the alignment of the text
      /// \param[in] _hAlign Horizontal alignment
      /// \param[in] _vAlign Vertical alignment
      public: void SetTextAlignment(const HorizAlign &_hAlign,
                                    const VertAlign &_vAlign);

      /// \brief Set the baseline height of the text
      /// \param[in] _height Baseline height
      /// \sa Baseline()
      public: void SetBaseline(const float _height);

      /// \brief Get the baseline height in meters.
      /// \return Baseline height
      /// \deprecated See Baseline().
      public: float GetBaseline() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get the baseline height in meters.
      /// \return Baseline height
      /// \sa SetBaseline()
      public: float Baseline() const;

      /// \brief True = text always is displayed ontop.
      /// \param[in] _show Set to true to render the text on top of
      /// all other drawables.
      /// \sa ShowOnTop()
      public: void SetShowOnTop(const bool _show);

      /// \brief True = text is displayed on top.
      /// \return True if MovableText::SetShownOnTop(true) was called.
      /// \deprecated See ShowOnTop().
      public: bool GetShowOnTop() const GAZEBO_DEPRECATED(9.0);

      /// \brief Get whether the is displayed above other objects.
      /// \return True if it is on top.
      /// \sa SetShowOnTop()
      public: bool ShowOnTop() const;

      /// \brief Get the axis aligned bounding box of the text.
      /// \return The axis aligned bounding box.
      public: ignition::math::Box AABB();

      /// \brief Update the text.
      public: void Update();

      /// \internal
      /// \brief Method to allow a caller to abstractly iterate over the
      /// renderable instances.
      /// \param[in] _visitor Renderable instances to visit
      /// \param[in] _debug True if set to debug
      public: virtual void visitRenderables(Ogre::Renderable::Visitor *_visitor,
          bool _debug = false) override;

      /// \brief Setup the geometry.
      /// \deprecated Use SetupGeometry instead
      protected: void _setupGeometry() GAZEBO_DEPRECATED(9.0);

      /// \brief Setup the geometry.
      protected: void SetupGeometry();

      /// \brief Update colors.
      /// \deprecated Use UpdateColors instead
      protected: void _updateColors() GAZEBO_DEPRECATED(9.0);

      /// \brief Update colors.
      protected: void UpdateColors();

      /// \internal
      /// \brief Get the world transform (from MovableObject)
      protected: void getWorldTransforms(Ogre::Matrix4 *_xform) const override;

      /// \internal
      /// \brief Get the bounding radiu (from MovableObject)
      protected: float getBoundingRadius() const override;

      /// \internal
      /// \brief Get the squared view depth (from MovableObject)
      protected: float getSquaredViewDepth(const Ogre::Camera *_cam) const
          override;

      /// \internal
      /// \brief Get the render operation
      protected: void getRenderOperation(Ogre::RenderOperation &_op) override;

      /// \internal
      /// \brief Get the material
      protected: const Ogre::MaterialPtr &getMaterial() const override;

      /// \internal
      /// \brief Get the lights
      /// \deprecated Function has never returned meaningful values
      protected: const Ogre::LightList &getLights() const override;

      /// \internal
      private: const Ogre::AxisAlignedBox &getBoundingBox() const override;

      /// \internal
      private: const Ogre::String &getMovableType() const override;

      /// \internal
      private: void _notifyCurrentCamera(Ogre::Camera *_cam) override;

      /// \internal
      private: void _updateRenderQueue(Ogre::RenderQueue *_queue) override;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<MovableTextPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
