/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GRID_HH
#define GRID_HH

#include <stdint.h>
#include <vector>

#include <OGRE/OgreMaterial.h>

#include "Param.hh"
#include "Color.hh"

namespace Ogre
{
  class SceneManager;
  
  class ManualObject;
  class SceneNode;
  
  class Any;
}

namespace gazebo
{

  class Scene;
  
  /**
   * \class Grid
   * \brief Displays a grid of cells, drawn with lines
   *
   * Displays a grid of cells, drawn with lines.  A grid with an identity orientation is drawn along the XZ plane.
   */
  class Grid
  {
    /// \brief Constructor
    ///
    /// \param Scene The scene this object is part of
    /// \param cell_count The number of cells to draw
    /// \param cell_length The size of each cell
    /// \param r Red color component, in the range [0, 1]
    /// \param g Green color component, in the range [0, 1]
    /// \param b Blue color component, in the range [0, 1]
    public: Grid( Scene *scene, uint32_t cell_count, float cell_length, 
                  float line_width, const Color &color );

    /// \brief Destructor
    public: ~Grid();

    public: void Init();

    
    /// \brief Get the Ogre scene node associated with this grid
    /// \return The Ogre scene node associated with this grid
    public: Ogre::SceneNode* GetSceneNode() { return this->sceneNode; }
    
    /// \brief Sets user data on all ogre objects we own
    public: void SetUserData( const Ogre::Any& data );
    
    public: void SetColor(const Color& color);
    public: Color GetColor() { return **this->colorP; }

    public: void SetCellCount(uint32_t count);
    public: float GetCellCount() { return **this->cellCountP; }

    public: void SetCellLength(float len);
    public: float GetCellLength() { return **this->cellLengthP; }

    public: void SetLineWidth(float width);
    public: float GetLineWidth() { return **this->lineWidthP; }

    public: void SetHeight(uint32_t count);
    public: uint32_t GetHeight() { return this->height; }
    
    private: void Create();

    private: Ogre::SceneNode* sceneNode;           ///< The scene node that this grid is attached to
    private: Ogre::ManualObject* manualObject;     ///< The manual object used to draw the grid
    
    private: Ogre::MaterialPtr material;

    private: ParamT<unsigned int> *cellCountP;
    private: ParamT<float> *cellLengthP;
    private: ParamT<float> *lineWidthP;
    private: ParamT<Color> *colorP;
    private: ParamT<float> *h_offsetP;
    private: std::vector<Param*> parameters;

    private: std::string name;
    private: unsigned int height;

    private: Scene *scene;
  };

}

#endif
