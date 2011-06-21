/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Josh Faust */

#ifndef SDF_COLOR_HH
#define SDF_COLOR_HH

#include <string>
#include <vector>
#include <math.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "common/Console.hh"

namespace sdf
{

  class Color
  {
    public: Color() {this->Clear();}
    public: float r;
    public: float g;
    public: float b;
    public: float a;

    public: void Clear()
            {
              r = g = b = 0.0f;
              a = 1.0f;
            }

            /// color
    public: bool Init(const std::string &_vectorStr)
            {
              this->Clear();

              std::vector<std::string> pieces;
              std::vector<float> rgba;
              boost::split( pieces, _vectorStr, boost::is_any_of(" "));

              for (unsigned int i = 0; i < pieces.size(); ++i)
              {
                if (!pieces[i].empty())
                {
                  try
                  {
                    rgba.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
                  }
                  catch (boost::bad_lexical_cast &e)
                  {
                    gzerr << "color rgba element (" << pieces[i] 
                      << ") is not a valid float\n";
                    return false;
                  }
                }
              }

              if (rgba.size() != 4)
              {
                gzerr << "Color contains " << rgba.size() 
                  << " elements instead of 4 elements\n";
                return false;
              }

              this->r = rgba[0];
              this->g = rgba[1];
              this->b = rgba[2];
              this->a = rgba[3];

              return true;
            }

    public: void Print(const std::string &prefix)
            {
              std::cout << prefix << *this;
            }
  public: friend std::ostream &operator<<( std::ostream &_out, 
                                           const Color &_color)
          {
            _out << "rgba='" << _color.r << " " << _color.g << " " 
                 << _color.b << " " << _color.a << "'";
            return _out;
          }
 
  };
}

#endif
