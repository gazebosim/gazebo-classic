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
#ifndef FOGPAGE_HH
#define FOGPAGE_HH

#include <wx/valtext.h>
#include <wx/combobox.h>
#include <wx/clrpicker.h>

#include "gui/ParamPage.hh"

namespace gazebo
{
	namespace gui
  {
    class FogPage : public ParamPage
    {
      /// \brief Constructor
      public: FogPage(wxWindow *parent);
  
      /// \brief Destructor
      public: virtual ~FogPage();
  
      /// \brief Apply the current parameters
      public: virtual void Apply();
  
      private: wxComboBox *typeBox;
      private: wxTextCtrl *startCtrl;
      private: wxTextCtrl *endCtrl;
      private: wxTextCtrl *densityCtrl;
      private: wxColourPickerCtrl *colorCtrl;
    };
  }

}
#endif
