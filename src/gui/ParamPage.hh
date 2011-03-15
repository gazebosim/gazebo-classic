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
#ifndef PARAMPAGE_HH
#define PARAMPAGE_HH

#include <wx/panel.h>

namespace gazebo
{
	namespace gui
  {
    class ParamPage : public wxPanel
    {
      public: ParamPage( wxWindow *parent ) : wxPanel( parent, wxID_ANY) {}
      public: virtual ~ParamPage() {}
  
      public: virtual void Apply() = 0;
    };
  }
}
#endif
