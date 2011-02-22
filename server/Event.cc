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

#include <iostream>
#include "Event.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
Connection::Connection(Event *e, int i) 
  : event(e), id(i) 
{
}

////////////////////////////////////////////////////////////////////////////////
Connection::~Connection()
{ 
  if (this->event && this->id >= 0)
  {
    ConnectionPtr self(this);
    this->event->Disconnect( self ); 
  }
}

////////////////////////////////////////////////////////////////////////////////
int Connection::Id() const
{ 
  return this->id; 
}
